# 交錯判定システム 仕様 (rev.3)

## 変更点

**rev.2 → rev.3**
- 実装コードを本文から分離し、[[CONFLICT]] フォルダの `*.vfl` に移動。仕様書は設計と根拠のみを記述する

**rev.1 → rev.2**
- 交錯点の抽出を `nearpoints` の距離判定から **Intersection Analysis による真の交点** に変更
- レーン照合を `road_name`（文字列）から **`lane_id`（int）** に統一
- 先読みを `next_road` 経由から **`prev_lane` を使った SOP 側での解決** に変更
- 交点の対称データ（road_a / road_b）から、**レーン prim 側の非対称データ**（自分から見た相手）へ焼き戻す工程を追加

---

## 1. 目的

信号だけでは表現できない交差点内の挙動を扱う。右折車が対向直進車の流れを見て「待つ / 出る」を判断する。左側通行を前提とする。

解消すべき欠陥は 2 つ。

- 交錯点の近傍でしか相手車を探しておらず、高速で接近する車を検出できない
- 静的な道路幾何をランタイムで毎フレーム再計算している

---

## 2. 設計原則

既存の 3 層構造を維持する。

| 層 | ノード | 責務 |
|---|---|---|
| 検出 | CONFLICT DETECT | attribute に書くだけ。減速しない |
| 判断 | STATE | 信号・歩行者・前車・交錯を比較し、最も近い制約を選ぶ |
| 実行 | ACCEL / INTEGRATE | 選ばれた制約に従って加減速 |

加えて 3 つ。

- **静的な情報は SOP で焼く。** 道路網は不変なので、レーン間の交錯関係は事前計算し prim attribute に持たせる
- **ランタイムでは「自分から見た相手」だけを引く。** 交点の対称データは SOP で非対称化し、「自分は A か B か」の判定をランタイムから消す
- **文字列比較を使わない。** レーンの同定は `lane_id`（int）で行う。`nearpoints` のグループ指定は定数に限る（KD-tree 再構築の回避）

---

## 3. データフロー

```
[SOP] CONNECT_LANE                    turn を prim attribute に追加
[SOP] Intersection Analysis           交点ポイントを生成
[SOP] CONFLICT POINT ATTRIB           交点に交わる2レーンと手前レーンを焼く
[SOP] BAKE CONFLICTS TO LANE          レーン prim に「自分から見た相手」を焼く
[DOP] GET ATTRIBUTE (road_changed時)  レーン prim から車へ転写
[DOP] CONFLICT DETECT (毎フレーム)     dist_to_conflict / yield_conflict を判定
[DOP] STATE → ACCEL
```

`prev_lane` を SOP で解決することで、手前の直線レーン prim にも `cross_lanes` が乗る。右折車は交差点の手前を走っている段階で交錯情報を持つため、ランタイムでの先読み処理が不要になる。

---

## 4. SOP 側の仕様

### 4.1 交点の生成と除外

Intersection Analysis で 2 本のスプラインの真の交点を得る。距離ベースの近傍判定と異なり、交差していない併走レーンを誤検出せず、ポイント刻みが粗くても取りこぼさない。

`laneA == laneB` の交点は **合流** であり交差ではないため除外する（合流の車間制御は CAR DETECT / IDM の守備範囲）。

### 4.2 交点が持つ情報

| attribute | 型 | 意味 |
|---|---|---|
| `candidate` | int[2] | 交わる 2 レーンの `lane_id` |
| `prev_a` / `prev_b` | int | 各レーンの手前レーンの `lane_id`（先読み用） |
| `P` | vector | 交点座標 |

**int で持つこと。** `"road_" + itoa(...)` の文字列化は不要。

### 4.3 レーン prim への焼き戻し

交点の対称データを、レーンごとの非対称データに変換する。

| attribute | 型 | 意味 |
|---|---|---|
| `cross_lanes` | int[] | このレーンと交錯する **相手** レーンの `lane_id` |
| `cross_pos` | vector[] | 対応する交点座標（同一 index で 1:1 対応） |

**重複排除をしない。** 同じ相手レーンと 2 回交わる形状があり得るため、全ペアを保持する。「前方の最寄り」の選択はランタイム側の責務。

---

## 5. 判定アルゴリズム

### 5.1 早期打ち切り

以下のいずれかで即座に return する。処理順が意味を持つ。

1. 交錯点を持たない → `cross_committed` と `wait_time` をリセット
2. 前方に残る交錯点がない（全通過）→ 同上リセット。**これがコミット解除の唯一の場所**
3. `s@turn != "right"` → 直進車は譲らない
4. `i@cross_committed == 1` → 判定済み。渡り切るまで再評価しない

### 5.2 到達時間の比較

距離ではなく到達時間（TTA）で判定する。

```
t_self  = 交錯点までの距離 / max(自車速度, cross_speed)
t_other = 交錯点までの距離 / max(相手速度, v_floor)
```

`t_self` の分母に `cross_speed` の下限を置くのが要点。停止中に実速度で割ると `t_self` が発散し、永久に発車できなくなる。「今出たら何秒で到達するか」を評価する必要がある。

**塞がれている条件（片側判定のみ。相手が先に通過するケースは阻害要因にならない）:**

```
t_other < t_self + need_gap
```

### 5.3 相手車のフィルタ

- `lane_id` が交錯相手レーンと一致
- `car_state != 2`（停止中の車は来ない。赤信号待ちの対向車で永久に出られなくなるのを防ぐ）
- 交錯点に向かって進行中（`dot(o_to_cp, o_dir) > 0`）

### 5.4 Gap acceptance

待ち時間に応じて許容ギャップを縮める。固定閾値だと交通量が増えたとき右折車が永久に出られない。

```
need_gap = fit(wait_time, 0, patience, base_gap, min_gap)
```

隙間が空いたときは `wait_time` を 2 倍速で減衰させる。一瞬の空きで忍耐がリセットされるのを防ぐ。

### 5.5 コミット

`blocked == 0` かつ交錯点まで `commit_dist` 以内になったら `cross_committed = 1`。以降は再判定しない。

交差点内で停止するのが最悪の状態であるため、一度出ると決めたら渡り切る。この設計は同時に、譲る / 譲らないの振動（チャタリング）を構造的に排除する。

---

## 6. インターフェース

### 入力（読む）

| attribute | 型 | 供給元 |
|---|---|---|
| `my_conflicts` | int[] | GET ATTRIBUTE |
| `my_conf_pos` | vector[] | GET ATTRIBUTE |
| `lane_id` | int | GET ATTRIBUTE |
| `turn` | string | GET ATTRIBUTE |
| `dir`, `vel`, `car_state`, `car` | — | 各検出ノード |

### 出力（書く）

| attribute | 型 | 意味 |
|---|---|---|
| `dist_to_conflict` | float | 譲る対象の交錯点までの距離 |
| `yield_conflict` | int | 譲る必要があるか |
| `cross_committed` | int | 進入決定済み |
| `wait_time` | float | 連続待機時間 |

---

## 7. パラメータ

| 名前 | 初期値 | 場所 |
|---|---|---|
| `conflict_scan` | 80.0 m | DOP |
| `base_gap` | 3.5 s | DOP |
| `min_gap` | 1.2 s | DOP |
| `patience` | 12.0 s | DOP |
| `cross_speed` | 5.0 m/s | DOP |
| `commit_dist` | 4.0 m | DOP |

Intersection Analysis を使うため、rev.1 にあった `conflict_radius` / `parallel_dot` は不要。

---

## 8. 実装

コードは [[CONFLICT]] フォルダを参照。

| ファイル | ノード | 内容 |
|---|---|---|
| `01_CONFLICT_POINT_ATTRIB.vfl` | SOP (Run Over: Points) | 交点に交わる2レーン・手前レーンを焼く（4.2） |
| `02_BAKE_CONFLICTS_TO_LANE.vfl` | SOP (Run Over: Primitives) | レーン prim へ非対称データを焼く（4.3） |
| `03_CONNECT_LANE_ADD.vfl` | SOP・既存ノードへの追加 | `turn` を prim/point attribute として書き出す |
| `04_GET_ATTRIBUTE_ADD.vfl` | DOP・既存ノードへの追加 | `road_changed` 時にレーン prim から車へ転写 |
| `05_CONFLICT_DETECT.vfl` | DOP | 5章のアルゴリズム本体 |
| `06_STATE_ADD.vfl` | DOP・既存ノードへの追加 | 前車比較・制御ソースへの組み込み |
| `07_COLORIZE_ADD.vfl` | DOP・既存ノードへの追加 | 右折待ちの可視化（マゼンタ） |

---

## 9. 検証手順

段階的に繋ぐ。一括で接続すると切り分けができない。

1. **交点の数を確認** — `laneA == laneB` の除外で消えた数が妥当か。消えすぎ / 残りすぎは `lane_id` の付与ミスを示す
2. **`cross_lanes` の目視確認** — Geometry Spreadsheet でレーン prim を見る。特に **交差点手前の直線レーン** に交錯情報が乗っているかが先読みの成否を決める。空だと以降が静かに no-op になる
3. **`i@lane_id` と `s@turn` が車に乗っているか** — `turn` が空文字だと右折車が直進扱いになり、一切譲らなくなる
4. **COLORIZE のみ接続** — 減速させず、右折待ちの点灯タイミングだけ観察。対向直進車が来ているときだけ光るのが正解
5. **STATE 接続**

---

## 10. 未確認事項

実装前に確認が必要な箇所。

- **`prev_lane` の供給元。** CONNECT_LANE には `prev_lane` を書く行がない。別ノードで付与しているか要確認。無いと全レーンが `prev_lane = 0` で誤マッチする
- **直線 prim の `lane_id`。** CONNECT_LANE はコネクタにのみ `lane_id` を設定している。元の直線 prim 側にも乗っているか
- **`pointprims` が返す prim。** 交差点付近では複数レーンが近接する。`pr[0]` が意図したレーンか。CONNECT_LANE が `addpoint` で新規点を作っている（既存点を共有していない）なら問題ない
- **`s@turn` の先読み条件。** 直線レーンに `turn = "straight"` が乗る場合、`04_GET_ATTRIBUTE_ADD.vfl` の無条件上書きが正しいか。`next_road` が確定するタイミング（直線に入った時点か、コネクタ進入後か）にも依存する

---

## 11. 既知の制約

- **曲線 × 曲線の交錯を扱わない。** Input 2 が直線ジオメトリのため、コネクタ同士の交差が漏れる。左側通行では対向右折同士はすれ違うため問題ないが、**直交方向からの右折** は実際に交わる。Intersection Analysis をもう 1 系統（コネクタ × コネクタ）並べ、`sp[0] == sp[1]` の自己交差を除外したうえで配列を結合する必要がある
- **直進同士の交錯を扱わない。** 信号なし交差点を実装する場合、turn によるフィルタを外し、相手の `cross_committed` を参照する相互譲歩ルールが必要になる
- **黄色コミットとの競合。** `i@go == 1` の車が交錯判定で停止すると交差点内に取り残される。STATE 側で `i@go` を優先させる必要があるかもしれない
- **専用右折レーンを前提とする。** 右折待ちが直進車線上で発生すると後続を塞ぐ

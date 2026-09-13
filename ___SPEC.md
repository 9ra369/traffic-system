# 交通シミュレーションシステム 仕様書

Houdini 上で構築した手続き型の交通流シミュレーション。道路ネットワークを SOP で生成し、車両エージェントを DOP Network 内の VEX Wrangle 群で毎フレーム駆動する。

---

## 1. システム全体像

```
[SOP] 道路ネットワーク生成
        ├ 元となるレーン曲線（src_id / lane_name / dir / curv を持つ点群）
        └ CONNECT_LANE : 交差点に接続曲線（コネクタ）を生成し、next_lanes を配る
                    ↓
[DOP] 車両シミュレーション（毎フレーム）
        SET_ID → INIT → CHANGE_ROAD → GET_ATTRIBUTE
              → SIGNAL_DETECT → HUMAN_DETECT → CAR_DETECT
              → STATE → ACCEL → INTEGRATE → COLORIZE → DELETE_PT
```

設計上の中心的な考え方は次の 3 点。

1. **アトリビュート駆動** — 車両の状態、道路の同一性、ノード間の通信はすべて点アトリビュートで表現する。ノードは「読む」「書く」だけで、内部状態を持たない。
2. **検出と判断の分離** — `*_DETECT` 系ノードは周囲の状況をアトリビュートに書き出すだけ。何を優先し、どう減速するかは `STATE` と `ACCEL` が単独で決める。
3. **経路は幾何から引く** — 車両は自前の経路データを持たず、毎フレーム道路点群から最寄り点を引いて `dir` と `curv` を取得する。道路形状を変えればそのまま挙動に反映される。

---

## 2. ジオメトリと入力構成

車両 Wrangle が参照する入力は以下で統一されている。

|入力|内容|主な用途|
|---|---|---|
|Input 0|車両点群（自分自身）|前方車両の検出|
|Input 1|道路ライン（点群 + プリミティブ）|進行方向・曲率の取得、道路切替判定|
|Input 2|信号点群|信号状態の取得|
|Input 3|歩行者点群|横断中歩行者の検出|衝突ポイント|

---

## 3. アトリビュート辞典

### 3.1 道路ライン（Input 1）

点アトリビュート:

|名前|型|意味|
|---|---|---|
|`dir`|vector|その地点での進行方向|
|`curv`|float|曲率を 0–1 に正規化した値。0 = 直線、1 = 最も急なカーブ|
|`src_id`|int|スプライン 1 本を一意に識別する整数|
|`lane_name`|string|論理的な「道路」の名前。同じ道路の複数スプライン（対向車線など）で共有され得る|
|`next_lanes`|string[]|この道路から進める次の道路名の候補リスト。区間の入口点に付与される|

プリミティブアトリビュート（CONNECT_LANE が生成するコネクタ）:

|名前|型|意味|
|---|---|---|
|`is_connector`|int|交差点内の接続曲線なら 1|
|`is_curved`|int|曲がるコネクタなら 1、直進なら 0|
|`lane_id` / `lane_index`|int|接続先レーンの識別子|
|`road`|int|接続元ラインから継承した道路 ID|
|`next_lane`|string|接続先の道路名|

プリミティブグループ: `connectors`, `curved`, `straight` 点グループ: `corner`（コネクタの中間制御点）

### 3.2 車両（Input 0）

|名前|型|書き込むノード|意味|
|---|---|---|---|
|`id`|float|SET_ID|乱数シード用の個体識別子|
|`car`|int|INIT|車両ジオメトリの判別フラグ（常に 1）|
|`initialized`|int|INIT|初期化済みフラグ|
|`lane_name`|string|CHANGE_ROAD|現在走行中の道路名|
|`next_lane_name`|string|GET_ATTRIBUTE|次に進む道路名|
|`src_id`|int|GET_ATTRIBUTE|**廃止（2026-09-10）**。旧: 現在走行中のスプラインID。現行実装では`lane_id`に置き換え（詳細 → 5.4節の追記）|
|`lane_changed`|int|CHANGE_ROAD / GET_ATTRIBUTE|道路切替が発生し ID 再解決が必要なことを示すフラグ|
|`dir`|vector|GET_ATTRIBUTE|進行方向|
|`curv`|float|GET_ATTRIBUTE|現在地点の曲率|
|`car_state`|int|STATE / ACCEL|0 = CRUISING, 1 = BRAKING, 2 = STOPPED|
|`go`|int|STATE|1 = 黄信号を通過するとコミット済み|
|`yellow_judged`|int|STATE|1 = この黄信号に対する判断を実施済み|
|`ctrl_src`|int|STATE|減速要因。0 = 信号、2 = 歩行者、3 = 右折コンフリクト、-1 = なし|
|`search_radius`|float|SIGNAL_DETECT|全検出ノード共通の探索半径|
|`signal_ptnum` / `signal_state` / `dist_to_signal` / `yellow_remain`||SIGNAL_DETECT|前方信号の情報|
|`ped_ptnum` / `ped_crossing` / `dist_to_ped`||HUMAN_DETECT|前方歩行者の情報|
|`car_ptnum` / `dist_to_car` / `car_vel` / `car_signal` / `car_is_cross`||CAR_DETECT|前方車両の情報|
|`dist_to_target` / `effective_dist` / `required_decel`|float|STATE|減速目標までの距離と必要減速度|
|`max_speed` / `max_accel` / `max_decel`|float|INIT|個体ごとの物理パラメータ|
|`vel` / `accel`|float|ACCEL / INTEGRATE|速度・加速度|
|`dead`|int|DELETE_PT|削除対象フラグ|

### 3.3 信号（Input 2）

|名前|型|意味|
|---|---|---|
|`signal_state`|int|0 = 青、1 = 黄、2 = 赤。車両側の未検出は -1|
|`yellow_remain`|float|黄信号の残り秒数|
|`lane_name`|string|この信号が制御する道路名|

### 3.4 歩行者（Input 3）

|名前|型|意味|
|---|---|---|
|`ped_crossing`|int|1 = 横断中|
|`lane_name`|string|横断している道路の名前|

---

## 4. 道路ネットワーク生成

### CONNECT_LANE（Detail Wrangle）

交差点で切断されたレーン端点どうしを接続する曲線を生成する。

**入口／出口の収集** `is_cut_end == 1` の点のうち、`flow == 1` を入口点、それ以外を出口点として集める。

**接続の可否判定** 入口点ごとに、同じ `intersection_id` を持つ出口点を走査する。

- `dot(d1, d2) < -0.9` の組み合わせは U ターンとみなして除外
- 進行方向の関係から曲がり方向を分類する
    - `dot(d1, d2) > 0.95` → `straight`
    - `cross(d1, d2).y > 0` → `left`
    - それ以外 → `right`
- 入口点の `turn` アトリビュートが許す方向だけを採用する
    - `turn == "left"` → left / straight
    - `turn == "right"` → right / straight
    - それ以外 → straight のみ

**コネクタの生成** 採用された組み合わせごとにポリラインを 1 本作る。構成点は次の 2〜3 点。

1. 入口点位置 `p1`（`dir = d1`）
2. 曲がる場合のみ、2 直線の交点にあたる中間点。`n = cross(d1, d2)` として `t = dot(cross(p2 - p1, d2), n) / dot(n, n)` を求め `p1 + d1 * t` に配置する。`dir` は `normalize(d1 + d2)`
3. 出口点位置 `p2`（`dir = d2`）

**経路候補の配布** 接続先の道路名は `"road_" + itoa(lane_id)` の形式で作る。コネクタの入口点には自分の行き先 1 件だけを `next_lanes` として持たせ、元の入口点には全候補の配列を `next_lanes` として書き戻す。この配列が、走行中の車両が交差点でどちらへ進むかを選ぶ材料になる。

---

## 5. 車両シミュレーション

### 5.1 SET_ID

```c
f@id = @ptnum+1;
```

個体ごとの乱数シード。`INIT` の物理パラメータ生成と、`GET_ATTRIBUTE` の経路選択に使う。

### 5.2 INIT（初回のみ）

`i@initialized` をガードにした早期 return で、生成フレームに 1 回だけ走る。**全アトリビュートの初期値を定義する唯一の場所**であり、他ノードでのデフォルト値定義は行わない。

物理パラメータは個体ごとにランダム化する。

|パラメータ|範囲|
|---|---|
|`max_speed`|11.1 – 17.7 m/s|
|`max_accel`|0.7 – 1.2 m/s²|
|`max_decel`|0.9 – 1.5 m/s²|

### 5.3 CHANGE_ROAD — 道路の切り替え

現在の `lane_name` に一致するプリミティブだけを対象に `xyzdist` を実行し、そのスプライン上でのパラメータ `u` を得る。

```c
float d = xyzdist(1, s@lane_name, @P, hitprim, uvw);
if (hitprim >= 0) {
    @u = uvw.x;
    if (uvw.x >= 0.99) {
        s@lane_name    = s@next_lane_name;
        i@lane_changed = 1;
    }
}
```

終端に到達したら `lane_name` を `next_lane_name` に差し替え、`lane_changed` フラグを立てる。フラグを立てるだけで ID の再解決は行わず、次のノードに委ねる。

### 5.4 GET_ATTRIBUTE — 進行方向と曲率の取得

> **2026-09-10 追記**：本節は旧アーキテクチャ当時の記述で、以下`src_id`と書いてある箇所は
> 現行実装（[[03_UPDATE_LANE_ATTRIB]]）では`lane_id`に置き換わっている。`src_id`は分割・
> 交差点処理前の「元の構成ライン」のIDで、左右分割や複数車線化で複数の最終レーンが同じ値を
> 共有しうるため、`nearpoint`/`findattribval`の一致キーとしては不正確だった（対向車線・並走
> 車線を誤って拾う恐れ）。`lane_id`は分割・コネクタ生成後の最終スプラインを一意に識別する
> ため、これに置き換えた。以下の説明・コード例は考え方の参考として残すが、属性名は
> 読み替えること。

2 段構えになっている。

**第 1 段: 切替直後の ID 再解決** `lane_changed == 1` のとき、新しい `lane_name` に属する点群から `nearpoint` で最寄り点を引き、`src_id` を確定する。解決に成功したときだけフラグを下ろす。失敗した場合はフラグが立ったままになるので、次フレームで再試行される（リトライパターン）。

同時に、確定した `src_id` の入口点から `next_lanes` を読み、この先どこへ進むかを 1 回だけ決める。

```c
int n = len(nexts);
if (n == 1)      s@next_lane_name = nexts[0];
else if (n > 1)  s@next_lane_name = nexts[min(int(rand(@id + i@src_id) * n), n - 1)];
else             s@next_lane_name = "";
```

シードに `@id` と `src_id` の両方を混ぜているため、同じ交差点でも車両ごとに異なる選択をし、かつ同一フレーム内で結果がぶれない。

**第 2 段: 毎フレームの参照** `src_id` が確定していれば、そのスプライン上の最寄り点から `dir` と `curv` を読む。

```c
int pt = nearpoint(1, "@src_id=" + itoa(i@src_id), @P);
v@dir  = point(1, "dir",  pt);
f@curv = point(1, "curv", pt);
```

**実行順の制約**: CHANGE_ROAD → GET_ATTRIBUTE の順に置く必要がある。この順序により、切り替えが起きたフレームのうちに新しい道路の `dir` が解決され、1 フレーム分の遅延や進行方向の飛びが発生しない。

### 5.5 SIGNAL_DETECT — 前方信号の検出

**探索半径の決定** このノードが `search_radius` を設定し、以降の HUMAN_DETECT と CAR_DETECT がそれを共有する。速度が `max_speed` の 60% を超えると、制動距離の伸びに合わせて半径を 1.2〜1.5 倍に拡大する。

```c
f@search_radius = chf("search_radius");
if (f@vel > f@max_speed * 0.6)
    @search_radius *= clamp(f@vel / f@max_speed, 1.2, 1.5);
```

**採用条件** `nearpoints` で得た候補のうち、次を満たす最寄りの 1 点を選ぶ。

- `lane_name` が自車と一致する
- `dot(normalize(v@dir), normalize(to_signal)) > 0.5` — 進行方向前方にある

### 5.6 HUMAN_DETECT — 横断歩行者の検出

構造は SIGNAL_DETECT と同じ。採用条件は次の 3 つ。

- `ped_crossing == 1`（横断中）
- `lane_name` が自車と一致する
- `dot(v@dir, normalize(to_ped)) > 0.3` — 信号より広い前方コーンを使う

### 5.7 CAR_DETECT — 前方車両の検出

同レーンの先行車と、交差・右左折してくる車を 1 つのループで扱う。区別は `lane_name` の一致で行い、それぞれ異なる判定を適用する。

||同レーン|交差・右左折|
|---|---|---|
|前方コーン|`fwd_dot > 0.5`|`fwd_dot > 0.7`|
|横オフセット|制限なし|自車進路の中心から 2.0 m 以内|
|`car_is_cross`|0|1|

交差車に対してコーン判定だけを使うと、遠方で隣レーンの車を拾ってしまう。そのため自車進行方向への射影を取り、横方向の距離で追加の絞り込みを行う。

```c
float forward = dot(to_other, v@dir);
float lateral = length(to_other - forward * v@dir);
if (lateral > cross_half_w) continue;
```

**IDM に渡す速度** 交差車の速度はそのまま使えないため、自車の進行方向へ射影した成分だけを取り、負の場合は 0 に切り上げる。

```c
f@car_vel = is_cross ? max(v_other * dot(dir_other, v@dir), 0.0) : v_other;
```

### 5.8 STATE — 減速要因の調停と状態遷移

検出された 3 つの要因（前車 / 信号 / 歩行者）から、どれに従うかを決める。

**優先順位** 前車がいる場合、信号と歩行者のうち最も近いものと距離を比較する。前車の方が近ければ IDM に任せ、状態を CRUISING に戻して早期 return する。信号・歩行者の方が近ければ、以降の停止判断に進む。

**制御要因の確定** 赤（`signal_state == 2`）と歩行者は `ctrl_state = 2`（無条件停止）、黄（`signal_state == 1`）は `ctrl_state = 1`（通過判断あり）として扱い、距離の近い方を採用する。

**黄信号の通過判断** `yellow_judged` により、1 つの黄信号につき判断を 1 回だけ行う。通過可能距離を残り時間から見積もり、停止線まで届くなら通過にコミットする。

```c
float possible_dist = f@vel * f@yellow_remain - 6;
if (possible_dist >= f@dist_to_signal) { i@car_state = 0; i@yellow_commit = 1; }
else                                   { i@car_state = 1; i@yellow_commit = 0; ... }
```

一度 `go == 1` になると、信号が赤に変わっても判断を覆さない。停止線を越えた後に急停止して交差点内に取り残されるのを防ぐためである。信号を通過し `signal_state == -1`（検出なし）になった時点でコミットを解除する。

**必要減速度** 停止線の 1 m 手前を目標とし、等減速で停止できる減速度を求める。

```c
f@effective_dist = f@dist_to_target - 1;
f@required_decel = (f@vel * f@vel) / (2.0 * max(f@effective_dist, 0.01));
```

この計算は CRUISING から BRAKING へ遷移する瞬間にだけ行う。以後は同じ減速度を維持するので、実際の制動は等減速運動になる。

### 5.9 ACCEL — 加速度の決定

`car_state` ごとに異なるモデルを使う。

**CRUISING: Intelligent Driver Model**

$$a = a_{max}\left[1 - \left(\frac{v}{v_0}\right)^{\delta} - \left(\frac{s^_}{s}\right)^2\right], \quad s^_ = s_0 + vT + \frac{v \Delta v}{2\sqrt{ab}}$$

|記号|意味|値|
|---|---|---|
|`v0`|希望速度|`max_speed`|
|`T`|希望車頭時間|1.5 s（交差車には 1.0 s）|
|`s0`|最小車間距離|10 m（交差車には 5 m）|
|`a`|最大加速度|`max_accel`|
|`b`|快適減速度|`max_decel`|
|`delta`|加速指数|4.0|

交差車は事実上の静止障害物に近く、通常のパラメータでは手前で過剰に減速する。そのため `s0` と `T` を詰めて反応を遅らせている。

前方に車がいない場合は `v_lead = v0`、`s = 9999` として干渉項を実質無効化し、自由走行させる。また障害物が突然出現したときの発散を防ぐため、加速度に `-b * 2.0` の下限を設けている。

**BRAKING** `STATE` が求めた `required_decel` を適用する。停止線まで 1 m を切ったら 1.2 倍に強めて確実に止める。速度が 0.1 を下回った時点で速度を 0 にクランプし、STOPPED へ遷移する。

**STOPPED** 加速度・速度ともに 0 で固定する。

### 5.10 INTEGRATE — 積分と位置更新

```c
float curve_limit = fit(f@curv, 0, 1, f@max_speed, chf("min_curve_speed"));
f@vel += f@accel * @dt;
f@vel  = clamp(f@vel, 0.0, min(f@max_speed, curve_limit));
@v     = v@dir * f@vel;
@P    += @v * @dt;
```

道路点群から引いてきた `curv` を速度上限に変換することで、カーブでの減速が実現される。直線では `max_speed`、最も急なカーブでは `min_curve_speed` まで落ちる。IDM による加速度計算とは独立した上限クランプなので、両者は干渉しない。

位置は `dir` 方向へ進めるだけで、道路曲線への吸着（プロジェクション）は行っていない。曲線への追従精度は `dir` の解像度、すなわち道路点群の密度に依存する。

### 5.11 COLORIZE — デバッグ表示

|色|条件|
|---|---|
|緑|CRUISING かつ通常走行|
|黄|CRUISING だが前車の影響で減速中（`accel < 0`）|
|赤|CRUISING だが前車の影響で停止中（`vel < 0.1`）、または BRAKING / STOPPED|

IDM による減速は `car_state` が 0 のままなので、状態だけを色にすると停止していても緑になってしまう。それを避けるため、前車の有無と加速度・速度を組み合わせて判定している。

### 5.12 DELETE_PT — 車両の削除

自分と同じ `lane_name` のプリミティブに対して `xyzdist` を行い、`u > 0.98` なら `i@dead = 1` を立てる。

**注意点**: Houdini では点を削除してもプリミティブが残り、残った点どうしが再接続される。ライン中間の点を消す場合は、Add SOP を "Delete Geometry But Keep the Points" に設定するか、Primitive Wrangle で `removeprim(0, @primnum, 0)` を明示的に呼ぶ必要がある。

---

## 6. 状態機械のまとめ

```
        ┌──────────────────────────────────────────┐
        │            CRUISING (car_state = 0)      │
        │  IDM で加減速。前車追従もここに含まれる  │
        └────────┬─────────────────────────▲───────┘
                 │ 赤信号 / 歩行者 / 通過不可の黄  │
                 │ が前車より近い                  │ 要因が消えた
                 ▼                                 │
        ┌──────────────────────────────────────┐   │
        │            BRAKING (car_state = 1)   │───┘
        │  required_decel による等減速         │
        └────────┬─────────────────────────────┘
                 │ vel < 0.1
                 ▼
        ┌──────────────────────────────────────┐
        │            STOPPED (car_state = 2)   │
        │  完全停止。要因が消えると CRUISING へ│
        └──────────────────────────────────────┘
```

補助フラグ:

- `go` — 黄信号の通過にコミット済み。BRAKING への遷移を抑止する
- `yellow_judged` — 同一の黄信号に対する判断の重複を防ぐ
- `lane_changed` — 道路 ID の再解決要求

---

## 7. 調整可能なパラメータ

|パラメータ|位置|役割|
|---|---|---|
|`search_radius`|SIGNAL_DETECT|全検出ノード共通の探索半径|
|`min_curve_speed`|INTEGRATE|最も急なカーブでの速度下限|
|`same_dot` = 0.5|CAR_DETECT|同レーン先行車の前方コーン|
|`cross_dot` = 0.7|CAR_DETECT|交差車の前方コーン|
|`cross_half_w` = 2.0|CAR_DETECT|交差車判定の横方向許容幅 [m]|
|`stop_offset` = 1|STATE|停止線手前のマージン [m]|
|黄信号マージン = 6|STATE|通過可能距離から引く安全マージン [m]|
|`end_threshold` = 0.98|DELETE_PT|車両を削除するスプライン上の位置|

---

## 8. 既知の課題と未確定事項

### 8.1 `lane_name` が複数スプラインで共有される場合の誤解決

`GET_ATTRIBUTE` は `nearpoint` で `lane_name` から `src_id` を解決する。対向車線のように同じ `lane_name` を持つスプラインが複数ある場合、切替直後に誤ったスプラインへ吸着する可能性がある。

対策候補は方向一致チェックの追加。候補点の `dir` と現在の `v@dir` の内積を取り、`dot(candidate_dir, v@dir) > 0` を満たすものだけを採用すれば、幾何的に近くても逆向きのスプラインを排除できる。

### 8.2 `v@dir` の正規化が統一されていない

`GET_ATTRIBUTE` は道路点群の `dir` をそのまま `v@dir` に代入している。SIGNAL_DETECT は `normalize(v@dir)` と明示的に正規化するが、HUMAN_DETECT と CAR_DETECT は生の値を使う。

`dir` が単位長でない場合、コーン判定の閾値が意図した角度からずれるだけでなく、CAR_DETECT の横オフセット判定（`forward * v@dir` による射影）が成立しなくなる。`GET_ATTRIBUTE` で 1 回正規化しておくのが最も確実。

```c
v@dir = normalize(point(1, "dir", pt));
```

### 8.3 INIT で初期化されていないアトリビュート

「全アトリビュートの初期値を定義する唯一の場所」という原則に対し、現状 INIT には次が含まれていない。

`src_id` / `lane_changed` / `lane_name` / `next_lane_name` / `yellow_judged` / `effective_dist` / `required_decel` / `search_radius` / `car_is_cross` / `car_vel`

特に `src_id = -1`、`lane_changed = 1` を INIT に置くことには意味がある。生成直後の車両が交差点通過時とまったく同じコードパスで道路を解決することになり、スポーン専用の特殊処理が不要になる。

### 8.4 `max_accel` と `max_decel` のシードが同一

```c
f@max_accel = fit01(rand(@id + 0.31), 0.7, 1.2);
f@max_decel = fit01(rand(@id + 0.31), 0.9, 1.5);
```

両者が同じシード `@id + 0.31` を使っているため完全に相関する。加速の良い車は必ず制動も強い、という不自然な分布になっている。意図的でなければ片方のオフセットを変える。

### 8.5 命名の不一致

`CONNECT_LANE` はレーンを `lane_id` / `next_lane` で扱い、道路名を `"road_" + itoa(lane_id)` で構築する。一方ランタイム側は `src_id` / `lane_name` / `next_lanes` を参照する。両者を橋渡しするリネーム処理が必要だが、現状の資料には含まれていない。

### 8.6 コネクタの点密度

`CONNECT_LANE` が生成するコネクタは 2〜3 点のポリラインで、中間点の `dir` は入口と出口の平均方向にすぎない。車両は `nearpoint` で最寄り点の `dir` を引くため、交差点内での進行方向が粗く飛ぶ。Resample と、`curv` の付与が別途必要になる。

### 8.7 EXTRACT_CRUSH_POINT の位置づけ

交差点の交差地点に置いた点から、こちらへ向かってくる車を検出するノード。`i@coming` / `i@coming_pt` / `f@coming_dist` を書き出す。ただし参照しているのが `lane_id` であること、出力を読む側のノードが存在しないことから、現時点ではメインの処理系に接続されていない。
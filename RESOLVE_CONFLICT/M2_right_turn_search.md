# M2: 右折車 → 直進車の検索・判断アルゴリズム

対象は **右折車（`turn=="right"`）**。自分（曲線）が持つ`cross_lines`が指す lane_id を持つ直進車を検索し、
譲る(yield)か進む(go)かを判断する。[[intersection-mockup]]で詰めたロジックを、実際のアトリビュート
（`cross_lines`の lane_id、[[M1_ahead_flag]]の`passed`）に載せ替える形で移植する。

## 手順

### 1. 候補の絞り込み（決定）

次の **3条件すべて** を満たす車を「判断対象の直進車」とする。コーン（内積による事前フィルタ）は
**使わない**（下記「決定事項」参照）。

1. まだ交錯点を通過していない（`passed==0`）
2. 自分の`cross_lines`に含まれる lane_id を持っている
3. 検索半径内にいる

### 2. 判断（車ごとに初見の瞬間だけ評価・以後は保持）
[[right-turn-changelog]] #18・#19 で踏んだ地雷（判断のチラつき）を繰り返さないための核心部分：

- 各直進車に `seen`（評価済みか）と `blocking`（停止対象か）を持たせる
- その車が初めて検索条件を満たした瞬間だけ、到達時間 `t_other = 交錯点までの距離 / max(相手の速度, v_floor)` を計算し、`blocking = (t_other < decideTTC)` を確定する
- **一度確定したら、その車については交錯点を通過する（＝`passed`が立つ）まで再評価しない**（速度一定の相手のTTCは時間経過だけで単調に減るため、毎フレーム再評価すると「進んでは止まる」を繰り返す）
- 全体の判断ラッチ：
  ```
  anyCandidate = 未通過・検索半径内の直進車が1台でもいるか
  hasSeen      = 上記のうち seen==true が1台でもいるか
  blocked      = 上記のうち blocking==true が1台でもいるか

  if (!anyCandidate)            decided = none
  else if (decided==none)       decided = hasSeen ? (blocked ? yield : go) : none
  else if (decided==go && blocked) decided = yield   // 本当に新しく見えた車がblockingなら即昇格
  // yieldはここでは自動でgoに戻さない
  ```

### 3. 反応
- **交錯点で居座らない**のが最優先。手前で確実に止まれる距離までは「止まる/減速する」、それを過ぎて止まり切れなければ止まらず抜け切る（`commit_dist`／停止線ルール）
- `stop_dist`未満：緊急減速で停止
- `decel_dist`未満：快適減速
- それ以上：徐行（`turnCreep`）で接近
- `go`と決めたら絶対に途中で止まらない

## 決定事項

- **検索コーンは使わない。** `_ARCHIVE/CONFLICT/SPEC_RIGHT_TURN_CONFLICT_CONE.md`で指摘されていた
  「幾何形状ごとにチューニングが要る」問題を避け、`passed==0` ＋ `cross_lines`のlane_id一致 ＋
  検索半径内、の3条件だけで候補を絞る。
- **道路幅は3.5m。** 検索半径はこれを基準に算出する（倍率は要チューニング。叩き台として
  `search_radius = 道路幅 × N` の形にしておき、Nをパラメータ化する）。

## 転用元との対応

| RESOLVE_CRUSHでの名前 | ここでの対応 |
|---|---|
| `cp`（固定の交点座標） | `cross_lines`が指す直進レーンとの交錯点座標（`cross_pos`、M1と共通） |
| `other.ahead_intersect` | `passed`（M1で作る） |
| `decided`（none/yield/go） | 同じ概念をそのまま使う |
| `anyCandidate` | 未通過・検索半径内のcross_lines一致車の有無 |
| `seenByEgo`/`blockingEgo` | 直進車ごとの評価済みフラグ／blocking確定値 |

## パラメータ（[[right-turn-spec]]の値を初期値の叩き台にする）

`decideTTC`, `stop_dist`, `decel_dist`, `stopMargin`, `v_floor`, `a_comfort`, `a_emergency`, `a_accel`,
`cruise`, `turnCreep` — 実際の交差点スケール・車速レンジに合わせて再チューニングが要る
（RESOLVE_CRUSHの値はモック用の仮の数値）。`search_radius`は上記の通り道路幅(3.5m)基準に変更。

## 決定事項（追記）

- **右折車自身の「交錯点を通過したか」の専用判定は作らない。** `s@turn`は右折レーンのコネクタ区間を
  抜けると自動的に`"right"`から`""`（直進扱い）に戻る（[03_UPDATE_LANE_ATTRIB.vfl](03_UPDATE_LANE_ATTRIB.vfl)、
  コネクタの点だけが`turn`を持ち直進レーン本体は既定値のため）ので、それでおおむね検索・判断・反応が
  止まる。仮にこのタイミングが実際の交錯点通過と多少ズレて「通過直後にまだ反応が残る」瞬間があっても、
  問題にならない：[[06_CAR DETECT]]の`is_cross`が内積＋横オフセットで毎フレーム独立にリアルタイム
  判定しているので、そちらが常にバックストップとして効く。だから交錯点通過を厳密に管理する必要が
  そもそもない。

## 未決定・要確認

- `v_floor`・`decideTTC`・`search_radius`の倍率など、実際の道路スケールに合わせた再チューニングが必要（モックの数値は仮、チューニングはこの後）

## 実装（済）

[[07_RESOLVE_CONFLICT]]に実装。設計との対応・簡略化した点は以下の通り。

- **前提として追加した属性**：
  - `i@lane_id`（車、全車）：[[03_UPDATE_LANE_ATTRIB]]で追加（`cross_lines`とのlane_id照合に必要。
    従来は未取得だった）。当初は`src_id`と一緒に毎フレーム再取得していたが、2026-09-10に
    レーン追跡の主キー自体を`src_id`から`lane_id`へ置き換え、`lane_changed`時に1回だけ確定して
    以後は書き換えない形に変更（`src_id`は左右分割・複数車線化で複数の最終レーンが同じ値を
    共有しうり、`nearpoint`/`findattribval`のキーとして不正確だったため）。
  - `i[]@cross_lines`（車、右折車のみ）：右折レーンのprim属性`cross_lines`を`lane_changed`時に
    キャッシュ（[[M1_ahead_flag]]の`cross_pos`キャッシュと同じ場所・同じタイミング）。
- **「相手ごとのseen/blocking」は"今ロック中の相手車1台"を、`cross_lines`のindexごとに独立に持つ形にした。**
  モック([[right-turn-changelog]])の設計は直進車側に`seen`/`blocking`を持たせ複数候補を同時に
  評価できる形だったが、ここでは右折車側に`rt_target_id`（ロック中の相手の`@id`）と
  `rt_target_lock`（その相手について確定したblocking判定）を持たせている。当初は1組だけの
  スカラーで近似していたが、**M5-1の実装により`i[]@rt_target_id`/`i[]@rt_target_lock`/
  `v[]@rt_target_cp`/`i[]@rt_decided`を`cross_lines`と同じ長さ・同じindexの配列にし、
  相手レーンごとに完全に独立してロックを持つ形に一般化した**（詳細 →
  [[M5_multi_lane_and_queue]]）。ロック中の相手が候補から消えたら（通過 or 検索半径外）
  そのindexだけ解除して次点を拾い直す。`i@yield_conflict`/`f@dist_to_conflict`は、
  全indexのうちyield中のものだけを見て「最も近い交錯点」に集約したスカラー値。
- **`cp`（交錯点座標）は相手の直進車が持つ`v@cross_pos`をそのまま読む。**
  右折車が別途`cross_pos`を検索し直す必要はない（M1で直進車ごとにキャッシュ済みの値を
  `point(0, "cross_pos", pt)`で読むだけ）。ロック時にそのindexの`v[]@rt_target_cp`へコピーして
  以後は検索なしで使い回す。
  → **【M6bで変更済み】** この「相手の直進車個体からcp を読む」方式は、対応する直進車が
  いない/まだ`lane_changed`を済ませていない瞬間に値が不安定になる問題があったため廃止。
  現在は`cross_lines`確定と同時にRoad Line側の`cross_pos`(prim)をlane_idキーで直接引き、
  相手車の有無に関係ない固定値として`rt_target_cp`を確定している。詳細 → [[M6_t_ego]]。
- **コーン・前方判定は本当に一切使わない。** `passed==0` ＋ `cross_lines`一致 ＋ 検索半径内、の
  3条件を満たした瞬間にそのまま「見えた」扱いにしてTTCを評価する（設計通り）。
- パラメータは`decideTTC=3.5s`, `v_floor=1.5m/s`, `search_radius=道路幅3.5m×6`,
  `stopMargin=1.5m`を仮置き。M4は実装済みだが、[[right-turn-spec]]の`stop_dist`/`decel_dist`
  による3段階の減速プロファイルは採用せず、既存の`required_decel`式（信号・歩行者と共通）に
  そのまま乗せる形にした（詳細 → [[M4_state_integration]]）。

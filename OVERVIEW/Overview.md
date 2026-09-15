# OVERVIEW

## 1. ドキュメント一覧

**コアパイプライン**(番号順。`.vfl`と対になっている`.md`は実装のコピー)

| ファイル | 内容 |
|---|---|
| [[00_STRUCTURE]] | 全体の一言サマリ：信号機・歩行者を検知し、そのアトリビュートを後続ノードで処理して速度などを制御 |
| [[01_INIT]] | 車両生成時（初回のみ）に全attributeの初期値を定義する唯一の場所 |
| [[02_CHANGE ROAD]] | 今いるスプラインの末端に来たら、次のレーンに切り替える（`lane_name`更新、`lane_changed`フラグを立てる） |
| [[03_UPDATE_LANE_ATTRIB]] | `lane_changed`直後だけ`lane_id`を再解決し`next_lane_name`を1回だけ決める。毎フレーム`dir`/`curv`/`turn`をレーンネットワークから取得（2026-09-10: 主キーを`src_id`から`lane_id`に変更）。`cross_pos`/`cross_lines`もRoad Line(Input 1)側から取得し、Input 3(歩行者ジオメトリ)は使わない（M6c） |
| [[04_SIGNAL DETECT]] | 同じレーンの信号機を検索し、状態・距離を取得 |
| [[05_HUMAN DETECT]] | 自分のレーンを横断中の歩行者を検知 |
| [[06_CAR DETECT]] | 同レーン先行車・交差/右左折車の中から最も近い1台を検出 |
| [[07_RESOLVE_CONFLICT]] | 右折 vs 直進コンフリクト解決。M1(`passed`判定)・M2(右折車のTTC判断)・M4(STATE統合)・M5-1(複数交錯レーン対応)実装済み。詳細は[[00_OVERVIEW]] |
| [[08_STATE]] | 信号・歩行者・前車・コンフリクト(M4)の情報から行動（進行/ブレーキ/停止）を判断 |
| [[09_ACCEL]] | STATEの状態に応じてIDM等で加速度を計算 |
| [[010_INTEGRATE]] | 速度・位置を積分して更新 |
| [[011_COLORIZE]] | `car_state`等からビューポート表示色(`Cd`)を決定 |
| [[11_CONNECT_LANE]] | 道路ネットワーク生成側：交差点の入口/出口点から接続曲線(コネクタ)を作り、`next_lanes`・`cross_lines`等の交差点まわりの属性を配る |
| [[12_BAKE_CROSS_POS]] | 道路ネットワーク生成側：直進レーンの交錯点座標を`cross_pos`としてRoad Line側（point/prim両方）に焼き込む。従来は車が`lane_changed`時に交錯点ジオメトリへ`nearpoint`していたが、右折車がlane_idキーで直接引けるようにするためのベイク（M6b） |

**RESOLVE_CONFLICT（右折 vs 直進コンフリクト解決の設計ドキュメント）**

| ファイル | 内容 |
|---|---|
| [[00_OVERVIEW]] | 設計全体・アトリビュート一覧・マイルストーン表 |
| [[M1_ahead_flag]] | 直進車の`passed`判定（実装済み） |
| [[M2_right_turn_search]] | 右折車→直進車の検索・TTC判断（実装済み） |
| [[M3_straight_car_guard]] | 直進車→右折車の専用検知（検討の結果、不要と判断） |
| [[M4_state_integration]] | `08_STATE`への統合（未実装） |
| [[M5_multi_lane_and_queue]] | 複数交錯点・複数右折車の待ち行列（保留） |

**REFLECTIONS（日付入りの検討記録。当時の状況のまま残す）**

| ファイル | 内容 |
|---|---|
| [[2026-09-07_nearpointとnearpointsの使い分け]] | `nearpoint`と`nearpoints`の使い分け基準の検討記録 |
| [[2026-09-07_右折コンフリクト判断ラッチの修正]] | RESOLVE_CRUSHモックのラッチ関連バグ修正の記録 |

**その他**

| ファイル | 内容 |
|---|---|
| [[___SPEC]] | 旧アーキテクチャ時点の詳細仕様書（ノード名が現行と一部異なる。属性名は今回のリネームに追従済み） |
| `_ARCHIVE/` | 書き直しにより退役した旧ノード・旧設計一式。属性名は当時のまま（リネーム対象外） |

---

## 2. パイプライン概要

01_INIT → 02_CHANGE ROAD → 03_UPDATE_LANE_ATTRIB → 04_SIGNAL DETECT → 05_HUMAN DETECT → 06_CAR DETECT → 07_RESOLVE_CONFLICT → 08_STATE → 09_ACCEL → 010_INTEGRATE → 011_COLORIZE

（`11_CONNECT_LANE` / `12_BAKE_CROSS_POS` は上記と別系統：車両シミュレーション本体ではなく、道路ネットワークジオメトリを事前生成するSOP側の処理。`12_BAKE_CROSS_POS`は`11_CONNECT_LANE`の後段、車Solverより前に1回だけ通す）

---

## 3. アトリビュート一覧

### 3.1 車（Input 1st = start point。Solver内では自分自身 = Input 0として参照される）

| 属性 | 型 | 役割 | 主な設定/参照ノード |
|---|---|---|---|
| `initialized` | int | 初期化済みフラグ | 01_INIT |
| `car` | int | 車であることの目印（1固定） | 01_INIT |
| `id` | int | 車ごとの一意なID（Houdini標準）。乱数シードや[[07_RESOLVE_CONFLICT]]の相手車追跡キーに使用 | 全体 |
| `lane_name` | string | 現在走行中のレーン名（旧`road_name`）。spawn時にレーンから継承、末端で更新 | 02, 03, 04, 05, 06 |
| ~~`src_id`~~ | int | **廃止（2026-09-10）**。旧: 現在のレーンの元スプラインID（旧`road_id`）で位置追跡の起点だったが、左右分割・複数車線化で複数の最終レーンが同じ値を共有しうる（対向車線・並走車線）ため、`nearpoint`/`findattribval`のキーとしては不正確だった。`lane_id`に置き換え済み。旧版は`BACKUP/03_UPDATE_LANE_ATTRIB.vfl`参照 | （廃止） |
| `lane_changed` | int | レーンが切り替わった直後を示すフラグ（旧`road_changed`） | 02, 03 |
| `next_lane_name` | string | 次に進むレーン名（旧`next_road`） | 02, 03 |
| `dir` | vector | 進行方向（正規化済み） | 03（毎フレーム更新）, 06, 07, 010 |
| `curv` | float | 現在地点の曲率（0-1正規化） | 03（毎フレーム更新）, 010 |
| `turn` | string | `"left"`/`"right"`/`"straight"`/`""`（既定＝直進扱い） | 03（毎フレーム更新）, 07, 010 |
| `lane_id` | int | 現在のレーン（分割・コネクタ生成後の最終スプライン）を一意に識別するID。位置追跡の主キー兼`cross_lines`との照合キー。`lane_changed`時に1回だけ確定し、以後そのレーンにいる間は書き換えない（`src_id`廃止に伴い2026-09-10変更） | 03（`lane_changed`時に確定）, 07 |
| `u` | float | 現在レーンspline上のパラメトリック位置 | 02 |
| `car_state` | int | 0=CRUISING / 1=BRAKING / 2=STOPPED | 01, 08, 09, 010, 011 |
| `yellow_commit` | int | 黄信号を止まらず通過するとコミット済みか（旧`go`） | 01, 08 |
| `yellow_judged` | int | 今回の黄信号での可否判定を済ませたか。**01_INITでの初期化なし**（暗黙の0初期値に依存） | 08 |
| `ctrl_src` | int | 現在`dist_to_target`を決めている対象（0=信号, 2=歩行者, 3=右折コンフリクト） | 08 |
| `dist_to_target` | float | 止まるべき対象までの距離（信号/歩行者/コンフリクトのうち近い方） | 08, 09 |
| `effective_dist` | float | 停止線オフセットを引いた実効停止距離 | 08, 09 |
| `required_decel` | float | 停止に必要な減速度 | 08, 09 |
| `signal_ptnum` / `signal_state` / `dist_to_signal` / `yellow_remain` | int/int/float/float | 前方信号機の点番号・状態・距離・黄残り時間 | 04（毎フレーム上書き）, 08 |
| `ped_ptnum` / `ped_crossing` / `dist_to_ped` | int/int/float | 前方横断中歩行者の点番号・横断中フラグ・距離 | 05（毎フレーム上書き）, 08 |
| `car_ptnum` / `dist_to_car` / `car_vel` / `car_signal` / `car_is_cross` | int/float/float/int/int | 前方最近傍車（同レーン or 交差）の点番号・距離・速度・signal反応中フラグ・交差車フラグ | 06（毎フレーム上書き）, 08, 09 |
| `passed` / `passed_frame_count` | int/int | 交錯点通過フラグ（6フレームバッファ確定）とそのカウンタ。直進車のみ意味を持つ | 01, 03（`lane_changed`時リセット), 07（毎フレーム判定） |
| `cross_pos` | vector | 自分の交錯点の世界座標（直進車）。`lane_changed`時に1回だけ取得しキャッシュ | 01, 03, 07 |
| `cross_lines` | int[] | 右折レーンが交錯する直進レーンの`lane_id`一覧。`lane_changed`時にレーンprimからキャッシュ | 03, 07 |
| `rt_decided` | int[] | 右折車の判断ラッチ。`cross_lines`と同じindexで1本ずつ独立：0=none/1=yield/2=go（M5-1） | 01, 03（`lane_changed`時に`cross_lines`と同じ長さで作り直す）, 07 |
| `rt_target_id` | int[] | 各indexごと、判断を保持中の相手車の`id`（いなければ-1） | 01, 03, 07 |
| `rt_target_lock` | int[] | 各indexごと、`rt_target_id`捕捉時に確定したblocking判定（以後保持） | 01, 03, 07 |
| `rt_target_cp` | vector[] | 各indexごと、この交錯レーン固有の交錯点座標。`cross_lines`確定と同時にRoad Line側の`cross_pos`(prim)から1回だけ取得する固定値で、追跡中の相手車個体には依存しない（M6b） | 01, 03（`lane_changed`時に確定）, 07（読むだけ） |
| `yield_conflict` / `dist_to_conflict` | int/float | 右折車がコンフリクトのため譲るべきか・対象交錯点までの距離。`rt_decided[]`のうちyield中のものを集約したスカラー値（最も近いものを採用）。08_STATEが読む（M4実装済み、ctrl_src=3） | 01, 07, 08 |
| `col` | int | `accel<0`で1、`vel<0.05`で2。**ヴォルト内のどこからも読まれていない**（用途不明、デバッグ/ビューポート着色用の可能性） | 09 |
| `max_speed` / `max_accel` / `max_decel` | float | 車両ごとにランダム化された物理パラメータ | 01, 09, 010 |
| `vel` / `accel` | float | 現在速度・加速度 | 01, 06, 09, 010, 011 |
| `dt` | float | `@TimeInc`のコピー | 010 |
| `search_radius` | float | 検出半径。04で速度連動計算され、06でも流用される（ノードをまたいで使い回す前提） | 04, 06 |
| `P`, `v` | vector | 位置・速度ベクトル（Houdini標準） | 010 |
| `Cd` | vector | ビューポート表示色 | 011 |

**注記**：`lane_name` / `lane_id` / `lane_changed` / `dir` / `curv` / `turn` は`01_INIT`では初期化されていない。スポーン時に道路ネットワーク側からコピーされて最初の値を持つ想定と思われるが、`___SPEC.md`（旧アーキテクチャ、当時の主キーは`src_id`）では`src_id=-1`・`lane_changed=1`をINITに置く設計だったため、現行実装と食い違いがある。スポーン時の初期値がどこから来るのか要確認（`src_id`は2026-09-10に`lane_id`へ置き換えたが、この初期化ギャップ自体は未解決のまま持ち越し）。

### 3.2 レーンネットワーク（Input 2nd = Road Line。Solver内ではInput 1として参照される）

| 属性 | 型 | 役割 |
|---|---|---|
| `P` | vector | 位置（暗黙） |
| `src_id`(point) | int | 元スプラインのID。位置追跡（`nearpoint`/`findattribval`）のキー |
| `lane_name` | string | レーン名（`xyzdist`/`nearpoint`のグループパターンに使う） |
| `dir` | vector | 進行方向 |
| `curv` | float | 曲率（0-1） |
| `turn`(point) | string | `"left"`/`"right"`/`"straight"`。コネクタ区間の点だけが値を持ち、それ以外は既定値 |
| `lane_id`(point) | int | レーンの一意なID |
| `lane_index`(point) | int | 内側から数えたレーンの通し番号（旧`num`） |
| `next_lanes` | string[] | 次に進めるレーン名の候補一覧。入口点にのみ乗る |
| `intersection_id` | int | 属する交差点のID（旧`node_id`） |
| `is_cut_end` | int | 交差点で切られた端点かどうか |
| `is_entry` | int | 1=入口点／0=出口点（旧`flow`） |
| `prev_lane` | int | 手前のレーンID |
| `src_id`(prim) | int | **`in_road`から複製された、point版とは別由来のprim属性。どこからも読まれていない**（旧`road`。前回のリネームで名前だけ揃えたが値の出所は別） |
| `lane_id`(prim) / `lane_index`(prim) / `turn`(prim) | int/int/string | point版と同じ値をprim単位でも複製 |
| `next_lane` | string | このコネクタ1本の行き先レーン名（単一値） |
| `is_connector` / `is_curved` | int | 接続線か／曲がっているか |
| `cross_lines`(prim) | int[] | 右折レーン（曲線）が交錯する直進レーンの`lane_id`一覧。**既存**（本ヴォルトのコードでは作られていない、Houdini側で焼き込み済み） |
| `cross_curve`(prim) | int | 直進レーンが交錯する右折レーンの`lane_id`。**既存**（同上） |
| `cross_pos`(point/prim) | vector | 直進レーンの交錯点座標。[[12_BAKE_CROSS_POS]]が本ヴォルトのコードで焼き込む（M6b。上記2つと違い、このリポジトリで生成する） |
| グループ | - | `connectors` / `curved` / `straight` / `corner` |

### 3.3 信号機（Input 3rd = Traffic Signal。Solver内ではInput 2として参照される）

| 属性 | 型 | 役割 |
|---|---|---|
| `P` | vector | 位置 |
| `lane_name` | string | この信号が制御するレーン名 |
| `signal_state` | int | 信号状態（0=青相当 / 1=黄相当 / 2=赤相当、`08_STATE`の分岐から逆算） |
| `yellow_remain` | float | 黄信号の残り時間 |

### 3.4 歩行者・交錯点（Input 4th = Pedestrian, cross。Solver内ではInput 3として参照される）

| 属性 | 型 | 役割 |
|---|---|---|
| `P` | vector | 位置 |
| `lane_name` | string | この歩行者が横断する/この交錯点が属するレーン名 |
| `ped_crossing` | int | 歩行者が横断中かどうか |
| グループ`cross` | - | 車の交錯点（M1/M2で使う`cross_pos`の実体）。歩行者の点群と同じジオメトリに同居する別グループ |

**注記**：`ped_crossing`（歩行者が横断中）と`cross`グループ（車同士の交錯点）は同じ"cross"語根で別概念。同じInput 4thのジオメトリに同居しているため紛らわしい。将来的に分ける場合は要相談。

---

## 4. Inputごとのアトリビュート早見表

| Input | 中身 | Solver内での参照番号 | 持つべき属性 |
|---|---|---|---|
| 1st | Start Point（車の初期状態） | 0（自分自身） | 3.1節の全属性。特にスポーン時点で`lane_name`/`src_id`/`lane_changed`/`dir`/`curv`/`turn`を持っている必要がある（詳細は3.1の注記） |
| 2nd | Road Line（レーンネットワーク） | 1 | 3.2節の全属性 |
| 3rd | Traffic Signal（信号機） | 2 | 3.3節の全属性 |
| 4th | Pedestrian, cross（歩行者＋交錯点） | 3 | 3.4節の全属性 |

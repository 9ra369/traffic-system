# RESOLVE_CONFLICT — 右折 vs 直進 コンフリクト解決（実装計画）

旧 `07_CONFLICT_DETECT` と `CONFLICT/`フォルダ一式は書き直しのため `_ARCHIVE/` に移動した。
代わりのノードは [[07_RESOLVE_CONFLICT]]（M1・M2・M4・M5-1実装済み）。ここに実装をマイルストーンに分けて計画する。

設計の元ネタ（考え方は転用するが、データ構造・座標系はゼロから作る）：
- [[intersection-mockup]] / [[right-turn-spec]] / [[right-turn-changelog]]（RESOLVE_CRUSHのHTMLモックで詰めたロジックと、そこで踏んだ地雷の記録）
- `_ARCHIVE/CONFLICT/SPEC_RIGHT_TURN_CONFLICT_CONE.md`（検索コーン＋到達時間方式の初期設計。lane_id/座標の持ち方の議論はここが詳しい）
- `_ARCHIVE/CONFLICT/05_CONFLICT_DETECT.vfl`（gap acceptance・commit_dist・wait_timeなど、実装に落としたときの工夫）

## アトリビュート

**既存**
| 属性 | 種類 | 意味 |
|---|---|---|
| `cross_lines` | prim（右折レーン＝曲線） | このレーンと交錯する直進レーンの lane_id |
| `cross_curve` | prim（直進レーン） | このレーンと交錯する右折レーンの lane_id |

**これから必要**
| 属性 | 種類 | 意味 |
|---|---|---|
| `passed`（旧`ahead`） | point（車、直進車のみ） | 交錯点を通過したか（0/1）。符号反転から6フレームのバッファ後に確定 |
| `passed_frame_count`（旧`pass_frame_count`） | point（車、直進車のみ） | `passed`確定までのフレームカウンタ |
| `yield_conflict` | point（車、右折車） | 右折先で直進車に譲るべきか |
| `dist_to_conflict` | point（車、右折車） | 譲る対象の交錯点までの距離 |
| `lane_id` | point（車、全車） | 現在のレーンID。`cross_lines`とのlane_id照合用に03_UPDATE_LANE_ATTRIBで取得。2026-09-10からレーン追跡そのものの主キー（旧`src_id`）も兼ねる（詳細 → 下記「決定事項（追記2）」） |
| `cross_lines` | point（車、右折車のみ） | 右折レーンの同名prim属性をlane_changed時にキャッシュした配列 |
| `rt_decided` / `rt_target_id` / `rt_target_lock` / `rt_target_cp` | point（車、右折車のみ。配列。`cross_lines`と同じ長さ・同じindex） | M2の内部状態。相手レーンごとに独立してロックを持つ（M5-1、詳細 → [[M2_right_turn_search]] / [[M5_multi_lane_and_queue]]） |

**決定事項**
- `cross_pos`（交錯点の世界座標）は初回（`lane_changed`時）に1回だけ取得してキャッシュする。
  取得方法は「その場で検索」を採用：Input 3の`cross`グループ点をlane_name一致で`nearpoint`する
  （詳細 → [[M1_ahead_flag]]）。
- 道路幅は **3.5m**。検索半径はこれを基準に算出する（倍率は要チューニング、[[M2_right_turn_search]]）。
- 直進車→右折車の専用検知（旧M3）は**不要と判断**。既存の`06_CAR DETECT`の`is_cross`判定で代替できる
  （理由 → [[M3_straight_car_guard]]）。

**決定事項（追記）**
- 交錯点ジオメトリ（`cross`ポイントグループ）は新設不要。**Input 3（歩行者ジオメトリ）に既にある
  `cross`ポイントグループ**をそのまま流用する（歩行者の`crossing`検知と同じ入力・同じ
  lane_name一致の`nearpoint`パターンで引ける）。詳細 → [[M1_ahead_flag]]。

**決定事項（M5-1、追記）**
- 1本の右折レーン（曲線）が複数の直進レーンと交錯する場合（＝`cross_lines`が複数値を持つ場合）は、
  右折車側の判断状態（`rt_decided`/`rt_target_id`/`rt_target_lock`/`rt_target_cp`）を
  `cross_lines`と同じ長さの配列にし、相手レーンごとに独立してロックを持つ形で対応済み
  （交錯点の座標自体は直進車側の`cross_pos`が既に一意なので変更不要だった）。
  詳細 → [[M5_multi_lane_and_queue]]。
  （直進レーン側の`cross_curve`は常に単一値と確定済み。[[M1_ahead_flag]]）

**決定事項（追記2、2026-09-10）**
- **レーン追跡の主キーを`src_id`から`lane_id`に変更した。** `src_id`は分割・交差点コネクタ生成前の
  「元の構成ライン」のID（十字路なら元は2本＝0と1）で、左右分割や複数車線化を経ると複数の
  最終レーン（対向車線・並走車線など）が同じ`src_id`を共有しうる。これを[[03_UPDATE_LANE_ATTRIB]]の
  毎フレーム`nearpoint`／`next_lanes`解決の`findattribval`のキーに使うと、別レーンの点を誤って
  拾う恐れがあった。`lane_id`は分割・コネクタ生成後の最終スプライン1本ずつに一意なIDなので、
  これに統一した。`i@lane_id`は`lane_changed`時に1回だけ確定し（旧`i@src_id`と同じ役割・
  同じタイミング）、以後そのレーンにいる間は書き換えない（従来M2用に毎フレーム再取得していたが
  それも廃止）。変更前の`.vfl`一式は`BACKUP/`フォルダに保存済み。

## マイルストーン

| # | 内容 | ファイル |
|---|---|---|
| M1 | 直進車の `passed` 判定（通過検知＋6フレームバッファ） — **実装済み** | [[M1_ahead_flag]] |
| M2 | 右折車→直進車：検索・TTC判断アルゴリズム — **実装済み**（複数交錯レーンの同時捌きはM5-1で対応済み） | [[M2_right_turn_search]] |
| M3 | 直進車→右折車：保険的減速の検知アルゴリズム — **検討の結果、不要と判断** | [[M3_straight_car_guard]] |
| M4 | `08_STATE` への統合 — **実装済み** | [[M4_state_integration]] |
| M5 | 複数交錯点・複数右折車の待ち行列 — **1は実装済み。2, 3は方針決定済みで実装課題なし。4はスコープ外** | [[M5_multi_lane_and_queue]] |

M1→M2→M4 の順で依存している（M2はM1の`passed`を前提に検索から除外を行うため）。M3は不採用。

## 全体を通しての設計方針（RESOLVE_CRUSHからの転用）

- **優先順位は直進＞右折。右折側の判定だけ厳密にする**（直進側は保険的な減速のみ）。
- 判断（yield/go）は **相手車ごとに、初めて検知した瞬間のTTCだけで1回確定し、以後その車については再評価しない**。速度一定の相手のTTCは時間経過だけで単調に減っていくので、毎フレーム再評価すると「進んでは止まる」を繰り返す（[[right-turn-changelog]] #19 参照）。
- 「判断を保持する条件」と「判断の中身を決める条件」は分ける。保持は緩く（対象車が本当にいなくなるまで）、確定の中身はTTC等で絞る。
- 安全側（yield）への切り替えはいつでも許可、緩める側（yield→go）は限定的な条件（交錯点を通過した／停止線を越えて止まり切れなかった）でしか許可しない、という非対称なラッチにする。
- **交錯点で居座らない**のが最優先。手前で確実に停止できるなら停止、間に合わなければ止まらず抜け切る（`commit_dist`）。これにより直進車側は「動いている右折車」にしか出会わない（＝停止して譲っている右折車に反応して膠着する、という事態が起きない）。だからこそM3のような直進車専用の右折車検知を新設しなくても、既存の`06_CAR DETECT`の`is_cross`判定だけで安全に成立する。

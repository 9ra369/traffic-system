# M6: t_ego導入 — 固定TTC閾値からの脱却

## 課題

M2のblocking判定は相手の到達時間`t_other`を固定値`decide_ttc`(3.9s)とだけ比較しており、
自車の状態（停止中か、すでに交錯点近くまで進入中か）を見ていなかった。結果、進入済みの
車にも一律ブレーキを要求し、[[08_STATE]]側で物理的に不可能な`required_decel`を要求する
不整合が起こり得た。

## 変更

判定基準を固定値比較から、自車の到達/通過時間との相対比較に変更した。

```
blocking = (t_other < t_ego + safety_margin)
```

- `t_other`：相手が交錯点に着くまでの時間（既存の計算をそのまま使用）
- `t_ego`：自車が交錯点に着くまでの時間
  - 走行中（`car_state==0` または `vel >= vel_stopped`）：現在の`P`/`vel`/`accel`から
    `t_other`と同じ等加速度式で算出
  - 停止中（`car_state>=1` かつ `vel < vel_stopped`）：`max_accel`による0初速からの
    到達時間 `sqrt(2*e_dist / max_accel)` を別途算出
- 交錯点の座標は追加で持たず、既存の`t_cp`（＝相手の`cross_pos`。両者に共通の同じ
  世界座標点）をそのまま自車からの距離計算にも使う。

### 実装

[[07_RESOLVE_CONFLICT]]のM2ブロック、`t_other`確定直後・`t_lock`確定前に追加。
`decide_ttc`パラメータは廃止し、`safety_margin`（加算マージン、秒）に置き換えた。
`vel_stopped`（停止とみなす速度閾値、m/s）を新規パラメータとして追加した。

## 効果

交錯点付近まで進入済みの車には無理な急減速を要求しなくなり、停止線で待機中の車には
従来通り慎重な判定を保てる。go→yield昇格時の物理的な矛盾（止まれないのに止まれと言う）
も、判定段階で未然に防げる。

## 積み残し

- 停止中`t_ego`の算出方法：厳密な等加速度式（採用中） vs `@id`ごとにINITで標準到達時間を
  事前計算する簡略版、要検討。
- `safety_margin`の妥当値は要チューニング（暫定 1.0s）。
- `vel_stopped`の妥当値も要チューニング（暫定 0.1 m/s）。

## M6b: rt_target_cpの取得元をRoad Line側に変更（実装済み）

### 課題

従来`rt_target_cp`（＝`t_cp`、交錯点座標）は、右折車が追跡中の相手（直進車個体）の
`cross_pos`を`point(0, "cross_pos", target_pt)`で毎フレーム読みに行っていた。これは
「今その交錯レーンにたまたま直進車がいて、しかもその車がすでに`lane_changed`を
済ませている」ことに依存する設計で、対応する直進車がまだ`03_UPDATE_LANE_ATTRIB`の
`lane_changed`ブロックを通っていない、あるいは近くに1台もいない瞬間には値が
不定（`{0,0,0}`寄りの初期値）になりうる脆さがあった。

交錯点の座標は本来、右折レーンと直進レーンの幾何学的な交点であり、車の走行状態には
一切依存しない値。「特定の直進車個体を経由する」という設計自体が不要な依存だった。

### 変更

- [[12_BAKE_CROSS_POS]]（新規）：道路網生成時に1回だけ、直進レーンのRoad Lineプリムへ
  `cross_pos`をpoint属性・prim属性の両方で焼き込む。`cross_lines`/`cross_curve`と
  同じ「Houdini側で焼き込み済み」の仲間に揃える形。
- [[03_UPDATE_LANE_ATTRIB]]：右折車の`lane_changed`ブロックで`cross_lines`を確定する
  のと同時に、各`opp_lane`（交錯する直進レーンの`lane_id`）についてRoad Line側の
  `cross_pos`(prim)を`findattribval`で直接引き、`rt_target_cp`をその場で確定する
  （＝`cross_lines`と同じタイミングで1回だけ）。
- [[07_RESOLVE_CONFLICT]]：`t_cp`を追跡中の相手車から毎フレーム読み直す処理を削除。
  相手が候補から消えた際の`t_cp = {0,0,0}`リセットも削除（このindex固有の固定値なので
  相手の有無に関係なく保持し続けてよい）。

### 効果

右折車の交錯点判断が、対応する直進車個体の状態（存在の有無・`lane_changed`の
タイミング）から完全に独立した。直進車が1台もいない交差点でも、右折車は正しい
交錯点座標で`t_ego`/`stop_margin`判定ができる。

## M6c: 直進車自身のcross_pos取得元もRoad Line側に統一（実装済み）

### 変更

直進車が自分自身のために持つ`v@cross_pos`（M1の`passed`判定に使う）も、従来は
Input 3(歩行者ジオメトリ)の"cross"グループ点へ`nearpoint`していたが、M6bで
`12_BAKE_CROSS_POS.vfl`がRoad Line側に`cross_pos`(point/prim)を焼き込むようになった
のを受けて、[[03_UPDATE_LANE_ATTRIB]]側もそちらを直接読む形に統一した
（`prim(1, "cross_pos", pr[0])`。`pr[]`はcross_linesキャッシュと共有）。

これにより`03_UPDATE_LANE_ATTRIB.vfl`はInput 3(歩行者ジオメトリ)を一切使わなくなった
（`05_HUMAN DETECT`など他ノードのInput 3利用には影響しない。ノードごとに独立した配線）。

### 実装上の注意（ハマった点）

`s@turn`でREAD対象を「直進レーンのときだけ」に絞ろうとしたが、`lane_changed`ブロック
実行時点の`s@turn`は**まだ前のレーンの値のまま**（`s@turn`の更新はファイル末尾の
毎フレームブロックで、`lane_changed`ブロックより後に実行される）。これでガードすると、
右折レーン→直進レーンに乗り換えた瞬間だけ`v@cross_pos`が更新されずに`{0,0,0}`へ
固まってしまう不具合になる。→ ガードは付けず無条件で読む形にした（右折レーンの
primは`cross_pos`が未設定＝既定値`{0,0,0}`が返るだけで実害はない。`v@cross_pos`は
直進車以外どこからも読まれないため）。

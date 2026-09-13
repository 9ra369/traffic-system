# M1: 直進車の `passed` 判定

対象は **直進車（`turn=="straight"`）だけ**。右折車はここでは扱わない。

属性名を `ahead` → **`passed`**、`pass_frame_count` → **`passed_frame_count`** に変更（「まだ前方＝ahead」だと
「通過した」の意味と紛らわしいため）。以降このリポジトリ内では `passed` で統一する。

## やること

1. 自分の交錯点（`cross`ポイントグループ、座標は`cross_pos`）へのベクトルと、自分の進行方向 `v@dir` の
   内積を毎フレーム計算する。
   ```
   to_cross = cross_pos - @P
   d = dot(normalize(v@dir), normalize(to_cross))
   ```
   - `d > 0`：交錯点はまだ前方 → 未通過
   - `d <= 0`：交錯点はもう後方 → 通過した瞬間
2. `d`の符号が反転した（＝通過した）フレームから数えて**6フレーム**経過したら `i@passed = 1` にし、
   同時に `i@passed_frame_count` を `0` に戻す。
   - 符号反転の瞬間に即 `passed=1` にしない理由：交錯点ちょうどの位置で `d` が0付近を細かく行き来する
     可能性があり、その揺れをそのまま`passed`に反映すると通過判定がチラつく。6フレームのバッファで
     ノイズを吸収する。
3. `passed==1` になった直進車は、**右折車側の検索（M2）からは除外**する。
   （直進車側から右折車を検知する専用ロジックは不要と判断した。[[M3_straight_car_guard]] 参照）

## 決定事項

- **`cross_pos`（交錯点の世界座標）の取得方法**：初回（後述の`lane_changed`時）に**1回だけ**取得し、
  以後はキャッシュを使い回す（毎フレーム探し直さない）。取得の仕方自体は
  ①その場で検索して取得（`nearpoint`等）／②レーンネットワーク構築時にあらかじめプリム属性として
  焼き込んでおき、そこから読むだけにする、のどちらでもよい（実装のしやすさで選ぶ）。
- **`passed`のリセット条件**：`lane_changed`が立った瞬間（＝新しいレーンに乗り換えた瞬間）に、
  そのレーンに紐づく交錯点用の状態（`passed`, `passed_frame_count`, `cross_pos`）を作り直す。
  → このリセットは`03_UPDATE_LANE_ATTRIB.vfl`の`if (i@lane_changed==1) {...}`ブロックに
  一緒に書くのが自然（`lane_id`/`next_lane_name`の解決と同じタイミングなので。2026-09-10時点、
  旧`src_id`から置き換え済み）。
- **`passed_frame_count`のリセット**：通過を検知して6フレーム経過し`passed=1`が確定したら、
  `passed_frame_count`は`0`に戻す（次にこのレーンに乗るとき＝`lane_changed`時のために掃除しておく）。

## 実装イメージ（擬似コード）

```c
if (s@turn != "straight") return;   // 直進車のみ

vector to_cross = cross_pos - @P;
float  d = dot(normalize(v@dir), normalize(to_cross));

if (i@passed == 0) {
    if (d <= 0) {
        i@passed_frame_count += 1;
        if (i@passed_frame_count >= 6) {
            i@passed = 1;
            i@passed_frame_count = 0;   // 確定したのでカウントは掃除しておく
        }
    } else {
        i@passed_frame_count = 0;       // まだ前方 → カウントは貯めない
    }
}
```

```c
// 03_UPDATE_LANE_ATTRIB.vfl 側、lane_changed のブロックに追加
if (i@lane_changed == 1) {
    // ...既存の lane_id / next_lane_name 解決...
    i@passed             = 0;
    i@passed_frame_count = 0;
    // cross_pos もここで（新しいレーンの交錯点として）取得し直す
}
```

## 決定事項（追記）

- **直進レーンの`cross_curve`は必ず単一値。** 直進レーン（曲線ではなく直線）が複数の右折レーンと
  同時に交錯することは今のところない、という前提で確定。よって`cross_pos`・`passed`・
  `passed_frame_count`は車1台につき1個の値で足り、配列化は不要。
  （右折レーン側＝`cross_lines`が複数値を持つケースは別。[[M5_multi_lane_and_queue]] 参照）

## 実装（済）

- [[01_INIT]]：`i@passed` / `i@passed_frame_count` / `v@cross_pos` の初期値を追加。
- [[03_UPDATE_LANE_ATTRIB]]：`lane_changed`ブロックに3属性のリセット＋`cross_pos`の再取得を追加。
  取得方法は①（その場で検索）を採用：交錯点ジオメトリは新設せず、**Input 3（歩行者ジオメトリ）に
  既にある`cross`ポイントグループ**を流用する。歩行者の`crossing`検索
  （[[05_HUMAN DETECT]]の`nearpoint(3, "@lane_name=..." + " && @ped_crossing=1", @P)`）と同じ入力・
  同じlane_name一致パターンで、`nearpoint(3, "cross && @lane_name=" + s@lane_name, @P)`により
  1点だけ引いて`P`を`cross_pos`にキャッシュする。lane_changed時の1回だけの検索なので、
  文字列を動的に組み立てても毎フレームコストにはならない（[[04_SIGNAL DETECT]]等と同じ考え方）。
  → 00_OVERVIEWの「交錯点ジオメトリの作り方」は**この`cross`グループ流用で決着**（新規ジオメトリ構築は不要）。
- [[07_RESOLVE_CONFLICT]]：擬似コードどおりに`passed`判定本体を実装。

**このノードの入力**：`03_UPDATE_LANE_ATTRIB`にInput 3（歩行者ジオメトリ）の接続が必要
（従来はInput 1のみ使用）。

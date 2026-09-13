# M4: `08_STATE` への統合

[[M2_right_turn_search]]（右折車側）で決めた `yield_conflict` / `dist_to_conflict` を、実際に車の挙動
（進行・ブレーキ・停止）へ反映させる。旧 `_ARCHIVE/CONFLICT/06_STATE_ADD.vfl` が同じ目的で書かれて
いたので、新しい属性名・ロジックに合わせて作り直す。

（[[M3_straight_car_guard]]は不要と判断したため、直進車側で新たにSTATEに統合するものはない。
右折車が実際に直進レーンへ進入してくる場面は既存の`06_CAR DETECT`の`is_cross`経由で
`dist_to_car`/`car_vel`に載り、既存の前車追従ロジックがそのまま対応してくれるはず。）

## 現状の `08_STATE.vfl` との統合ポイント

現行の[[08_STATE]]は信号(`signal_state`)・歩行者(`ped_crossing`)・前車(`car_ptnum`)は見ているが、
コンフリクト判定はまだ何も参照していない（[[00_OVERVIEW]]で触れた既存のギャップ）。

旧`06_STATE_ADD.vfl`のやり方：
```c
if (i@yield_conflict == 1)
    nearest_stop = min(nearest_stop, f@dist_to_conflict);

if (i@yield_conflict == 1 && f@dist_to_conflict < f@dist_to_target) {
    f@dist_to_target = f@dist_to_conflict;
    ctrl_state = ...;   // 信号・歩行者と同じ枠組みで「止まるべき対象」として扱う
}
```
この形（信号・歩行者・コンフリクトを「止まるべき対象のうち一番近いもの」として同列に比較し、
`f@dist_to_target`を更新する）は踏襲してよさそうだが、`ctrl_src`の値の割り当てなど
実際の`08_STATE.vfl`の分岐と整合させる必要がある。

## やること

1. M2の出力（`yield_conflict`, `dist_to_conflict`）を`08_STATE.vfl`の`nearest_stop`/`f@dist_to_target`の
   比較に組み込む
2. 前車(`car_ptnum`)との優先順位も確認する：前車が近ければIDMに任せる、という既存のロジック
   （[[08_STATE]]冒頭）に、コンフリクト判定がどう絡むか整理する

## 決定事項

- **コンフリクトで停止するときの減速度は、既存の`required_decel`の計算式にそのまま乗せる。**
  専用のブレーキ設定（`a_comfort`/`a_emergency`等）は持たない。信号・歩行者と同じ
  `f@dist_to_target`比較の枠組みに乗せているので、減速度計算も同じ式（`(vel^2)/(2*effective_dist)`）
  を再利用すれば自然に整合する。

## 実装（済）

[[08_STATE]]に実装。旧`06_STATE_ADD.vfl`の設計をほぼそのまま踏襲した。

- 前車(`car_ptnum`)との優先順位比較（`nearest_stop`）に`yield_conflict==1`時の`dist_to_conflict`を追加。
  前車の方が近ければ従来どおりIDMに任せる。
- 信号・歩行者と同列で`f@dist_to_target`比較に追加：
  ```c
  if (i@yield_conflict == 1 && f@dist_to_conflict < f@dist_to_target) {
      f@dist_to_target = f@dist_to_conflict;
      ctrl_state = 2;      // 赤信号・歩行者と同じ「止まって待つ」
      i@ctrl_src = 3;
  }
  ```
- `ctrl_state`は黄色判定（1）ではなく赤信号と同じ即時停止（2）に割り当てた。M2側の設計方針
  （「右折側の判定だけ厳密にする」「一度yieldに決めたら緩めない」）と整合しており、コンフリクトに
  黄色コミットのような「間に合えば通過する」判断は不要なため。
- `ctrl_src=3`は旧`06_STATE_ADD.vfl`と同じ割り当て（0=信号, 2=歩行者, 3=コンフリクト）。
  [[___SPEC]]・[[Overview]]の`ctrl_src`表も更新済み。

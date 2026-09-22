// ════════════════════════════════════════════════════
// KILL_POINTS: 車を消す処理をここに集約する。
// 読む: i@lane_dead_end, @u（02_CHANGE ROADが毎フレーム更新）
// 書く: i@lane_end_count
//
// ★ Solverの最後（012_COLLISIONの後ろ）に置く。点が消えるタイミングを1箇所に
//   まとめておかないと、同じフレームの下流ノードが消えた車を前車として見る。
// ★ POP Wrangleに置く場合は removepoint ではなく i@dead = 1（KNOWLEDGE/Houdini運用メモ）
//
// Input 0: 車両（Solver内の自分自身）
// ════════════════════════════════════════════════════

if (i@car != 1) return;

// 行き止まりレーンの末端に到達 → 即削除
if (i@lane_dead_end == 1 && @u >= 0.999) {
    removepoint(0, @ptnum);
    return;
}

// ── 末端に張り付いたまま動けない車を回収する ──
// ★ next_lane_nameが空かは見ない。乗り換え先を解決できずlane_nameだけ変わった車は
//   next_lane_nameが残ったまま張り付くので、その条件を付けると永久に回収されない
// hitprim<0のフレームでも@uは前フレームの値が残るので、張り付きはここで検出できる
int end_frames = 12;
if (@u >= 0.9999) {          // 末端ちょうど。健全な車は0.99で乗り換わるのでここに留まらない
    i@lane_end_count += 1;
    if (i@lane_end_count >= end_frames) removepoint(0, @ptnum);
} else {
    i@lane_end_count = 0;
}

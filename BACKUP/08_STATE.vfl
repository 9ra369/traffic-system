f@dist_to_target = 9999.0;
int ctrl_state = 0;
i@ctrl_src     = -1;

// 前車がいる場合、信号・歩行者・コンフリクトより遠ければIDMに任せる
if (i@car_ptnum >= 0) {
    float nearest_stop = 9999.0;
    if ((i@signal_state == 1 || i@signal_state == 2))
        nearest_stop = min(nearest_stop, f@dist_to_signal);
    if (i@ped_crossing == 1)
        nearest_stop = min(nearest_stop, f@dist_to_ped);
    if (i@yield_conflict == 1)
        nearest_stop = min(nearest_stop, f@dist_to_conflict);

    if (f@dist_to_car <= nearest_stop) {
        // 前車の方が近い → IDMに任せる
        i@car_state     = 0;
        i@yellow_commit = 0;
        i@yellow_judged = 0;
        return;
    }
    // 信号・歩行者・コンフリクトの方が近い → 以降の判定に進む
}

if ((i@signal_state == 1 || i@signal_state == 2)
        && f@dist_to_signal < f@dist_to_target) {
    f@dist_to_target = f@dist_to_signal;
    ctrl_state = i@signal_state;
    i@ctrl_src = 0;
}
if (i@ped_crossing == 1 && f@dist_to_ped < f@dist_to_target) {
    f@dist_to_target = f@dist_to_ped;
    ctrl_state = 2;
    i@ctrl_src = 2;
}
// M4: 右折コンフリクト（07_RESOLVE_CONFLICTのyield判定）。赤信号・歩行者と同じ
// 「止まって待つ」(ctrl_state=2)として同列に比較する。減速度計算は他と共通の
// required_decel式にそのまま乗せる（専用のブレーキ設定は持たない）。
if (i@yield_conflict == 1 && f@dist_to_conflict < f@dist_to_target) {
    f@dist_to_target = f@dist_to_conflict;
    ctrl_state = 2;
    i@ctrl_src = 3;
}
// ── 黄色通過コミット中 ──────────────────────────────
if (i@yellow_commit == 1 && i@ctrl_src == 0) {
    i@car_state = 0;
    if (i@signal_state == -1) { i@yellow_commit = 0; i@yellow_judged = 0; }
    return;
}
// ── 状態遷移 ────────────────────────────────────────
if (ctrl_state == 2) {
    i@yellow_commit = 0;  i@yellow_judged = 0;
    if (i@car_state == 0) {
        i@car_state = 1;
        float stop_offset = 1;
        f@effective_dist  = f@dist_to_target - stop_offset;
        f@required_decel  = (f@vel * f@vel) / (2.0 * max(f@effective_dist, 0.01));
    }
    if (f@vel <= 0.05) i@car_state = 2;
} else if (ctrl_state == 1) {
    if (i@yellow_judged == 0) {
        i@yellow_judged = 1;
        float stop_offset = 1;
        f@effective_dist  = f@dist_to_signal - stop_offset;
        float possible_dist = f@vel * f@yellow_remain - 6;
        if (possible_dist >= f@dist_to_signal) {
            i@car_state = 0;  i@yellow_commit = 1;
        } else {
            i@car_state = 1;  i@yellow_commit = 0;
            f@required_decel = (f@vel * f@vel) / (2.0 * max(f@effective_dist, 0.01));
        }
    }
} else {
    i@yellow_commit = 0;  i@yellow_judged = 0;
    if (i@car_state == 1 || i@car_state == 2) i@car_state = 0;
}
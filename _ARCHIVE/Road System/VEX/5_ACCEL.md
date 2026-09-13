// ════════════════════════════════════════════════════
// ACCEL: car_state → f@accel を決定
// ════════════════════════════════════════════════════

if (i@car_state == 0) {
    // ── CRUISING ──────────────────────────────────────
    float speed_ratio = clamp(f@vel / f@max_speed, 0.0, 1.0);
    f@accel = f@max_accel * (1.0 - speed_ratio);

} else if (i@car_state == 1) {
    // ── BRAKING ───────────────────────────────────────
    // 停止目標距離を決定
    // 歩行者がいる場合はそちらを優先、いなければ信号
    float target_dist = (i@ped_crossing == 1)
                        ? f@dist_to_ped
                        : f@dist_to_signal;

    // 緊急度を決定
    // 歩行者は急制動、信号は通常制動
    float active_decel = (i@ped_crossing == 1)
                         ? f@decel_emergency
                         : f@max_decel;

    float stopping_dist = (f@vel * f@vel) / (2.0 * active_decel);

    if (target_dist <= f@stop_dist) {
        // 停止目標到達 → 強制停止
        f@accel = -active_decel;

    } else if (target_dist <= stopping_dist * 1.05) {
        // 物理的に止まれないラインに近い → max制動
        f@accel = -active_decel;

    } else {
        // まだ余裕あり → 滑らかな減速
        float required_decel = (f@vel * f@vel) / (2.0 * target_dist);
        f@accel = -clamp(required_decel, 0.0, active_decel);
    }

} else if (i@car_state == 2) {
    // ── STOPPED ───────────────────────────────────────
    f@accel = 0.0;
}
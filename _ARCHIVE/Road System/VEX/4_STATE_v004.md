**次の工程での処理：車の状態から、加速度などを決定

**実装内容：車の状態判断**
・青、黄色、赤、信号機が範囲内にない、の4つの状態で場合分け
・歩行者の横断状況も加味し、実効信号状態を先に解決してから判断する
・車の状態は、進行中、ブレーキ中、停止中の3つに分けられる
・つまり、信号機の状態 + 歩行者状態 + 前のフレームでの車の状態から、次の車の状態を判断している

**歩行者による実効信号の格上げ**
・横断中(ped_crossing == 1) かつ 通過コミット未(go == 0) → 実効信号を赤扱いに格上げ
・通過コミット済み(go == 1) の場合は格上げしない
  why? → すでに交差点に進入している車を急停止させないための安全弁

**青の場合（歩行者横断なし）**
・進行中 → そのまま
・ブレーキ中 or 停止中 → 進行中へ

**黄色の場合**
この処理は、黄色信号を初めて取得したフレームだけで実行する
why? → 信号機までの距離と最大加速度から判断しているため、毎フレーム実行すると、
        距離が縮まっていくため、たまに止まれないはずなのに止まれるって判定されることがある
・進行中 →
    最大加速度で停止できそうなら、ブレーキ中へ
    停止が厳しそうだったら、通過コミット(go = 1)
・ブレーキ中、停止中 → そのまま
※ 横断者ありの場合は赤扱いに格上げされるため、この判定に来ない

**赤の場合（または歩行者横断中）**
・進行中 → ブレーキ中へ
・ブレーキ中、停止中 → そのまま

**信号機が範囲外の場合**
・ブレーキ中 or 停止中 → 進行中へ（信号なし区間と同じ扱い）

**次の工程での処理：車の状態から、加速度などを決定**

// ════════════════════════════════════════════════════
// STATE: 信号 + 前車 + 歩行者 → car_state / i@go
// 読む: i@signal_state, f@dist_to_signal,
//       i@car_signal,   f@dist_to_car,   f@car_vel,
//       i@ped_crossing, f@dist_to_ped,
//       f@vel, f@max_decel
// 書く: f@dist_to_target, i@ctrl_src, i@car_state, i@go, i@yellow_judged
// ctrl_src: -1=なし 0=信号 1=前車 2=歩行者
// ════════════════════════════════════════════════════

// ── 0) 停止対象の選定 ──────────────────────────────────
f@dist_to_target = 9999.0;
int ctrl_state = 0;    // 0=進 1=黄 2=止
i@ctrl_src     = -1;   // -1=なし 0=信号 1=前車 2=歩行者

if ((i@signal_state == 1 || i@signal_state == 2)
        && f@dist_to_signal < f@dist_to_target) {
    f@dist_to_target = f@dist_to_signal;
    ctrl_state = i@signal_state;
    i@ctrl_src = 0;
}
if (i@car_signal == 1 && f@dist_to_car < f@dist_to_target) {
    f@dist_to_target = f@dist_to_car;
    ctrl_state = 2;
    i@ctrl_src = 1;
}
if (i@ped_crossing == 1 && f@dist_to_ped < f@dist_to_target) {
    f@dist_to_target = f@dist_to_ped;
    ctrl_state = 2;
    i@ctrl_src = 2;
}

// ── ① 黄色通過コミット中（信号のみ有効）─────────────────
if (i@go == 1 && i@ctrl_src == 0) {
    i@car_state = 0;
    if (i@signal_state == -1) { i@go = 0; i@yellow_judged = 0; }
    return;
}

// ── ② 状態遷移 ───────────────────────────────────────
if (ctrl_state == 2) {
    i@go = 0;  i@yellow_judged = 0;
    if (i@car_state == 0) {
        i@car_state = 1;
        float stop_offset    = 1;
        f@effective_dist     = f@dist_to_target - stop_offset;
        f@required_decel     = (f@vel * f@vel) / (2.0 * max(f@effective_dist, 0.01));
    }
    if (f@vel <= 0.05) i@car_state = 2;

} else if (ctrl_state == 1) {   // 黄（信号のみここに来る）

    if (i@yellow_judged == 0) {
    i@yellow_judged = 1;
    float stop_offset = 1;
    f@effective_dist  = f@dist_to_signal - stop_offset;
    float possible_dist = f@vel * f@yellow_remain - 6;

    if (possible_dist >= f@dist_to_signal) {
        i@car_state = 0;  i@go = 1;
    } else {
        i@car_state = 1;  i@go = 0;
        f@required_decel = (f@vel * f@vel) / (2.0 * max(f@effective_dist, 0.01));
    }
}
} else {                        // 青／対象なし
    i@go = 0;  i@yellow_judged = 0;
    if (i@car_state == 1 || i@car_state == 2) i@car_state = 0;
}
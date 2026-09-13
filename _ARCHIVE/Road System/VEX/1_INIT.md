**実装内容：システムに必要な初期情報を設定**
・一度、書き込まれたらこの処理は再度は施されない
　→@ initializedで判断している

// ════════════════════════════════════════════════════
// INIT: 車両生成時（初回のみ）実行
// 全 attribute の初期値を定義する唯一の場所
// ════════════════════════════════════════════════════
if (i@initialized == 1) return;
i@initialized = 1;

//To identify geo as car
i@car = 1;

// ── 状態管理 ──────────────────────────────────────────
i@car_state = 0;    // 0=CRUISING  1=BRAKING  2=STOPPED
i@go        = 0;    // 0=通常停止判断  1=黄色通過コミット済み

// ── 信号情報（SIGNAL DETECTが毎フレーム上書き） ────────
i@signal_ptnum   = -1;
i@signal_state   = -1;
f@dist_to_signal = 9999.0;

// ── 歩行者情報（CROSSING DETECTが毎フレーム上書き） ────
i@ped_ptnum    = -1;
i@ped_crossing =  0;
f@dist_to_ped  = 9999.0;

// ── 前車情報（CAR DETECTが毎フレーム上書き） ────
i@car_ptnum   = -1;
f@dist_to_car = 9999.0;
i@car_signal  = -1;

// ── 物理パラメータ（車両ごとにランダム化） ───────────────
f@max_speed     = fit01(rand(@id + 0.73), 11.1, 17.7);  // m/s
f@max_accel     = fit01(rand(@id + 0.31),  0.7,  1.2);  // m/s²
f@max_decel     = fit01(rand(@id + 0.31),  0.9,  1.5);  // m/s²

// ── 速度（INTEGRATEが毎フレーム更新） ─────────────────
f@vel   = 0.0;
f@accel = 0.0;
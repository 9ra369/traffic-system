// ── CRUISING: IDM ────────────────────────────────────
if (i@car_state == 0) {
    float v0    = f@max_speed;
    float T     = 1.5;
    float s0    = 8;
    float a     = f@max_accel;
    float b     = f@max_decel;
    float delta = 4.0;

    float v_lead, s;
    if (i@car_ptnum >= 0) {
        v_lead = f@car_vel;
        s      = max(f@dist_to_car, 0.1);
    } else {
        v_lead = v0;
        s      = 9999.0;
    }

    float free_road_term   = pow(f@vel / max(v0, 0.01), delta);
    float delta_v          = f@vel - v_lead;
    float s_star           = s0 + f@vel * T + (f@vel * delta_v) / (2.0 * sqrt(a * b));
    float interaction_term = pow(s_star / s, 2);
    f@accel = a * (1.0 - free_road_term - interaction_term);
    
    if(@accel<0) i@col = 1;
    else if(@vel<0.05) i@col = 2;

// ── BRAKING: 信号・歩行者 ────────────────────────────
} else if (i@car_state == 1) {
    if (f@effective_dist <= 1) {
        f@accel = -f@required_decel * 1.2;
    } else {
        f@accel = -f@required_decel;
    }
    if (f@vel < 0.1) {
        f@accel     = 0.0;
        f@vel       = 0.0;
        i@car_state = 2;
    }
// ── STOPPED ──────────────────────────────────────────
} else if (i@car_state == 2) {
    f@accel = 0.0;
    f@vel   = 0.0;
}
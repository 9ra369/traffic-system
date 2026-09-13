// ════════════════════════════════════════════════════
// CAR DETECT: 前方の最近傍車両（同レーン先行車 + 交差/右左折車）
// 書く: i@car_ptnum, f@dist_to_car, f@car_vel, i@car_signal, i@car_is_cross
// ════════════════════════════════════════════════════
i@car_ptnum    = -1;
f@dist_to_car  = 9999.0;
f@car_vel      = 0.0;
i@car_signal   = -1;
i@car_is_cross = 0;

float same_dot     = 0.5;   // 同レーン先行車のコーン
float cross_dot    = 0.7;   // 交差/右左折車のコーン
float cross_half_w = 2.0;   // 自車進路の半幅[m]

int pts[] = nearpoints(0, @P, @search_radius);

foreach(int pt; pts) {
    if (pt == @ptnum) continue;
    if (point(0, "car", pt) != 1) continue;

    vector to_other = point(0, "P", pt) - @P;
    float  d = length(to_other);
    if (d < 1e-4) continue;
    float fwd_dot = dot(to_other / d, v@dir);

    int same_lane = (point(0, "lane_name", pt) == s@lane_name);
    int is_cross  = 0;

    if (same_lane) {
        if (fwd_dot < same_dot) continue;
    } else {
        if (fwd_dot < cross_dot) continue;
        // コーンだけだと遠方で隣レーンを拾うので横オフセットでも絞る
        float forward = dot(to_other, v@dir);
        float lateral = length(to_other - forward * v@dir);
        if (lateral > cross_half_w) continue;
        is_cross = 1;
    }

    if (d < f@dist_to_car) {
        f@dist_to_car  = d;
        i@car_ptnum    = pt;
        i@car_is_cross = is_cross;

        // IDM用: 自車進行方向へ射影した前方速度
        float  v_other   = point(0, "vel", pt);
        vector dir_other = point(0, "dir", pt);
        f@car_vel = is_cross
                  ? max(v_other * dot(dir_other, v@dir), 0.0)
                  : v_other;

        int front_state = point(0, "car_state", pt);
        i@car_signal = (front_state == 1 || front_state == 2) ? 1 : 0;
    }
}
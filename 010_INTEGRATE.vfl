f@dt = @TimeInc;
if (i@car_state == 2) {
    f@vel = 0.0;
    @v    = {0, 0, 0};
} else {
    // ★ min_curve_speed / right_turn_speedは固定値を持たず、avg_speed_kmhからの
    //   比率で自動算出する（01_INITのmax_speed_kmh/avg_speed_kmhと連動）。
    //   比率はcurve_speed_ratio/right_turn_speed_ratioチャンネルで調整可能
    //  （既定: カーブ=avgの40%, 右折crawl=avgの15%）。
    float avg_speed_ms    = chf("avg_speed_kmh") / 3.6;
    float min_curve_speed = avg_speed_ms * chf("curve_speed_ratio");
    float right_turn_speed = avg_speed_ms * chf("right_turn_speed_ratio");

    // curvature(0-1に正規化済み)から曲がり用の速度上限を作る
    // curv=0 → max_speed, curv=1 → min_curve_speed
    float curve_limit = fit(f@curv, 0, 1, f@max_speed, min_curve_speed);

    // 右折時はさらに速度上限をかける（対向車確認のため、curvが緩くても徐行させる）
    // ★ crawlさせるのは「実際に対向車へ譲っている(yield_conflict==1)」ときだけにする。
    //   対向車がいない/goが確定した右折まで無条件でcrawlさせていたのが「右折が遅すぎる」の原因。
    float turn_limit = (s@turn == "right" && i@yield_conflict == 1)
                        ? right_turn_speed : f@max_speed;

    // 通常の加速度で速度更新
    f@vel += f@accel * @dt;

    // 上限は max_speed / curve_limit / turn_limit のうち一番小さいもの
    float speed_limit = min(f@max_speed, min(curve_limit, turn_limit));

    // ★ 上限超過分はclamp()で瞬間的に叩き落とさず、max_decelでなめらかに落とす。
    //   旧実装はcurvがコーナー頂点で急に1.0へ跳ぶ/turn_limitがオンになる瞬間に
    //   1フレームで速度を上限まで即クランプしていたため、物理的にありえない
    //   急減速（「急に大きく減速する」）が起きていた。
    if (f@vel > speed_limit) {
        f@vel = max(speed_limit, f@vel - f@max_decel * @dt);
    } else {
        f@vel = min(f@vel, speed_limit);
    }
    f@vel = max(f@vel, 0.0);

    @v  = v@dir * f@vel;
    @P += @v * @dt;

    // ── レーン中心線への横方向補正 ───────────────────
    // 位置はdirの積分だけで決まり、横ズレを戻す仕組みが無かった。
    // ゲインは「進んだ距離あたり」にして、軌跡が速度に依存しないようにする
    float lane_snap = chf("lane_snap");   // 1/m。0で無効
    int    hp = -1;
    vector uvw = 0;
    if (lane_snap > 0 && i@lane_prim >= 0)
        xyzdist(1, itoa(i@lane_prim), @P, hp, uvw);   // 02が特定したprim1本に限定
    if (hp >= 0) {
        vector onlane = primuv(1, "P", hp, uvw);
        vector fwd    = normalize(v@dir);
        vector lat    = onlane - @P;
        lat -= dot(lat, fwd) * fwd;       // 前後成分は捨てる。06/07の距離判定を動かさない

        f@dbg_lane_lat = length(lat);
        // 極端なズレは別レーンへ引かれている疑い。黙って寄せずに値だけ残す
        if (f@dbg_lane_lat < chf("lane_snap_max"))
            @P += lat * min(lane_snap * f@vel * @dt, 1.0);
    }
}

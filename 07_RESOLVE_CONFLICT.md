// ════════════════════════════════════════════════════
// RESOLVE_CONFLICT: 右折 vs 直進 コンフリクト解決（M1・M2・M4・M5-1実装済み）
// 旧 07_CONFLICT_DETECT / CONFLICTフォルダ一式 を置き換える（_ARCHIVE へ移動済み）。
// 設計・進捗は RESOLVE_CONFLICT/00_OVERVIEW.md 以下のマイルストーンMDを参照。
//
// Input 0: 車両
// 読む: s@turn, v@dir, v@cross_pos, i@passed（直進車、M1でキャッシュ済み）
//      i[]@cross_lines, i@lane_id（右折車、03_UPDATE_LANE_ATTRIB.vflでキャッシュ済み）
// 書く: i@passed, i@passed_frame_count（直進車）
//      i[]@rt_decided, i[]@rt_target_id, i[]@rt_target_lock, v[]@rt_target_cp
//        （右折車、内部状態。cross_linesと同じindexで1本ずつ独立に持つ。M5-1）
//      i@yield_conflict, f@dist_to_conflict（右折車、全indexを集約した値。M4が読む）
// ════════════════════════════════════════════════════

// M1: 直進車の passed 判定（旧ahead。名前を変更）
//   cross_pos/passedのリセット・再取得は lane_changed の瞬間に行う
//   （03_UPDATE_LANE_ATTRIB.vfl の lane_changed ブロックに実装済み）。
if (s@turn == "straight" && i@passed == 0) {
    vector to_cross = v@cross_pos - @P;
    float  d = dot(normalize(v@dir), normalize(to_cross));

    if (d <= 0) {
        // 交錯点はもう後方 → 通過。6フレーム連続で確認できたら確定する
        i@passed_frame_count += 1;
        if (i@passed_frame_count >= 6) {
            i@passed             = 1;
            i@passed_frame_count = 0;   // 確定したのでカウントは掃除しておく
        }
    } else {
        i@passed_frame_count = 0;       // まだ前方 → カウントは貯めない
    }
}
// passed==1の直進車は、右折車側の検索(M2)から除外する。

// M2: 右折車(turn=="right")側 — 交錯する直進車の検索・判断
//   詳細・パラメータの根拠は RESOLVE_CONFLICT/M2_right_turn_search.md 参照。
//   M5-1: cross_linesが複数のlane_idを持つ場合（1本の右折曲線が複数の直進レーンと
//   交錯する）に対応するため、判断ラッチ(rt_decided/rt_target_id/rt_target_lock/
//   rt_target_cp)はcross_linesと同じindexの配列にしてあり、相手レーンごとに完全に
//   独立して「初見TTCで確定・以後保持」のロックを持つ。08_STATEが読む
//   i@yield_conflict/f@dist_to_conflictは、全indexのうちyield中のものだけを見て
//   「最も近い交錯点」に集約する。
if (s@turn != "right") {
    // 右折区間を抜けた（またはそもそも右折でない）→ 状態を一律クリアしておく
    // （次に別の交差点で右折を始めたときにゼロから評価し直すため）
    i[]@rt_decided     = {};
    i[]@rt_target_id   = {};
    i[]@rt_target_lock = {};
    v[]@rt_target_cp   = {};
    i@yield_conflict   = 0;
    f@dist_to_conflict = 9999.0;
} else {
    float road_width    = 3.5;
    float search_radius = road_width * 6.0;   // 要チューニング
    float decide_ttc    = 3.5;                // s
    float v_floor        = 1.5;               // m/s
    float stop_margin    = 1.5;               // m（交錯点手前、これを切ったら止まらず抜ける）

    int cands[] = nearpoints(0, @P, search_radius);   // indexループ全体で使い回す

    // 配列attributeへのindex書き込みは避け、ローカル配列にコピー→ループ内で更新→
    // 最後にまとめて書き戻す（02_BAKE_CONFLICTS_TO_LANE.vflと同じ流儀）。
    int    lanes[]        = i[]@cross_lines;
    int    in_decided[]     = i[]@rt_decided;
    int    in_target_id[]   = i[]@rt_target_id;
    int    in_target_lock[] = i[]@rt_target_lock;
    vector in_target_cp[]   = v[]@rt_target_cp;

    int    out_decided[]     = {};
    int    out_target_id[]   = {};
    int    out_target_lock[] = {};
    vector out_target_cp[]   = {};

    int   any_yield  = 0;
    float best_dist  = 9999.0;
    int   n_cross    = len(lanes);

    for (int idx = 0; idx < n_cross; idx++) {
        int    opp_lane  = lanes[idx];
        int    t_id      = in_target_id[idx];
        int    t_lock    = in_target_lock[idx];
        vector t_cp      = in_target_cp[idx];
        int    t_decided = in_decided[idx];

        // ── 候補探索：このindexの相手レーン(opp_lane)を持ち、未通過・検索半径内の直進車 ──
        int   any_candidate         = 0;
        int   target_seen_this_frame = 0;
        int   nearest_pt            = -1;
        float nearest_d             = 9999.0;

        foreach (int pt; cands) {
            if (pt == @ptnum) continue;
            if (point(0, "passed", pt) == 1) continue;
            if (point(0, "lane_id", pt) != opp_lane) continue;

            any_candidate = 1;
            if (point(0, "id", pt) == t_id) target_seen_this_frame = 1;

            float d = distance(point(0, "P", pt), @P);
            if (d < nearest_d) { nearest_d = d; nearest_pt = pt; }
        }

        // ロック中の相手が候補集合から消えた（通過 or 検索半径外）→ 解除して次点を拾えるようにする
        if (t_id >= 0 && !target_seen_this_frame) {
            t_id   = -1;
            t_lock = 0;
            t_cp   = {0, 0, 0};
        }

        // 新しい相手を初めて捕捉した瞬間だけTTCを評価して確定する（以後はその相手が
        // 通過するまで再評価しない。速度一定ならTTCは時間経過だけで単調に減るため、
        // 毎フレーム再評価すると「進んでは止まる」を繰り返す）
        if (t_id < 0 && nearest_pt >= 0) {
            t_cp = point(0, "cross_pos", nearest_pt);
            float o_vel   = point(0, "vel", nearest_pt);
            float t_other = distance(t_cp, point(0, "P", nearest_pt)) / max(o_vel, v_floor);

            t_id   = point(0, "id", nearest_pt);
            t_lock = (t_other < decide_ttc) ? 1 : 0;
        }

        int has_seen = (t_id >= 0);
        int blocked  = has_seen && (t_lock == 1);

        // ── 判断ラッチ（このindex単独） ──
        if (!any_candidate) {
            t_decided = 0;                                     // 周囲に対象車が本当にいない → none
        } else if (t_decided == 0 && has_seen) {
            t_decided = blocked ? 1 : 2;                        // 初回確定 yield:1 / go:2
        } else if (t_decided == 2 && blocked) {
            t_decided = 1;                                      // 本当に新しく見えた相手がblockingなら即昇格
        }
        // yieldはここでは自動でgoに戻さない（安全側は自動で緩めない）

        // ── 交錯点で居座らない：止まり切れず手前まで来てしまったら通過へ切替 ──
        if (t_decided == 1 && t_id >= 0) {
            float dCp = distance(t_cp, @P);
            if (dCp < stop_margin) {
                t_decided = 2;
            } else {
                any_yield = 1;
                best_dist = min(best_dist, dCp);
            }
        }

        append(out_decided,     t_decided);
        append(out_target_id,   t_id);
        append(out_target_lock, t_lock);
        append(out_target_cp,   t_cp);
    }

    i[]@rt_decided     = out_decided;
    i[]@rt_target_id   = out_target_id;
    i[]@rt_target_lock = out_target_lock;
    v[]@rt_target_cp   = out_target_cp;

    // ── 集約：どれか1つでもyield中なら止まる。距離は一番近いものを使う ──
    i@yield_conflict   = any_yield;
    f@dist_to_conflict = any_yield ? best_dist : 9999.0;
}

// M3: 直進車側の右折車検知は不要と判断（検討の記録は RESOLVE_CONFLICT/M3_straight_car_guard.md）。
//   06_CAR DETECT.vfl の is_cross 判定が代わりに機能する。

// M4: 08_STATE への統合 — 実装済み。
//   ここで書いた i@yield_conflict / f@dist_to_conflict を 08_STATE.vfl が読み、
//   赤信号・歩行者と同列の「止まって待つ」対象として f@dist_to_target と比較する
//   （ctrl_src=3）。減速度は専用式を持たず、既存の required_decel 式にそのまま乗せる。
//   詳細 → RESOLVE_CONFLICT/M4_state_integration.md

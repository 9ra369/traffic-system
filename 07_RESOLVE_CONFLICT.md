// ════════════════════════════════════════════════════
// RESOLVE_CONFLICT: 右折 vs 直進 コンフリクト解決（M1・M2・M4・M5-1・M6実装済み）
// 旧 07_CONFLICT_DETECT / CONFLICTフォルダ一式 を置き換える（_ARCHIVE へ移動済み）。
// 設計・進捗は RESOLVE_CONFLICT/00_OVERVIEW.md 以下のマイルストーンMDを参照。
//
// Input 0: 車両
// 読む: s@turn, v@dir, v@cross_pos, i@passed（直進車、M1でキャッシュ済み）
//      i[]@cross_lines, v[]@rt_target_cp（右折車、03_UPDATE_LANE_ATTRIB.vflで
//        lane_changed時にキャッシュ済み。rt_target_cpはRoad Line側のcross_pos(prim)
//        由来で、追跡中の相手車個体には依存しない。M6b）
// 書く: i@passed, i@passed_frame_count（直進車）
//      i[]@rt_decided, i[]@rt_target_id, i[]@rt_target_lock, v[]@rt_target_cp
//        （右折車、内部状態。cross_linesと同じindexで1本ずつ独立に持つ。M5-1）
//      i@yield_conflict, f@dist_to_conflict（右折車、全indexを集約した値。M4が読む）
// ════════════════════════════════════════════════════

//   cross_pos/passedのリセット・再取得は lane_changed の瞬間に行う
//   （03_UPDATE_LANE_ATTRIB.vfl の lane_changed ブロックに実装済み）。

//直進車と交錯点の位置関係の判断
if (s@turn == "straight" && i@passed == 0) {
    vector to_cross = v@cross_pos - @P;
    float  d = dot(normalize(v@dir), normalize(to_cross));

    if (d <= 0) {
        // 交錯点はもう後方 → 通過。15フレーム連続で確認できたら確定する
        i@passed_frame_count += 1;
        if (i@passed_frame_count >= 15) {
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
//   独立してTTCラッチを持つ（追跡中は毎フレーム再評価、ただしyield確定後はgoに
//   自動で戻さない安全側ラッチ）。rt_target_cpだけは追跡中の相手車ごとの値ではなく、
//   このindex(=交錯レーン)固有の固定値（M6b、03_UPDATE_LANE_ATTRIB.vflでlane_changed時に
//   Road Line側から確定済み）。08_STATEが読む
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
    float search_radius = 36;   // 要チューニング
    float safety_margin = 3.0;                // s（t_other < t_ego + margin でblocking。M6。要チューニング）
    float v_floor        = 1.5;               // m/s
    float vel_stopped    = 0.1;               // m/s（これ未満は停止中とみなす。M6）
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
        int   target_pt              = -1;   // ★ 追跡中の相手(t_id)そのもののpt。毎フレームのTTC再評価に使う
        int   nearest_pt            = -1;
        float nearest_d             = 9999.0;

        foreach (int pt; cands) {
            if (pt == @ptnum) continue;
            if (point(0, "passed", pt) == 1) continue;
            if (point(0, "lane_id", pt) != opp_lane) continue;

            any_candidate = 1;
            if (point(0, "id", pt) == t_id) { target_seen_this_frame = 1; target_pt = pt; }

            float d = distance(point(0, "P", pt), @P);
            if (d < nearest_d) { nearest_d = d; nearest_pt = pt; }
        }

        // ロック中の相手が候補集合から消えた（通過 or 検索半径外）→ 解除して次点を拾えるようにする
        // t_cpはリセットしない：M6bでRoad Line側のcross_pos(prim)から確定した、
        // この交錯レーン(index)固有の値であり、追跡中の相手車個体とは無関係。
        if (t_id >= 0 && !target_seen_this_frame) {
            t_id   = -1;
            t_lock = 0;
        }

        // 新しい相手を初めて捕捉
        if (t_id < 0 && nearest_pt >= 0) {
            t_id      = point(0, "id", nearest_pt);
            target_pt = nearest_pt;
        }

        // ★ 追跡中は毎フレームTTCを再評価する（以前は初めて捕捉した瞬間の1回だけだった
        //   ため、最初に遅い/遠い状態でgo判定が固定されると、その後相手が加速して
        //   本当に危険になっても気づけなかった）。t_decidedはyield→goに自動で戻さない
        //   ので、ここでt_lockが0↔1を行き来してもblockingの一方通行(go→yield昇格)は
        //   崩れず、「進んでは止まる」の往復は起きない。
        if (t_id >= 0 && target_pt >= 0) {
            // t_cpは03_UPDATE_LANE_ATTRIB.vflのlane_changed時にRoad Line側の
            // cross_pos(prim)から既に確定済み（M6b）。ここでは追跡中の相手車からは読まない。
            float o_vel   = point(0, "vel", target_pt);
            float o_accel = point(0, "accel", target_pt);
            float o_dist  = distance(t_cp, point(0, "P", target_pt));

            // TTC(t_other)は等速前提(d/v)ではなく、相手の現在の加速度も使った
            // 等加速度運動の式で解く: d = v0*t + 0.5*a*t^2 → t = (-v0+sqrt(v0^2+2ad))/a
            // （2根のうちこちらが最初に交錯点へ達する正の解）
            float t_other;
            if (abs(o_accel) < 1e-4) {
                t_other = o_dist / max(o_vel, v_floor);          // a≈0 → 等速の式に退化
            } else {
                float disc = o_vel * o_vel + 2.0 * o_accel * o_dist;
                if (disc < 0) {
                    t_other = 9999.0;                             // 減速中で手前に止まる → 到達しない
                } else {
                    t_other = (-o_vel + sqrt(disc)) / o_accel;
                    if (t_other <= 0) t_other = o_dist / max(o_vel, v_floor);   // 保険（等速にフォールバック）
                }
            }

            // M6: t_ego — 固定TTC閾値(decide_ttc)との比較ではなく、自車が交錯点に
            // 着くまでの時間との相対比較でblockingを決める。交錯点(t_cp)はRoad Line側に
            // 焼き込み済みの値（M6b）で、両者に共通の同じ世界座標点。
            // 詳細・積み残し → RESOLVE_CONFLICT/M6_t_ego.md
            float e_dist = distance(t_cp, @P);
            float t_ego;
            if (i@car_state >= 1 && f@vel < vel_stopped) {
                // 停止中：現在0初速 → max_accelで発進したとみなした到達時間（等加速度式）
                t_ego = sqrt(2.0 * e_dist / max(f@max_accel, 0.01));
            } else {
                // 走行中：t_otherと同じ式を自車のP/vel/accelで解く
                if (abs(f@accel) < 1e-4) {
                    t_ego = e_dist / max(f@vel, v_floor);
                } else {
                    float e_disc = f@vel * f@vel + 2.0 * f@accel * e_dist;
                    if (e_disc < 0) {
                        t_ego = 9999.0;                              // 減速中で手前に止まる
                    } else {
                        t_ego = (-f@vel + sqrt(e_disc)) / f@accel;
                        if (t_ego <= 0) t_ego = e_dist / max(f@vel, v_floor);   // 保険（等速にフォールバック）
                    }
                }
            }

            t_lock = (t_other < t_ego + safety_margin) ? 1 : 0;
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

// ════════════════════════════════════════════════════
// Input 1: Road Line  (src_id, lane_id, dir, curv, turn, next_lanes)
// Input 3: Pedestrian (歩行者。"cross"グループ点＝直進レーンの交錯点、lane_name一致で引く)
// ════════════════════════════════════════════════════
//
// ★ レーン追跡の主キーは lane_id（分割・交差点コネクタ生成後の、最終的な1本1本の
//   スプラインを一意に識別するID）を使う。src_id は分割・交差点処理前の「元の構成
//   ライン」のIDで、左右分割や複数車線化によって複数の最終レーン（対向車線・並走
//   車線など）が同じsrc_idを共有しうる。そのままnearpoint/findattribvalのキーに
//   使うと別レーンの点を誤って拾う恐れがあるため、毎フレーム参照・next_lanes解決とも
//   lane_id ベースに統一する（旧: src_idベース。2026-09-10変更、旧版はBACKUP/参照）。

// 切り替え直後: 新しい lane_name から lane_id を再解決
// lane_nameは、末端でnext_lane_nameから値を引き継いでいる
if (i@lane_changed == 1) {
    int pt = nearpoint(1, "@lane_name=" + s@lane_name, @P);
    if (pt >= 0) {
        i@lane_id = point(1, "lane_id", pt);
        i@lane_changed = 0;   // 解決できたらオフ

        // ★ この道路の next_lanes を読んで next_lane_name を1回だけ確定
        //    next_lanes を持つ点(in点)を lane_id で探す
        //    （src_idだと左右分割・複数車線で複数レーンが同じ値を共有するため、
        //     別レーンのnext_lanesを誤って拾う恐れがある。lane_idは最終スプライン
        //     ごとに一意なので、自分の入口点(in点)だけが確実に見つかる）
        int inpt = findattribval(1, "point", "lane_id", i@lane_id);
        // ここで、このレーンの持つ、次のレーンの候補を取得し、次のレーンを決定
        if (inpt >= 0) {
            string nexts[] = point(1, "next_lanes", inpt);
            int n = len(nexts);
            if (n == 1) {
                s@next_lane_name = nexts[0];
            //次のLaneが複数ある時
            } else if (n > 1) {
                int idx = int(rand(@id + i@lane_id) * n);  // 車×道路でseed
                idx = min(idx, n - 1);
                s@next_lane_name = nexts[idx];
            } else {
                s@next_lane_name = "";   // 候補なし(行き止まり等)
            }

            // ★ M1: 新しいレーンの交錯点情報を作り直す（直進車のみ使う）
            i@passed             = 0;
            i@passed_frame_count = 0;
            v@cross_pos          = {0, 0, 0};
            // 交錯点はInput 3(歩行者ジオメトリ)の"cross"グループ点。
            // lane_name一致で1個引く（歩行者のcrossing検索と同じ入力・同じパターン）
            int cp = nearpoint(3, "cross && @lane_name=" + s@lane_name, @P);
            if (cp >= 0) {
                v@cross_pos = point(3, "P", cp);
            }

            // ★ M2: 右折レーンのcross_linesをキャッシュ（直進レーンでは既定の空配列のまま）
            //   ptのprimを見る（lane_name一致＋現在地に最も近い点なので、今まさに
            //   乗っているprim＝コネクタなら曲線側を確実に指す）。
            i[]@cross_lines = {};
            int pr[] = pointprims(1, pt);
            if (len(pr) > 0) {
                i[]@cross_lines = prim(1, "cross_lines", pr[0]);
            }

            // ★ M5-1: cross_linesと同じ長さ・同じindexで判断ラッチ配列を作り直す
            //   （右折区間に入り直すたび、前の交差点の状態を引きずらないようゼロから積む）
            int    n_cross = len(i[]@cross_lines);
            int    new_decided[]     = {};
            int    new_target_id[]   = {};
            int    new_target_lock[] = {};
            vector new_target_cp[]   = {};
            for (int ci = 0; ci < n_cross; ci++) {
                append(new_decided,     0);
                append(new_target_id,   -1);
                append(new_target_lock, 0);
                append(new_target_cp,   {0, 0, 0});
            }
            i[]@rt_decided     = new_decided;
            i[]@rt_target_id   = new_target_id;
            i[]@rt_target_lock = new_target_lock;
            v[]@rt_target_cp   = new_target_cp;
        }
    }
}

// 毎フレーム: 今のレーン(lane_id)上の最寄り点から dir/curv/turn を読み直す。
// ★ lane_id < 0（未解決）でもnearpointは単に見つからずpt=-1を返すだけなので、
//   「lane_id >= 0のときだけ」というガードは無くても安全（内側のif (pt>=0)が
//   両ケースとも同じフォールバックになる）。
int pt = nearpoint(1, "@lane_id=" + itoa(i@lane_id), @P);
if (pt >= 0) {
    v@dir  = point(1, "dir",  pt);
    f@curv = point(1, "curv", pt);
    s@turn = point(1, "turn", pt);   // ★ コネクタ上でなければ既定値("")のまま＝直進扱い
}
// pt < 0 のとき（lane_id未解決、または一致点が見つからない）は前フレームのdir/curv/turnを維持する
// ★ lane_idはlane_changed時に一度確定した「今のレーン」の主キーなので、src_idの
//   ときとは違い、ここで毎フレーム読み直す（上書きする）ことはしない。

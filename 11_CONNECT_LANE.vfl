// Detail Wrangler: 一回しか処理が行われないため、配列を準備し、それから要素を1つずつ取り出して処理する

//INとOUTのポイントを配列に入れる
int in_pts[];  int out_pts[];
foreach(int pt; expandpointgroup(0, "*")) {
    if (point(0, "is_cut_end", pt) == 0) continue;
    if (point(0, "is_entry", pt) == 1) append(in_pts, pt);
    else                           append(out_pts, pt);
}

// 入口点について、1つずつ処理する
foreach(int ip; in_pts) {
    // 入口点について、intersection_id, P, dir, turnを取得
    int    in_intersection_id = point(0, "intersection_id", ip);
    vector p1 = point(0, "P", ip);
    vector d1 = normalize(point(0, "dir", ip));
    string turn_in = point(0, "turn", ip);
    // 元のラインを取得する必要がある → もし直進するなら、区切られる前に一体だったラインに進む
    int prev_lane = point(0,"lane_id",ip);
    int in_src_id = -1;                                     // ★ 元ラインのsrc_id
    int    ip_prims[] = pointprims(0, ip);                // ★
    if (len(ip_prims) > 0) {                              // ★
        in_src_id = prim(0, "src_id", ip_prims[0]);           // ★
    }

    // turn_inから「進める出口の向き」を決める
    string allowed[];
    if (turn_in == "left") {
        allowed = array("left", "straight");
    } else if (turn_in == "right") {
        allowed = array("right", "straight");
    } else {
        allowed = array("straight");
    }

    string candidates[];
    foreach(int op; out_pts) {
        // 同じ交差点(intersection_id一致)の出口を総当たり
        if (point(0, "intersection_id", op) != in_intersection_id) continue;
        // 出口点について、P, dir, turnを取得
        vector p2 = point(0, "P", op);
        vector d2 = normalize(point(0, "dir", op));
        if (dot(d1, d2) < -0.9) continue;
        // 入口の進行方向ベクトルd1と出口の進行方向ベクトルd2の内積を計算し、車の進行方向を決定
        string turn_out;
        if (dot(d1, d2) > 0.9) {
            turn_out = "straight";
        } else if (cross(d1, d2).y > 0) {
            turn_out = "left";
        } else {
            turn_out = "right";
        }
        // allowedの中に、turn_outが含まれているかどうかをチェック
        if (find(allowed, turn_out) < 0) continue;
        // 出口点の情報を取得
        int    out_lane = point(0, "lane_id", op);
        int    out_lane_index  = point(0, "lane_index", op);
        string lane_name = point(0, "lane_name", op);
        int    out_src_id = point(0, "src_id", op);   // ★ 行き先レーンの src_id をコネクタにも複製する
        float  out_curv    = point(0, "curv", op);      // ★ 同様に curv も複製（未設定だとカーブで減速できない）
        // ここで、inからoutへの候補となるlane_nameを格納
        append(candidates, lane_name);
        // 入口点と出口点をつなぐラインの生成開始
        // Houdiniのジオメトリでは「点(point)」と「プリミティブ(primitive)」は別物で、両者を繋ぐのが「頂点(vertex)」
        int prim = addprim(0, "polyline"); // まだ中身が空の"polyline"プリミティブの器だけを作る
        int pt_in = addpoint(0, p1); // 空間上に点を1つ作る（まだどのプリミティブにも属していない）
        setpointattrib(0, "dir", pt_in, d1);
        setpointattrib(0, "lane_id", pt_in, out_lane);
        setpointattrib(0, "lane_index", pt_in, out_lane_index);
        setpointattrib(0, "lane_name", pt_in, lane_name);         // ★ 行き先レーンの lane_name/src_id/curv を継承
        setpointattrib(0, "src_id", pt_in, out_src_id);         //    （これが無いと CHANGE_ROAD/GET_ATTRIBUTE がコネクタ上で車を見失う）
        setpointattrib(0, "curv", pt_in, out_curv);
        setpointattrib(0, "turn", pt_in, turn_out);               // ★ GET_ATTRIBUTEがs@turnを拾えるように点属性化
        string connect_nexts[] = array(lane_name);               // 出口lane_name
        setpointattrib(0, "next_lanes", pt_in, connect_nexts);   // 入口点にpoint属性で付与
        addvertex(0, prim, pt_in); //その点を、プリミティブprimの頂点リストに追加する
        // 内積から、曲がるかどうか判定
        int is_curved = (dot(d1, d2) < 0.9);
        if (is_curved) {
            vector n = cross(d1, d2);
            float denom = dot(n, n); // 外積の2乗
            //n = cross(d1, d2)の大きさは、d1とd2のなす角のsinに比例 → しっかり角度がついているときだけ、計算
            if (denom > 1e-6) {
                vector diff = p2 - p1;
                // 点p1から方向d1へ進んだ直線 と 点p2から方向d2へ進んだ直線 の交点を求めるための公式
                float  t = dot(cross(diff, d2), n) / denom; 
                // p1とp2をつなぐ点を作成し、その点にアトリビュートを付与
                int pt_c = addpoint(0, p1 + d1 * t);
                setpointattrib(0, "dir", pt_c, normalize(d1 + d2));
                setpointattrib(0, "lane_id", pt_c, out_lane);
                setpointattrib(0, "lane_index", pt_c, out_lane_index);
                setpointattrib(0, "lane_name", pt_c, lane_name);
                setpointattrib(0, "src_id", pt_c, out_src_id);
                setpointattrib(0, "curv", pt_c, max(out_curv, 1.0));  // 角の点は最低でもカーブ扱いにする
                setpointattrib(0, "turn", pt_c, turn_out);
                setpointgroup(0, "corner", pt_c, 1);
                addvertex(0, prim, pt_c);
            }
        }
        // 最後に出口側の点を作って、primに繋げる。これでpt_in→(pt_c)→pt_outの線が完成
        int pt_out = addpoint(0, p2);
        setpointattrib(0, "dir", pt_out, d2);
        setpointattrib(0, "lane_id", pt_out, out_lane);
        setpointattrib(0, "lane_index", pt_out, out_lane_index);
        setpointattrib(0, "lane_name", pt_out, lane_name);
        setpointattrib(0, "src_id", pt_out, out_src_id);
        setpointattrib(0, "curv", pt_out, out_curv);
        setpointattrib(0, "turn", pt_out, turn_out);
        addvertex(0, prim, pt_out);
        // ここからは線本体(prim)へのアトリビュート付け。あとで「これは接続線だよ」とか「元は何番か」を判別するための目印
        setprimattrib(0, "is_connector", prim, 1);          // これは接続線ですよ、という印
        setprimattrib(0, "lane_id", prim, out_lane);
        setprimattrib(0, "lane_index", prim, out_lane_index);
        setprimattrib(0, "src_id", prim, in_src_id);              // ★ 元ラインのsrc_idを継承
        setprimattrib(0, "next_lane", prim, lane_name);       // この線がどのレーンに繋がるか
        setprimgroup(0, "connectors", prim, 1);               // 接続線をまとめて選べるようにグループ化
        setprimattrib(0, "is_curved", prim, is_curved);
        setprimattrib(0, "prev_lane", prim, prev_lane);
        setprimattrib(0, "turn", prim, turn_out);   // ★ 追加
        // 曲がってるか直進かで、グループも分けておく（あとで見た目を変えたりするのに便利）
        if (is_curved) setprimgroup(0, "curved", prim, 1);
        else           setprimgroup(0, "straight", prim, 1);
    }
    setpointattrib(0, "next_lanes", ip, candidates);
}

// ===== 全体の流れまとめ =====
// 1. 全点から「切れ端(is_cut_end)」だけを拾い、flowで入口(in_pts)と出口(out_pts)に振り分ける
// 2. 入口点を1つずつ処理する(外側foreach)
//    - turn(左折/右折/直進)から、この入口が繋がって良い曲がり方(allowed)を決める
//    - 同じ交差点(intersection_id一致)の出口を総当たりし(内側foreach)、
//      d1・d2の内積と外積から実際の曲がり方(turn_out)を判定、allowedに含まれる組だけ残す
//    - 条件を満たす入口→出口ペアごとに、接続用のpolylineを1本生成
//      (直進ならpt_in→pt_outの直線、曲がるならp1・p2の延長線の交点pt_cを挟んだ折れ線)
//      同時にlane_id/src_id/turn/is_curvedなどの属性とグループ(connectors/curved/straight)を付与
//    - 見つかった出口のlane_nameを候補としてcandidatesに溜めていく
// 3. 内側foreachが全出口を調べ終えたら、その入口ip自身にcandidatesをnext_lanes属性として書き込む
// → 結果、交差点内の「どの入口からどの出口へ、どの向きに曲がって繋がるか」を表す接続線と、
//   各点が持つ「次に行けるレーン一覧(next_lanes)」が一括で出来上がる
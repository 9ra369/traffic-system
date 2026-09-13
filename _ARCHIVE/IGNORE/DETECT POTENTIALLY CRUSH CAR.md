// ============================================================
// 交差点(衝突候補点)ごとに接近車を検知し、
// 優先度の低い車にstop指示を書き込む
// input 0 : 交差点(衝突候補)点  @P, i[]@sourceprim (交差するレーンprim番号)
// input 1 : 車点群            @P, @dir, i@lane_id / レーンprim @priority
// ============================================================

float rad    = chf("radius");         // 検知半径 (例: 10)
float thresh = chf("dot_threshold");  // 接近判定のdot閾値 (例: 0.7)

// --- この交差点自身が属するレーン ---
int lane = prim(1, "lane_id", i[]@sourceprim[0]);

// --- 周辺の車を収集 ---
int pts[] = nearpoints(1, @P, rad);

int   cand_pt[]   = array();  // 接近候補の点番号
float cand_dist[] = array();  // 距離
int   cand_prio[] = array();  // レーン優先度

foreach(int pt; pts){
    int plane = point(1, "lane_id", pt);
    if(plane == lane) continue;              // 同一レーンは先行車追従の別ロジックなので除外

    vector cpos  = point(1, "P", pt);
    vector delta = cpos - @P;
    float  d     = length(delta);
    if(d < 1e-5) continue;

    vector to_car = delta / d;
    vector cdir   = normalize(point(1, "dir", pt));

    if(dot(cdir, to_car) < -thresh){
        append(cand_pt,   pt);
        append(cand_dist, d);

        // レーンprimの priority 属性(無ければ0扱い)
        int prio = 0;
        int lane_prims[] = findattribval(1, "prim", "lane_id", plane);
        if(len(lane_prims) > 0)
            prio = prim(1, "priority", lane_prims[0]);
        append(cand_prio, prio);
    }
}

// --- 既存互換の単一最接近車情報 ---
int found = len(cand_pt) > 0;
i@coming = found;
i@coming_pt   = found ? cand_pt[argmin(cand_dist)]  : -1;
f@coming_dist = found ? cand_dist[argmin(cand_dist)] : -1;

// ============================================================
// 2台以上が競合 → 優先度(同点なら距離)で通行権を決定
// ============================================================
if(len(cand_pt) >= 2){
    int winner = 0;
    for(int j = 1; j < len(cand_pt); j++){
        if(cand_prio[j] > cand_prio[winner] ||
          (cand_prio[j] == cand_prio[winner] && cand_dist[j] < cand_dist[winner])){
            winner = j;
        }
    }

    for(int j = 0; j < len(cand_pt); j++){
        int pt       = cand_pt[j];
        int stopflag = (j == winner) ? 0 : 1;

        // atomicなmax/minなのでマルチスレッド実行でも安全に集約される
        setpointattrib(1, "stop",      pt, stopflag,     "max"); // 一度立ったstopは他の交差点で0に戻らない
        setpointattrib(1, "stop_dist", pt, cand_dist[j],  "min"); // 一番近い停止条件を優先
    }
}
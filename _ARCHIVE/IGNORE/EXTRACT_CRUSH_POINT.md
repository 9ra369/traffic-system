float rad    = chf("radius");        // 10
float thresh = chf("dot_threshold"); // 0.7 くらい

int lane = prim(1, "lane_id", i[]@sourceprim[0]);  // 既に i@lane_id があるならそれを使う

int   found = 0;
int   hit_pt = -1;
float best_d = 1e9;

int pts[] = nearpoints(1, @P, rad);
foreach(int pt; pts){
    if(point(1, "lane_id", pt) != lane) continue;

    vector cpos = point(1, "P", pt);
    vector delta = cpos - @P;
    float  d = length(delta);
    if(d < 1e-5) continue;               // 同一位置は除外

    vector to_car = delta / d;
    vector cdir   = normalize(point(1, "dir", pt));

    // 車の進行方向が「交点→車」の逆を向いている＝こちらに向かってくる
    if(dot(cdir, to_car) < -thresh){
        found = 1;
        if(d < best_d){ best_d = d; hit_pt = pt; }
    }
}

i@coming     = found;
i@coming_pt  = hit_pt;
f@coming_dist = found ? best_d : -1;
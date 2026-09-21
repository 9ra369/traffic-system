// ════════════════════════════════════════════════════
// CROSSING DETECT: 前方の横断中歩行者を探索し、attribに書く
// 実際には、
// 後続ノードはここで書かれた値を読むだけでよい
//
// Input 1: Road Line  (dir)
// Input 3: Pedestrian  (crossing)
// ════════════════════════════════════════════════════

// ── デフォルト値をリセット ────────────────────────────
i@ped_ptnum    = -1;
i@ped_crossing =  0;
f@dist_to_ped  = 9999.0;

// lane_nameで絞ってから横断中を探す。nearpointsは距離順なので最初の1点が最寄り
int pts[] = nearpoints(3, "@lane_name=" + s@lane_name, @P, f@search_radius);

int ptnum = -1;
foreach (int pt; pts) {
    if (point(3, "ped_crossing", pt) != 1) continue;
    ptnum = pt;
    break;
}


if (ptnum >= 0) {
    vector ped_pos = point(3, "P", ptnum);
    vector to_ped  = ped_pos - @P;
    float  d       = length(to_ped);

    // 検出半径内であることはnearpointsの時点で保証済み。前方判定（dot > 0.3）だけここで行う
    if (dot(v@dir, normalize(to_ped)) > 0.3) {
        i@ped_ptnum    = ptnum;
        i@ped_crossing = 1;
        f@dist_to_ped  = d;
    }
}

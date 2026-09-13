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

// lane_name一致 かつ 横断中、を満たす歩行者のうち最寄りの1人を直接引く。
// どちらも歩行者側(Input 3)の属性なので、グループ指定にまとめて渡せる
// （前方判定だけは自車のdir依存なので、取得後に別途チェックする）。
// ★ maxdist引数にsearch_radiusを渡し、半径外なら最初からptnum=-1にする
//   （04_SIGNAL DETECTと同じ最適化。手動でのd<@search_radius比較は不要）。
int ptnum = nearpoint(3, "@lane_name=" + s@lane_name + " && @ped_crossing=1", @P, @search_radius);

if (ptnum >= 0) {
    vector ped_pos = point(3, "P", ptnum);
    vector to_ped  = ped_pos - @P;
    float  d       = length(to_ped);

    // 検出半径内であることはnearpointの時点で保証済み。前方判定（dot > 0.3）だけここで行う
    if (dot(v@dir, normalize(to_ped)) > 0.3) {
        i@ped_ptnum    = ptnum;
        i@ped_crossing = 1;
        f@dist_to_ped  = d;
    }
}
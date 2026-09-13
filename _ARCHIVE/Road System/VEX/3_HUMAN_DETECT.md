**実装内容：歩行者の横断情報を取得**
・信号機と同じように、検索範囲内を人が横断歩道を進行しているか判断する

**次の工程での処理：この歩行状況と前の信号機の状況から、車の状態を決定**

// ════════════════════════════════════════════════════
// CROSSING DETECT: 前方の横断中歩行者を探索し、attribに書く
// 後続ノードはここで書かれた値を読むだけでよい
//
// Input 1: Road Line  (dir)
// Input 3: Pedestrian  (crossing)
// ════════════════════════════════════════════════════

// ── デフォルト値をリセット ────────────────────────────
i@ped_ptnum    = -1;
i@ped_crossing =  0;
f@dist_to_ped  = 9999.0;

float search_radius = chf("search_radius");
int   pts[]  = nearpoints(3, @P, search_radius);
vector ndir  = normalize(@dir);

foreach (int ptnum; pts) {
    // 横断中でなければスキップ
    if (point(3, "ped_crossing", ptnum) != 1) continue;

    // 前方コーン判定
    vector ped_pos = point(3, "P", ptnum);
    vector to_ped  = ped_pos - @P;
    float  d       = length(to_ped);

    // ✓ 前方（dot > 0.3）でなければスキップ
    if (dot(ndir, normalize(to_ped)) <= 0.3) continue;

    // より近ければ採用
    if (d < f@dist_to_ped) {
        i@ped_ptnum    = ptnum;
        i@ped_crossing = 1;
        f@dist_to_ped  = d;
    }
}
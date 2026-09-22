// ════════════════════════════════════════════════════
// COLLISION: 車同士の点が接触距離まで近づいたことを記録し、その車をピンクにする。
// 書く: i@collided, i@collide_now, i@collide_count, i@collide_id,
//       i@collide_frame, v@collide_pos, f@collide_dist, @Cd
//
// ★ Solverの最後（011_COLORIZEの後ろ）に置く。@Cdを上書きするのと、
//   接触は010_INTEGRATEが位置を動かした後の配置で見たいため。
// ★ 円の表示は23_VIZ_COLLISION（Solverの外）が担当する。
//
// Input 0: 車両（Solver内の自分自身）
// ════════════════════════════════════════════════════

float  collide_dist = chf("collide_dist");   // m。点間距離がこれ以下で接触（既定 0.5）
vector pink         = set(1.0, 0.15, 0.7);

if (i@car != 1) return;

int    was_now = i@collide_now;   // Solverなので属性には前フレームの値が残っている
int    hit     = 0;
int    hit_id  = -1;
vector hit_p   = @P;
float  min_d   = 9999.0;

// 半径で絞るので、ここでの距離比較は不要（nearpointsが半径外を返さない）
int pts[] = nearpoints(0, @P, collide_dist);
foreach (int pt; pts) {
    if (pt == @ptnum) continue;
    if (point(0, "car", pt) != 1) continue;

    vector opp_p = point(0, "P", pt);   // 型を明示して受ける（多重定義回避）
    float  d     = distance(@P, opp_p);
    hit = 1;
    if (d < min_d) {
        min_d  = d;
        hit_p  = opp_p;
        hit_id = point(0, "id", pt);
    }
}

i@collide_now = hit;

if (hit) {
    if (was_now == 0) i@collide_count += 1;              // 接触の立ち上がりだけ数える
    if (min_d < f@collide_dist) f@collide_dist = min_d;  // 一番食い込んだ距離を残す
    if (i@collided == 0) {
        i@collided      = 1;
        i@collide_id    = hit_id;
        i@collide_frame = int(@Frame);
        v@collide_pos   = (@P + hit_p) * 0.5;   // 2台の中点＝ぶつかった地点
    }
}

// ★ ラッチ表示。一度ぶつかった車は削除されるまでピンクのまま残る
//   （接触中だけ塗るなら i@collided を i@collide_now に変える）
if (i@collided == 1) @Cd = pink;

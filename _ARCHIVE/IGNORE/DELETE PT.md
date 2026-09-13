// POP Wrangle
// Input 1 (opinput 1): 道路カーブ
float end_threshold = 0.98;  // 例: 0.98

int prim;
vector primuv;

// 自分と同じ road_name を持つプリムだけを検索対象にする
string grp = sprintf("@road_name=%s", s@road_name);
xyzdist(1, grp, @P, prim, primuv);

if (primuv.x > end_threshold) {
    i@dead = 1;
}
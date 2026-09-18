// ════════════════════════════════════════════════════
// BAKE_CROSS_POS: 直進レーンの交錯点座標(cross_pos)をRoad Line側に焼き込む
// 道路網生成時に1回だけ実行する（Point Wrangle。11_CONNECT_LANEの後段、
// 車Solverとは別系統。車Solverより前に1回だけ通ればよい）。
//
// 背景：右折車(07_RESOLVE_CONFLICT M2)は自分がblockingか判断するために交錯点座標が
// 要るが、従来はそれを「今たまたま近くにいる直進車個体が持つcross_pos」から読んで
// いた（point(0,"cross_pos",target_pt)）。これだと対応する直進車がまだ
// lane_changedを済ませていない/近くに1台もいない、という状況に判断が引きずられる。
// 交錯点は道路形状だけで決まる値（車の状態に一切依存しない）なので、道路網生成時に
// Road Line側へ焼き込み、右折車はlane_idキーで直接引けるようにする。
// 詳細 → RESOLVE_CONFLICT/M6_t_ego.md
//
// Input 0: Road Line（レーンネットワーク。直進レーン・右折レーン双方の各点を処理）
// Input 1: 交錯点（Pedestrianジオメトリの"cross"グループ。candidate属性(lane_name配列)を持つ点群。
//           1つの交錯点が複数レーンのcandidateに含まれうる＝1レーンが複数の交錯点を持ちうる。
//           右折スプラインが複数の直進レーンと交錯する場合などが該当する（M5-1のcross_linesと同じ状況）
//
// 読む: s@turn, s@lane_name（Input 0） / candidate, P（Input 1）
// 書く: v[]@cross_pos（Input 0。point属性・prim属性の両方に焼き込む。複数ヒットしうるため配列）
// ════════════════════════════════════════════════════

// 直進・右折の両方が対象（右折 vs 直進コンフリクトの当事者双方。07_RESOLVE_CONFLICT参照）。
// 左折はコンフリクト判定の対象外なので除外する。
if (s@turn != "straight" && s@turn != "right") return;

// candidateはlane_nameの配列（1つの交錯点が複数レーンに対応しうる）なので、
// findattribvalの単一完全一致では引けない → 交錯点を総当たりし、findで配列に
// lane_nameが含まれるかを見る（11_CONNECT_LANE.vflのcandidates/findと同じ流儀）
vector cross_pos[];
int n1 = npoints(1);
for (int cp = 0; cp < n1; cp++) {
    string candidate[] = point(1, "candidate", cp);
    if (find(candidate, s@lane_name) < 0) continue;
    vector pos = point(1, "P", cp);
    append(cross_pos, pos);
}
if (len(cross_pos) == 0) return;   // 対応する交錯点が無いレーン（対象外。歩行者横断のみの区間など）

v[]@cross_pos = cross_pos;   // point属性として持たせる（配列）

// 右折車側（07_RESOLVE_CONFLICT）がlane_idキー→prim経由で直接引けるよう、
// prim側にも同じ値を複製する（lane_id(prim)/turn(prim)などと同じ流儀）
int prims[] = pointprims(0, @ptnum);
foreach (int pr; prims) {
    setprimattrib(0, "cross_pos", pr, cross_pos, "set");
}

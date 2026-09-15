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
// Input 0: Road Line（レーンネットワーク。直進レーンの各点を処理）
// Input 1: 交錯点（Pedestrianジオメトリの"cross"グループ。lane_name属性を持つ点群）
//
// 読む: s@turn, s@lane_name（Input 0） / lane_name, P（Input 1）
// 書く: v@cross_pos（Input 0。point属性・prim属性の両方に焼き込む）
// ════════════════════════════════════════════════════

if (s@turn != "straight") return;   // 直進レーンのみが対象（cross_curveと同じ前提。M1_ahead_flag.md）

// lane_nameは空間的な近さではなく一意キーでの完全一致なので、
// nearpointではなくfindattribvalで引く（03_UPDATE_LANE_ATTRIB.vflのlane_id照合と同じ流儀）
int cp = findattribval(1, "point", "lane_name", s@lane_name);
if (cp < 0) return;   // 対応する交錯点が無いレーン（対象外。歩行者横断のみの区間など）

vector pos = point(1, "P", cp);
v@cross_pos = pos;   // point属性として持たせる

// 右折車側（07_RESOLVE_CONFLICT）がlane_idキー→prim経由で直接引けるよう、
// prim側にも同じ値を複製する（lane_id(prim)/turn(prim)などと同じ流儀）
int prims[] = pointprims(0, @ptnum);
foreach (int pr; prims) {
    setprimattrib(0, "cross_pos", pr, pos, "set");
}

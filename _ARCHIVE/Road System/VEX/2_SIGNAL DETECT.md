**実装内容：信号機の点灯情報を取得**
・道路のスプラインから、進行方向を毎フレーム、進行方向の@dirを取得
・信号機が前にあるかどうか、判定(内積を用いて、前方144°以内を検索)
・自分が走行している道路と同じ道路の信号機(@road_nameが一致)なら、信号機の情報を取得

**次の工程での処理：この信号機の情報から、車の状態を決定**

// ════════════════════════════════════════════════════
// SIGNAL DETECT: 最近傍信号機を探索し、関連情報をattribに書く
// 後続ノードはここで書かれた値を読むだけでよい
//
// Input 1: Road Line   (dir, road_name)
// Input 2: Traffic Signal (signal_state, road_name)
// ════════════════════════════════════════════════════

// ── デフォルト値をリセット（信号が見つからない場合の保証） ──
i@signal_ptnum   = -1;
i@signal_state   = -1;
f@dist_to_signal = 9999.0;

// ── 自分の道路名と進行方向を取得 ──────────────────────
int road_pt      = nearpoint(1, @P);
v@dir            = point(1, "dir",       road_pt);
string road_name = point(1, "road_name", road_pt);

float search_radius = chf("search_radius");

// ── Input 2 (Traffic Signal) から最近傍点を探索 ──────
int ptnum = nearpoint(2, @P, search_radius);
if (ptnum < 0) return; // 範囲内に信号なし → デフォルト値のまま終了

// ── 同じ道路の信号機かどうか確認 ──────────────────────
string signal_road = point(2, "road_name", ptnum);
if (road_name != signal_road) return; // 別の道路の信号機 → 無視

// ── 前方判定 ──────────────────────────────────────────
vector signal_pos = point(2, "P", ptnum);
vector to_signal  = signal_pos - @P;
float  front_dot  = dot(normalize(@dir), normalize(to_signal));

if (front_dot <= 0.3) return; // 前方コーン外 → 無視

// ── 有効な信号機が見つかった → attributに書き込む ──────
i@signal_ptnum   = ptnum;
i@signal_state   = point(2, "signal_state", ptnum);
f@dist_to_signal = length(to_signal);
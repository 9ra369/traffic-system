---

title: 交通シミュレーション — 右左折・車線変更・IDM統一停止システム

created: 2026-07-03

tags: [traffic-simulation, houdini, vex, javascript, idm, bezier, digital-twin]

type: technical-note

status: complete

related: [[IDM]], [[Bezier曲線]], [[Catmull-Rom]], [[交通シミュレーション Web版 設計書]]

---

  

# 交通シミュレーション — 右左折・車線変更・IDM統一停止システム

  

Web版（Canvas/JS）で確立した右左折・車線変更・停止制御のシステムを、Houdini実装の観点も含めて整理する。Web版で実際に踏んだバグ（縮退サンプル → コネクタのフック反転）はHoudiniでも同型の問題が起きるため、事例研究として最後にまとめる。

  

---

  

## 1. 全体アーキテクチャ — 3層構造

  

システムは3層に分離されている。この分離が右左折を「簡単な問題」に変えている。

  

| 層 | 責務 | 実体 |

|---|---|---|

| グラフ層（ルーティング） | どの道をどの順で走るか | ノード + 有向エッジ、`links` 配列 |

| ジオメトリ層（形状） | 各エッジの空間的な形 | 弧長パラメータ化された点列 `centerSamples` |

| 運動層（走行） | 加減速・車間・停止 | IDM + 仮想障害物、`(edgeId, t)` の1次元状態 |

  

最重要の設計判断は「**車の状態は `(edgeId, t)` の1次元**」であること。車はワールド座標を持たず、「どのエッジの、始点から何pxの位置か」だけを持つ。ワールド座標と向きは描画時にエッジの点列から引くだけ（読み取り専用）。これにより:

  

- 右左折 = 「エッジIDの乗り換え」に還元される（旋回の運動計算は不要）

- IDMの車間距離 = 同一エッジ上の `t` の差分（1次元の引き算）

- 毎フレームの `xyzdist` 的な最近接点探索が完全に不要になる

  

---

  

## 2. 右左折システム

  

### 2.1 設計思想 — 旋回は「走行時に計算」せず「ジオメトリとして事前生成」する

  

車が交差点で曲がるとき、車は何も特別なことをしない。**「レーンエッジ → ターンコネクタ → レーンエッジ」と、エッジIDを2回乗り換えるだけ**。旋回の滑らかさはコネクタの曲線品質が100%決めるので、問題は「走行ロジック」から「曲線生成」に完全に切り離される。

  

Houdiniで右左折のカーブ生成が難しかった原因の多くは、この分離をせずに「走行中の車の向きを補間する」「交差点内でカーブをその場で解く」方向に行ってしまうことにある。事前生成に振り切るのが正解。

  

### 2.2 パイプライン（4ステップ）

  

**Step 1: 有向レーンカーブの生成。** 街路（無向）1本から、左側通行の有向エッジを2本作る。中心線を等弧長リサンプリングし、各点の接線（heading）の直交方向へキャリッジウェイ幅ぶんオフセットする。

  

**Step 2: 交差点トリム（今回の最大の教訓）。** レーンエッジを交差点中心から `NODE_RADIUS` ぶん両端切り詰めて、**旋回用の空間を空ける**。これをサボって道路をノード中心まで通したままコネクタを張ると、始点と終点の位置関係が破綻し（P0がP1より奥に来る）、ベジエが必ずフック形状になる。実際の交差点も「停止線 = 交差点境界、旋回パスは内部」という同じ構造。

  

**Step 3: コネクタ生成（3次ベジエ）。** 各交差点で「流入エッジ × 流出エッジ」の全組み合わせ（同一街路へのUターンは除外。行き止まりのみ許可）に対して1本ずつ:

  

```

P0 = 流入エッジ終端の走行レーン位置

T0 = その点の接線（単位ベクトル）

P1 = 流出エッジ始端の走行レーン位置

T1 = その点の接線

  

d  = |P1 - P0| × 0.45          // ハンドル長

B1 = P0 + T0 × d

B2 = P1 - T1 × d

  

コネクタ = CubicBezier(P0, B1, B2, P1)

```

  

両端で接線を強制するので、直進・右折（大回り）・左折（小回り）が**すべてこの1つの式**で滑らかに繋がる。ハンドル係数 0.45 は経験値。円弧に近づけたい場合は接線間角度 θ から `d = R·(4/3)·tan(θ/4)`（ベジエによる円弧近似の標準式）も使える。

  

**Step 4: 弧長リサンプリング。** 生成したコネクタも必ず等弧長でサンプリングし直し、`t`（累積弧長）と heading を各点に持たせる。これで IDM の `t` が「レーン → コネクタ → レーン」を通して**連続した実走行距離**になり、曲線上でも速度が一定に保たれる。

  

### 2.3 走行時の処理（乗り換えだけ）

  

```js

// エッジ遷移（updateループ内）

while (car.t >= edge.length) {

  car.t -= edge.length;                       // 余った距離を持ち越す

  if (edge.kind === "turn")

    car.edgeId = edge.exitEdgeId;             // ターン終了 → 流出レーンへ

  else

    car.edgeId = car.routeLink;               // レーン終了 → 事前選択済みコネクタへ

  edge = edgeById[car.edgeId];

}

```

  

ルート選択（どのコネクタに入るか）はエッジ終端の `NODE_LOOKAHEAD`(50px) 手前で事前確定する。これは前方先読み（次エッジ上の車を前車として見る）のために必要で、遷移の瞬間に選ぶのでは遅い。

  

---

  

## 3. Houdini実装ガイド

  

### 3.1 データモデル（アトリビュート設計）

  

道路ネットワークは1つのジオメトリストリームに、**プリミティブ = エッジ**として持つ。

  

| 対象 | アトリビュート | 型 | 内容 |

|---|---|---|---|

| prim (全エッジ) | `s@kind` | string | `"lane"` / `"turn"` |

| prim | `s@from_node`, `s@to_node` | string | 接続ノードID |

| prim | `f@length` | float | 弧長（Measure SOPで計測） |

| prim (lane) | `i@sibling` | int | 対向エッジのprim番号（Uターン除外用） |

| prim (lane) | `i[]@links` | int array | このエッジから入れるコネクタのprim番号群 |

| prim (turn) | `i@exit_edge` | int | 流出レーンのprim番号 |

| point (カーブ上) | `v@tangentu` | vector | 接線（PolyFrameで生成） |

| 別ジオメトリ: 車 | `i@edge`, `f@t`, `f@speed`, `f@max_speed`, `i@lane`, `f@lane_frac`, `i@route_link` | — | Web版の car オブジェクトと1:1対応 |

  

車は**Solver SOP内のポイント群**として、`(edge, t)` 空間でシミュレートする。POP Solverは3D空間の力学が主戦場なので、グラフ空間のシミュレーションにはSolver SOP + Wrangleの方が素直。

  

### 3.2 SOPネットワーク構成

  

```

[道路中心線カーブ] (ノード位置はpoint、街路はcurve)

    │

    ├─ Fuse                     … 重複点の除去（★バグ予防。§6参照）

    ├─ Resample (等弧長, 0.05m) … 弧長パラメータ化の土台

    ├─ PolyFrame                … tangentu 生成 (Style: First Edge)

    │

    ├─ Point Wrangle            … 方向別オフセット（有向レーンカーブ×2生成）

    │    ※実際には For-Each で正/逆2回、逆方向は Reverse SOP 後に同処理

    │

    ├─ Measure (perimeter→length) + 交差点トリム

    │    方法A: For-Each Primitive + Carve（u1 = R/length を prim() 式で）

    │    方法B: VEXで点列再構築（Web版 trimSamples と同じ）

    │    ※どちらでも良い。要点は「交差点空間を確実に空ける」こと

    │

    ├─ Detail Wrangle           … コネクタ生成（§3.3のVEX）

    ├─ Resample (等弧長)        … コネクタも必ず弧長パラメータ化

    ├─ Measure + PolyFrame      … length / tangentu を最終ジオメトリで再計測（★重要）

    │

[Solver SOP]  input1 = 上記ネットワーク

    │  内部: 車ポイントに対する Point Wrangle 群

    │    ① ルート選択  ② 車線変更  ③ IDM加速度  ④ 積分+エッジ遷移

    │

    ├─ Point Wrangle            … ワールド座標投影（§3.5）

    └─ Copy to Points           … 車ジオメトリ配置（orient使用）

```

  

### 3.3 コネクタ生成のVEX（核心部分）

  

```c

// Detail Wrangle — Run Over: Detail

// input0: トリム済み有向レーンカーブ

//   (prim attr: from_node, to_node, sibling, kind="lane" / point attr: tangentu)

float d_ratio = chf("handle_ratio");   // 0.45

int   res     = chi("resolution");     // 24〜32

  

int nl = nprimitives(0);

for (int a = 0; a < nl; a++) {

    string toN = prim(0, "to_node", a);

    // 流入エッジ a の終端位置と接線（等弧長リサンプル済みなら u=1 が正確な終端）

    vector p0 = primuv(0, "P",        a, {1,0,0});

    vector t0 = normalize(primuv(0, "tangentu", a, {1,0,0}));

  

    for (int b = 0; b < nl; b++) {

        if (b == a) continue;

        if (prim(0, "from_node", b) != toN) continue;   // 同一ノード接続のみ

        if (prim(0, "sibling",   a) == b)   continue;   // Uターン除外

  

        vector p1 = primuv(0, "P",        b, {0,0,0});

        vector t1 = normalize(primuv(0, "tangentu", b, {0,0,0}));

  

        float dist = distance(p0, p1);

        float d    = max(dist * d_ratio, 0.01);

        vector b1  = p0 + t0 * d;

        vector b2  = p1 - t1 * d;

  

        // 3次ベジエを手評価してポリライン化

        int pr = addprim(0, "polyline");

        for (int i = 0; i <= res; i++) {

            float u = i / float(res), v = 1.0 - u;

            vector P = v*v*v*p0 + 3*v*v*u*b1 + 3*v*u*u*b2 + u*u*u*p1;

            addvertex(0, pr, addpoint(0, P));

        }

        setprimattrib(0, "kind",      pr, "turn");

        setprimintattrib(0, "exit_edge", pr, b);

        // 流入エッジ a の links 配列にも pr を追記（appendした配列をsetback）

        int lk[] = prim(0, "links", a);

        append(lk, pr);

        setprimattrib(0, "links", a, lk);

    }

}

```

  

生成後に **Resample → Measure → PolyFrame を必ずやり直す**。ベジエの媒介変数 u は弧長に比例しないので、リサンプルしないと `primuv` の u と走行距離が一致せず、コネクタ上で速度がムラつく（Web版で `resampleByArcLength` を挟んでいるのと同じ理由）。

  

### 3.4 SolverのVEX（IDM + エッジ遷移）

  

```c

// Solver SOP 内 Point Wrangle（車ポイント）

// input0 = 前フレームの車 / input1 = 道路ネットワーク

#define CAR_LEN 4.0

#define A_MAX 2.0

#define B_DEC 2.5

#define S0 2.0

#define T_HW 1.2

  

// --- ① 前車探索（同一エッジ・同一車線。O(n^2)、数百台なら十分）---

float best = 1e9; float leadv = 0;

for (int i = 0; i < npoints(0); i++) {

    if (i == @ptnum) continue;

    if (point(0, "edge", i) != i@edge) continue;

    if (point(0, "lane", i) != i@lane) continue;

    float g = point(0, "t", i) - f@t;

    if (g > 0 && g < best) { best = g; leadv = point(0, "speed", i); }

}

// ここで信号停止線・歩行者・合流待ちの「仮想障害物」も

// (gap, leadv=0) として best と比較し、最小 gap を採用する（§5）

  

// --- ② IDM ---

float v = f@speed, v0 = f@max_speed, a;

if (best > 9e8) {

    a = A_MAX * (1 - pow(v / v0, 4));

} else {

    float s  = max(best - CAR_LEN, 0.1);

    float dv = v - leadv;

    float ss = S0 + max(0.0, v * T_HW + v * dv / (2 * sqrt(A_MAX * B_DEC)));

    a = A_MAX * (1 - pow(v / v0, 4) - (ss / s) * (ss / s));

}

f@speed = clamp(v + a * f@TimeInc, 0, v0);

f@t += f@speed * f@TimeInc;

  

// --- ③ エッジ遷移 ---

float elen = prim(1, "length", i@edge);

while (f@t >= elen) {

    f@t -= elen;

    string k = prim(1, "kind", i@edge);

    if (k == "turn") {

        i@edge = prim(1, "exit_edge", i@edge);

    } else {

        int lk[] = prim(1, "links", i@edge);

        int n = len(lk);

        i@edge = lk[min(int(rand(set(@ptnum, f@t, @Frame)) * n), n - 1)];

    }

    elen = prim(1, "length", i@edge);

}

```

  

### 3.5 ワールド座標への投影（Solver後、描画専用）

  

```c

// Point Wrangle — input1 = 道路ネットワーク

float u    = f@t / prim(1, "length", i@edge);

vector pos  = primuv(1, "P",        i@edge, set(u, 0, 0));

vector tang = normalize(primuv(1, "tangentu", i@edge, set(u, 0, 0)));

vector side = normalize(cross({0,1,0}, tang));   // 符号は巻き方向で調整

  

string k  = prim(1, "kind", i@edge);

float lat = (k == "turn") ? 0.0

          : (f@lane_frac - (LANES_PER_DIR - 1) * 0.5) * LANE_WIDTH;

  

v@P     = pos + side * lat;

p@orient = quaternion(maketransform(tang, {0,1,0}));

```

  

等弧長リサンプル済みなら u ∝ 弧長 が成り立つので、`primuv` 1回で位置と接線が正確に引ける。**毎フレームの xyzdist は一切不要**（車がカーブから離れないので、最近接点探索という問題自体が存在しない）。

  

### 3.6 Houdini固有の落とし穴

  

1. **PolyFrame前のFuse必須。** 重複点・ゼロ長セグメント上のPolyFrameは不定な接線を返す。Web版のフックバグ（§6）と完全に同型。

2. **オフセット後・トリム後・コネクタ生成後は length / tangentu を再計測。** 曲がった道路ではオフセット後に弧長が変わる（内周は短く外周は長い）。古い length を使い回すと u マッピングとトリム位置がずれる——これもWeb版バグの本質と同じ「帳簿の不整合」。

3. **Carveは媒介変数u基準。** 等弧長リサンプル後でないと「Rメートルだけ切る」にならない。

4. **単位系。** Web版はpx、Houdiniはm推奨。§7の対照表で換算。

  

---

  

## 4. 車線変更システム

  

### 4.1 2層モデル — lane（意思）と lane_frac（身体）

  

最大の設計判断: **車線は独立したカーブではなく、キャリッジウェイカーブ上の「横方向スカラー」**である。

  

```js

car.lane      // int:   目標車線（意思決定。0 or 1）

car.laneFrac  // float: 表示上の連続位置（0.0〜1.0を滑らかに遷移）

```

  

- `lane` は判定ロジックが**即座に**切り替える。IDMの前車探索は `lane` 基準なので、変更を決めた瞬間から新しい車線の車間を守り始める（安全性は意思決定側で担保）。

- `laneFrac` は毎フレーム `LANE_CHANGE_RATE`(2.8車線/秒 ≒ 1車線0.36秒) で `lane` に向かって線形に追従する（見た目の滑らかさ担保）。

- ワールド座標は `カーブ位置 + 直交方向 × laneLateral(laneFrac)` で決まる。

  

車線変更のために**ジオメトリを一切触らない**（別カーブへの乗り移り・再投影が不要）のがこの方式の利点。トレードオフとして全車線が同じ弧長を共有する（実際はカーブ内周の車線の方が短い）が、この規模では許容誤差。

  

### 4.2 判定ルール（優先度順）

  

```

1. ターンコネクタ上   → 強制 lane 0（交差点内は単列）

2. ノード手前 MERGE_ZONE(95px) 内

                      → lane 0 へ合流（laneSafe なら即時、ダメなら毎フレーム再試行）

3. それ以外の直線区間 → 追い越し判定:

     発動条件: 前車ギャップ < OVERTAKE_TRIGGER(72px)

     利得条件: 隣車線の前方ギャップ > 自車線ギャップ + OVERTAKE_GAIN(45px)

     安全条件: laneSafe = 変更先車線の [-26px, +30px] 帯に車がいない

```

  

これは学術モデルでいう **MOBIL** (Minimizing Overall Braking Induced by Lane changes) の簡略版。MOBILは「利得 = IDM加速度の改善量」「安全 = 変更先後続車の減速が閾値内」で判定するが、本実装はそれをギャップ距離の比較に落としている。IDMの数理を深掘りするなら、次のアップグレード先はMOBIL本式（判定にIDM加速度そのものを使うので、既存の `idmAcceleration` がそのまま流用できる）。

  

### 4.3 合流ブロッカー（横テレポート対策）

  

コネクタは lane 0 の位置に張られているため、lane 1 のままエッジ終端に達すると進入時に車線幅ぶん横へテレポートする。これを**構造的に**防ぐのが合流ブロッカー:

  

```js

// 合流未完了（lane≠0 または laneFrac>0.05）のままMERGE_ZONE内にいる車には、

// 停止線位置に「速度0の仮想先行車」を見せる

function findMergeObstacle(car) {

  if (edge.kind !== "lane") return null;

  if (car.lane === 0 && car.laneFrac < 0.05) return null;  // 合流完了

  const gap = (edge.length - STOP_LINE_OFFSET) - car.t;

  if (gap <= 0 || gap > MERGE_ZONE) return null;

  return { car: { speed: 0 }, gap };

}

```

  

合流できない車はIDMが自然に停止線で待たせる。lane 0 の車列が流れればギャップが開き、`laneSafe` が真になった瞬間に合流して発進する。デッドロックしない（lane 0 側は止まる理由がなく、信号で止まっていても青で必ず流れる）。

  

### 4.4 Houdiniへの写像

  

そのまま移る。`i@lane` / `f@lane_frac` を車ポイントに持たせ、判定Wrangleを積分Wrangleの前に置く。横オフセットは§3.5の投影Wrangleで既に処理済み。`laneSafe` / `laneGapAhead` は前車探索と同じO(n²)ループで書ける。

  

---

  

## 5. 停止判定の統一 — 「右左折中の前車停止はIDM？」への回答

  

**はい、その認識で合っている。** さらに言えば、前車だけでなく**この系のすべての減速・停止がただ1つのIDM式**で処理されている。個別の「停止ロジック」は存在しない。

  

### 5.1 effectiveLead — 4種類の障害物を1本化

  

毎フレーム、各車について以下の4候補から**最も近いもの1つ**を選び、IDMに渡す:

  

```

effectiveLead(car) = min-gap of {

  ① 実先行車     … 同一エッジ・同一車線の前方直近（speed = 前車の実速度）

  ② 赤/黄信号     … 停止線位置の仮想車（speed = 0）

  ③ 横断中の歩行者 … 横断歩道位置の仮想車（speed = 0）

  ④ 合流ブロッカー … 停止線位置の仮想車（speed = 0）

}

```

  

IDM側は渡された `(gap, 相手速度)` が本物の車か停止線かを区別しない。動く相手なら追従、止まった相手（speed=0）なら手前で停止——同じ式の連続的な帰結として両方が出てくる。「信号で止まる」「歩行者を待つ」「合流待ち」はすべて「止まっている車の後ろについた」のと数学的に同一。

  

### 5.2 エッジ境界の先読み（右左折時の前車検出の実体)

  

右左折の場面で効いているのはこの部分。前車探索は同一エッジ内だけでなく、**エッジ境界をまたいで次エッジの車も見る**:

  

```js

// findLeadCar 内

const remaining = edge.length - car.t;

if (remaining < NODE_LOOKAHEAD && nextEdge) {

  for (const o of cars) {

    if (o.edgeId !== nextEdge) continue;

    const gap = remaining + o.t;   // 境界をまたいだ連続距離

    if (gap < best.gap) best = { car: o, gap };

  }

}

```

  

- レーン上でノードに接近中 → 事前確定した**コネクタ上の車**が前車候補になる

- コネクタ上を旋回中 → `exitEdgeId` の**流出レーン上の車**が前車候補になる

  

`gap = 残距離 + 相手のt` という単純な足し算で成立するのは、全エッジが弧長パラメータ化されているから。つまり「右左折中に前の車がいたら止まる」は、**先読み付きの前車検出（グラフ層）+ IDM（運動層）**の組み合わせで、旋回そのものは何も関与していない。

  

### 5.3 既知の制限（正直な線引き）

  

- 先読みは1エッジ先まで。極端に短いコネクタ越しの2エッジ先は見えない（NODE_LOOKAHEAD=50pxで実用上カバー）。

- 交差点内で**交差する別コネクタ同士**の衝突判定はない。信号フェーズ（全赤クリアランス含む）による時間分離が前提。無信号交差点を作るなら、コネクタ同士の交差点（conflict point）を事前計算し、優先権ルールで仮想障害物化する拡張が必要——これも「仮想先行車としてIDMに渡す」枠組みにそのまま乗る。

  

---

  

## 6. バグ事例研究 — 縮退サンプルが生むフック反転

  

T字路で観測された「少し進む → 急旋回 → 反転して曲がる」の根本原因。Houdiniで同型バグを踏まないための記録。

  

**連鎖の全体像:**

  

```

道路長がサンプル間隔の整数倍

  → 終端点が最後の等間隔点と完全一致（ゼロ長セグメント）

  → atan2(0,0) = 0° のゴミheadingが最終サンプルに混入

  → 東向きエッジ: 偶然 0°=正解 → 無症状（バグが片方向にしか出ない）

  → 西向きエッジ: 正解180°のところに0° → オフセットが最終点だけ逆側へ

  → 末尾に20pxの偽キンク → 弧長が340→360pxに水増し

  → 水増しした長さ基準でトリム → トリム位置が20pxずれ、エッジが交差点にめり込む

  → コネクタのP0がP1より奥に → ベジエが接線を守ろうとして後ろ向きフック

  → 車がフックをなぞって「反転」

```

  

**教訓:**

  

1. **ゼロ長セグメントは方向情報を破壊する。** `atan2(0,0)`、正規化不能ベクトル、重複点上のPolyFrame——すべて同じ穴。Web版の修正は「終端点が直前サンプルとほぼ同一なら追加しない」。Houdiniでは Fuse を接線計算の前に必ず入れる。

2. **片方向にしか出ないバグは縮退を疑う。** ゴミ値が偶然正解と一致する方向では無症状になる。対称なはずの処理（正方向/逆方向エッジ）の結果を突き合わせるテストが有効だった（length 288 vs 308 で即発覚）。

3. **幾何を変えたら帳簿（length/接線）を再計測。** オフセット・トリム・コネクタ生成の各段の後で Measure/PolyFrame をやり直す。古い弧長の使い回しがトリム位置のずれを生んだ。

4. **症状の観察がそのまま診断になる。** 「進む→反転→曲がる」はベジエのフック形状の走行軌跡そのもの。曲線サンプルのダンプ（x座標が単調でない）で一発確定した。ヘッドレスで heading の急変(>100°)を検出する自動テストは、この種の幾何バグの回帰防止に安い保険。

  

---

  

## 7. パラメータ対照表

  

Web版(px)とHoudini移植時(m)の推奨換算。1px ≒ 0.25m（車長14px ≒ 3.5m基準）。

  

| パラメータ | Web (px) | Houdini (m) | 意味 |

|---|---|---|---|

| LANE_WIDTH | 10 | 3.0 | 車線幅 |

| CAR_LENGTH | 14 | 3.5〜4.5 | 車体長（IDMの車間計算に使用） |

| NODE_RADIUS | 26 | 7〜10 | 交差点半径（エッジのトリム量） |

| NODE_LOOKAHEAD | 50 | 12〜15 | 次エッジ確定 & 前方先読み距離 |

| MERGE_ZONE | 95 | 25〜30 | lane 0 への合流ゾーン |

| STOP_LINE_OFFSET | 8 | 2 | エッジ終端から停止線まで |

| IDM: v0 | 70 px/s | 8〜14 m/s | 希望速度（市街地 30〜50km/h） |

| IDM: T | 1.2 s | 1.0〜1.6 s | 安全車間時間 |

| IDM: s0 | 9 | 2.0 | 停止時最小車間 |

| IDM: aMax | 45 px/s² | 1.5〜2.5 m/s² | 最大加速度 |

| IDM: b | 55 px/s² | 2.0〜3.0 m/s² | 快適減速度 |

| LANE_CHANGE_RATE | 2.8 車線/s | 同値 | 横移動速度（1変更 ≒ 0.36s） |

| OVERTAKE_TRIGGER | 72 | 18〜20 | 追い越し検討を始める前車距離 |

| OVERTAKE_GAIN | 45 | 11〜12 | 車線変更に必要な利得 |

| handle_ratio | 0.45 | 同値 | ベジエハンドル係数 |

| SAMPLE_SPACING | 2 px | 0.05〜0.1 m | 弧長リサンプル間隔 |

  

---

  

## 付録: 要点の一行サマリ

  

- 右左折 = 事前生成した接線連続ベジエコネクタへの**エッジID乗り換え**。走行側に旋回ロジックは存在しない。

- コネクタの前に**必ず道路を交差点半径ぶんトリム**する。これを怠るとフックする。

- 車線 = カーブではなく**横方向スカラー**。lane（意思・即時）と lane_frac（身体・連続）の2層。

- 停止はすべて**「仮想先行車 + IDM」の一元化**。信号も歩行者も合流待ちも「止まっている車」。

- 右左折中の前車停止 = **エッジ境界先読み付きの前車検出 + IDM**。認識は正しい。

- 縮退点（ゼロ長セグメント）は方向情報を破壊する。Fuse/重複除去と帳簿の再計測を怠らない。
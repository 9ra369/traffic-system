# 右折 vs 直進 コンフリクト解決 — 実装仕様（Houdini向け）

上から見た交差点で、右折車が対向直進車に譲る／抜けるを判断するロジック。
モックHTMLで詰めた最終仕様を、Houdini（POP/Solver SOP + VEX）で組める粒度でまとめる。

---

## 1. 前提・データモデル

- **レーン**：カーブ（ポリライン）。交錯する相手レーンについて `{lane_id, cp}` を持つ。`cp` = 2レーンの交点ワールド座標。
- **交点 cp**：静的。右折開始時に1回だけ取得してキャッシュ（形状が変わらなければ再取得不要）。
- **車（ポイント）**：属性として
  - `P`（位置）, `dir`（進行方向, 正規化）, `speed`（スカラー速度）
  - `lane_id`, `turn`（`"straight"` / `"right"`）
  - `dist_on_path`（自レーン上の進行距離。パスに沿って積算）
  - `ahead_intersect`（0/1：自分の cp を通過したか）
  - 右折車のみ：`decided`（`none`/`yield`/`go`）

> モックでは cp は単一の固定点。実際は「交錯する各直線レーンごとに cp」を持ち、車ごとに評価する。

---

## 2. 優先順位

**直進 ＞ 右折。** 右折車が譲る。よって右折車側の判定を厳密にし、直進車側は保険的な減速・停止に留める。

---

## 3. 右折車ロジック（毎フレーム）

### 3.1 検知の開始条件
- **右折を開始してから**だけ検知する（直進の進入区間では検知しない）。
  `turning = (dist_on_path >= turn_start)`（`turn_start` = カーブが始まるパス距離）。
- `afterCp`（自分が cp を通過）になったら検知終了。

### 3.2 候補の絞り込み（各直進車 other について）
順に弾く：
1. `other.ahead_intersect == 1` → 対象外（通過済み）
2. `length(other.P - self.P) > search_radius` → 対象外（近傍外）
3. `dot(self.dir, normalize(other.P - self.P)) < fwd_min` → 対象外（自分の前方にいる車だけ残す）
4. コーン判定：`cone = dot(normalize(other.P - self.P), other.dir)`。
   `coneLo <= cone <= coneHi` の車だけ「コーン内」＝判断対象。

### 3.3 判断（車ごとに初見の瞬間だけTTCを評価・以後は保持）
- `anyCandidate` = 3.2 の絞り込みのうち **1, 2 だけ**（未通過・検索半径内）を満たす車が1台でもいるか。前方判定・コーン判定より緩い条件で、「判断を保持するかどうか」はこれで決める。
- **各車ごとに `seen`（評価済みか）と `blocking`（停止対象か）を持つ**。まだ `seen` でない車が初めてコーン内（前方判定＋コーン判定を満たす）に入った、その瞬間だけ
  `t_other = distance(cp, other.P) / max(other.speed, v_floor)` を計算し、`blocking = (t_other < decideTTC)` を確定して `seen = true` にする。**以後、この車については通過するまで再評価しない**（速度一定なら接近するほど `t_other` は時間経過分だけ単調に減っていくので、毎フレーム再評価すると「既知の車のTTCが閾値を割っただけ」で新規の脅威と誤認し、進んでは止まるを繰り返す）。
- `blocked` = 通過していない車のうち、`seen && blocking` が1台でもいるか。`hasSeen` = 通過していない車のうち `seen` が1台でもいるか。
- 判断ラッチ：
  ```
  if (!turning || afterCp || !anyCandidate)   decided = none;                 // 周囲に対象車が本当にいなくなった→リセット
  else if (decided == none && hasSeen)        decided = blocked ? yield : go; // 初回検知で確定
  else if (decided == go && blocked)          decided = yield;                // 本当に新しく見えた車がblockingなら即昇格
  ```
  - `decided` は前方判定・コーン判定の出入り（自分の向き `dir` がカーブで回転することによる一瞬の判定漏れ）では消さない。`anyCandidate` が偽＝本当に周囲に対象車がいなくなったときだけリセットする。
  - `yield` はこのラッチでは `go` に戻さない（安全側は自動で緩めない）。
- **停止線を越えたら通過へ切替**（交錯点に居座らない）：
  ```
  stop_line = cp_dist_on_path - stopMargin;   // cp 手前の停止線（パス距離）
  if (decided == yield && dist_on_path >= stop_line)  decided = go;
  ```

### 3.4 反応（目標速度と減速度）
`dCp = distance(cp, self.P)`、`creep = cruise * turnCreep`。
| 状態 | 目標速度 | 減速度 |
|---|---|---|
| `!turning`（直進進入） | `cruise` | 加速 `a_accel` |
| `afterCp`（通過後） | `cruise` | 加速 `a_accel` |
| `yield` かつ `dCp < stop_dist` | `0` | 緊急 `a_emergency` |
| `yield` かつ `dCp < decel_dist` | `0.28 * cruise` | 快適 `a_comfort` |
| `yield` かつそれ以上 | `creep` | ― |
| `go` / 未判断 | `creep` | ― |

- **go と決めたら絶対に途中で止まらない**（目標が0になる分岐に入らない）。
- 速度積分：`a = (target >= speed) ? a_accel : brakeA`、`speed += clamp(target-speed, ±a*dt)`。位置は `dist_on_path += speed*dt`、そこからパスをルックアップして `P, dir` を更新。

---

## 4. 直進車ロジック（毎フレーム・保険的減速/停止）

直進車は優先だが、**右折車が「こっちに向かってきているか」だけ**で反応する。
```
if (ego.ahead_intersect == 1)  無視;            // 右折車が通過済み
ego_coming = (ego.speed > 0.5);                  // 動いている＝向かってきている
V   = ego.P - self.P;
fwd = dot(self.dir, normalize(V));               // 右折車が自分の前方にいるか
if (fwd >= strFwdMin && ego_coming) {
    d = length(V);
    if      (d <= strStop)  → 停止（speed→0）
    else if (d <= strDecel) → 減速
}
```
- 停止して譲っている（動いていない）右折車には反応しない → **膠着しない**。
- 右折車が停止線を越えて動いて抜けるときは `ego_coming` が真になり、直進車が止まって受ける。

---

## 5. パラメータ（最終既定値）

| 記号 | 意味 | 既定 |
|---|---|---|
| `decideTTC` | 相手到達時間がこれ未満なら停止(yield) | 3.5 s |
| `coneLo / coneHi` | 検索コーン `dot` 下限/上限 | -1.0 / 0.14 |
| `fwd_min` | 右折車の前方判定 `dot(dir, →相手)` 下限 | 0.5 |
| `search_radius` | 近傍検索半径 | 25 m |
| `stop_dist / decel_dist` | 停止/減速を始める cp までの距離 | 5.5 / 12.0 m |
| `stopMargin` | cp 手前の停止線。越えたら通過へ | 1.5 m |
| `v_floor` | 到達時間計算の最低速度（発散防止） | 1.5 m/s |
| `a_comfort / a_emergency / a_accel` | 快適減速/緊急減速/加速 | 2.5 / 6.0 / 3.0 m/s² |
| `cruise / turnCreep` | 巡航速度 / 右折中の徐速係数 | 6.0 m/s / 0.65 |
| `strFwdMin` | 直進車の前方判定下限 | 0.65 |
| `strDecel / strStop` | 直進車が減速/停止する対右折車距離 | 9 / 5 m |

---

## 6. Houdini 実装メモ

- **構成**：Solver SOP（フレーム間フィードバック）内の Point Wrangle に上記を VEX で記述。車＝ポイント。
- **属性の保持**：`decided`(int), `ahead_intersect`(int), `speed`(float), `dir`(vector), `dist_on_path`(float) は Solver 内で毎フレーム持ち越す。`decided` は enum を int で（0=none,1=yield,2=go）。
- **レーン形状とcp**：レーンカーブ上で `dist_on_path` を弧長として持たせる（`resample` で `curveu`→弧長、または各セグメント長を積算）。cp・turn_start・stop_line は各レーンにあらかじめ `detail`/`prim` 属性で持たせるか、右折開始時に一度だけ計算してポイント属性へキャッシュ。
- **近傍検索**：相手レーンの車だけの point cloud に対し `pcopen`/`pcfilter` か `nearpoints()`。`lane_id` でフィルタ。相手数は数個なので線形探索でも可。
- **ベクトル演算**：`dot()`, `normalize()`, `distance()`, `length()` をそのまま使用。
- **通過判定**：`ahead_intersect` は「進行方向に対して cp を過ぎたか」＝ `dist_on_path > cp_dist_on_path` で更新。
- **速度→移動**：`dist_on_path += speed * @TimeInc`。パス上位置は `primuv`/自前ルックアップで `P` を取得、`dir` は前後サンプル差分を正規化。

### VEX 擬似コード（右折車ポイント）
```c
float creep = cruise * turnCreep;
int   turning = (dist_on_path >= turn_start);
int   afterCp = (dist_on_path >  cp_dist_on_path + 0.5);
i@ahead_intersect = (dist_on_path > cp_dist_on_path);

// --- 検知（右折開始後のみ）---
// other.seen / other.blocking は車（ポイント）側が持つ属性。通過したらリセット不要（もう見ない）、
// 新しいターン開始時（turn_start通過時）にだけ 0 にリセットする。
int hasCandidate = 0;   // 未通過・半径内にいる車が1台でもいるか（判断を"保持"する根拠。コーン/前方判定より緩い）
int hasSeen = 0, blocked = 0;
if (turning && !afterCp) {
    // 相手レーンの車を近傍検索して other ごとに:
    //   if other.ahead_intersect: continue;
    //   vector V = other.P - @P;
    //   if (length(V) > search_radius) continue;
    //   hasCandidate = 1;
    //   int inCone = dot(@dir, normalize(V)) >= fwd_min
    //             && (float cone = dot(normalize(V), other.dir)) >= coneLo && cone <= coneHi;
    //   if (inCone && !other.seen) {                              // 初めてコーン内に入った瞬間だけ評価
    //       other.seen = 1;
    //       float t_other = distance(cp, other.P) / max(other.speed, v_floor);
    //       other.blocking = (t_other < decideTTC) ? 1 : 0;
    //   }
    //   if (other.seen) { hasSeen = 1; if (other.blocking) blocked = 1; }
    //   // ※ other.seen 後は t_other を再計算しても other.blocking は変更しない（保持したまま）
}

// --- 判断ラッチ ---
if (!turning || afterCp || !hasCandidate)        i@decided = 0;                 // none（周囲に対象車が本当にいない）
else if (i@decided == 0 && hasSeen)              i@decided = blocked ? 1 : 2;   // 初回確定 yield:1 / go:2
else if (i@decided == 2 && blocked)              i@decided = 1;                 // 本当に新しく見えた車がblockingなら昇格
float stop_line = cp_dist_on_path - stopMargin;
if (i@decided == 1 && dist_on_path >= stop_line) i@decided = 2;     // 越えたら通過

// --- 反応 ---
float dCp = distance(cp, @P);
float target = cruise, brake = a_comfort;
if      (!turning)          target = cruise;
else if (afterCp)           target = cruise;
else if (i@decided == 1) {                                          // yield
    if      (dCp < stop_dist)  { target = 0;            brake = a_emergency; }
    else if (dCp < decel_dist) { target = 0.28*cruise;  brake = a_comfort;   }
    else                         target = creep;
} else                        target = creep;                       // go / none

// --- 速度積分 & 前進 ---
float a  = (target >= f@speed) ? a_accel : brake;
f@speed += sign(target - f@speed) * min(a * @TimeInc, abs(target - f@speed));
f@speed  = max(0, f@speed);
dist_on_path += f@speed * @TimeInc;   // → パスから @P, @dir を更新
```

---

> 修正の経緯・設計判断の記録は別ファイル `right-turn-changelog.md` を参照。

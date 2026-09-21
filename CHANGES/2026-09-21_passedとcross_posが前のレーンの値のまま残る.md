# 2026-09-21 `passed` / `cross_pos` が前のレーンの値のまま残る

道路形状を作り替えたあと、直進車の `i@passed` がリセットされないという指摘から。

## 原因1：`lane_changed` を下ろすのが早すぎた

`03_UPDATE_LANE_ATTRIB.vfl` の切替直後の再解決は2段になっている。

```c
int pt = nearpoint(1, "@lane_name=" + s@lane_name, @P);
if (pt >= 0) {
    i@lane_id = point(1, "lane_id", pt);
    i@lane_changed = 0;                                   // ← ここで下ろしていた
    int lane_pt = findattribval(1, "point", "lane_id", i@lane_id);
    if (lane_pt >= 0) {
        ...
        i@passed = 0; i@passed_frame_count = 0; v@cross_pos = {0,0,0};   // ← 内側
    }
}
```

`passed` / `cross_pos` / `cross_lines` / ラッチ配列のリセットは**内側**にあるのに、
フラグは**外側**で下りていた。`lane_pt` の解決に失敗すると、リセットは一度も走らないまま
再試行の機会も失われ、前のレーンの値が永久に残る。
`___SPEC.md` が書いているリトライパターン（解決に成功したときだけフラグを下ろす）が、
内側の解決には適用されていなかった。

`findattribval` が失敗するのは、**Road Line側に `lane_id` が存在しないとき**。
つまり道路を作り替えてベイク／属性付与をやり直していない場合で、今回の状況と一致する。

### 変更

`i@lane_changed = 0;` を内側ブロックの末尾へ移動。

検証（hython, Houdini 20.5。`passed=1` / `cross_pos={99,0,99}` を持つ車を投入）:

| Road Lineの状態 | 結果 |
|---|---|
| `lane_id` あり | `lane_changed=0` / `passed=0` / `cross_pos` 再取得 ＝ 従来どおり |
| `lane_id` 無し | `lane_changed=1` のまま ＝ **次フレームで再試行**（修正前はここで下りて固着していた） |

## 原因2（未修正・要確認）：`cross_pos` が未ベイクだと `passed` は必ず1になる

`07_RESOLVE_CONFLICT.vfl` の直進車ブロックは交錯点との前後関係で `passed` を決める。

```c
vector to_cross = v@cross_pos - @P;
float  d = dot(normalize(v@dir), normalize(to_cross));
if (d <= 0) { ... 48フレームで i@passed = 1; }
```

`cross_pos` が `{0,0,0}`（＝13/12のベイクが無い・古い）のとき、`to_cross` は
**ワールド原点へ向くベクトル**になる。原点から遠ざかる向きに走っている車では `d <= 0` が
成立し続けるので、48フレームで全車が `passed = 1` に落ちる。

`passed` は現在デバッグ用で、右折側の候補探索では読んでいない（07のM5-1以降）。
つまり `passed` の固着そのものは `yield_conflict` を壊さないが、
**`cross_pos` が焼けていないことの指標**にはなる。`DEBUG_BAKE_STATE.vfl` と併せて確認する。

## 原因3（未修正・要確認）：`s@turn` が "straight" でないと `passed` は一切更新されない

`if (s@turn == "straight" && i@passed == 0)` が入口なので、直進レーンの点に
`turn` が付いていない（空文字）場合、このブロックは一度も走らない。
`passed` が初期値のまま動かないときはこちらを疑う。原因2とは症状が逆（1に固着 vs 0のまま）。

## 追記：`n_lane > 0` かつ `n_alive == 0` を確認 → 候補は `t_cp` の除外行で全滅していた

```c
// 07_RESOLVE_CONFLICT.vfl 候補ループ
if (dot(normalize(opp_dir), t_cp - opp_p) < -opp_clear) continue;
```

`t_cp` が `{0,0,0}` だと `t_cp - opp_p` は**ワールド原点へ向くベクトル**になり、長さは
原点までの距離になる。原点から遠ざかる向きに走る対向車は内積が大きな負値になって
`-opp_clear(5.0)` を下回り、**全員がここで落ちる**。原点へ向かう車は残るが、
`o_dist` が原点までの距離になるので `t_other` が巨大になり `blocked` が立たない。
どちらにせよ `yield_conflict` は立たない。

### 未ベイクを黙って通さないようにした

`cp_valid = (dCp > 1e-4)` は「自車から `t_cp` までの距離」なので、`t_cp` が原点でも
自車が原点から離れていれば成立してしまい、**未ベイクを検出できていなかった**。
座標ではなく「属性が焼かれているか」で判定する。

- `03`: `cross_lines` があるのに `cross_line_pos` の本数が合わなければ `i@cp_baked = 0`
  （属性が無いと `prim()` は空配列を返すだけでエラーは出ない）
- `07`: `cp_valid` に `i@cp_baked` を併せる。さらに、未ベイクだと候補が全滅して
  `t_decided` が1にならず `go_reason` に何も出ないため、判断に入る前でも `go_reason=4` を出す
- `01`: 既定値は1（焼けている前提。03が否定したときだけ0になる）

検証（hython, Houdini 20.5）:

| Road Lineの `cross_line_pos` | 結果 |
|---|---|
| あり | `cp_baked=1` / `go_reason=0` / `rt_target_cp=(120,0,0)` |
| 無し | `cp_baked=0` / `go_reason=4` / `rt_target_cp=(0,0,0)` |

これは検出であって修復ではない。`cp_baked=0` が出ている間は、13/14 のベイクを
今の道路形状で焼き直す必要がある。

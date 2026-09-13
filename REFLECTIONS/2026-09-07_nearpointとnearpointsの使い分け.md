# nearpoint と nearpoints の使い分け（2026-09-07）

[[04_SIGNAL DETECT]]・[[05_HUMAN DETECT]]・[[06_CAR DETECT]] を見比べながら、`nearpoints`（半径内を全部列挙してVEXでループ）を`nearpoint`（属性一致グループの最寄り1点を検索構造側で探す）に置き換えられないか検討した記録。04と05は置き換え、06は置き換えなかった。その判断基準をまとめる。

## 2つの違い

| | `nearpoints(geo, pos, radius)` | `nearpoint(geo, group, pos)` |
|---|---|---|
| 返す値 | 半径内の点**全部**を配列で返す | 条件に合う点のうち**最寄りの1点**だけ |
| 絞り込みの場所 | 配列を受け取った後、VEX側の`foreach`で1点ずつ`point()`で属性を読んで`continue`で捨てる | 検索構造（内部的な空間分割）の中で条件フィルタも一緒にやって、最初から絞られた中で最近傍を探す |
| コストの決まり方 | 半径内にいる点の数に比例して増える。ループ内の属性読み出し・分岐もその数だけ発生 | 探索自体は最近傍探索なので、「配列化してVEXでループする」オーバーヘッドがない分、同じ結果を得るなら基本的に軽い |

`nearpoint`の第2引数（グループ）には、通常の名前付きグループだけでなく `"@road_id=" + itoa(id)` のような属性ベースのパターン文字列も渡せる。さらに `&&` で複数条件をAND結合できる（例: `"@road_name=" + s@road_name + " && @ped_crossing=1"`）。これは[[03_UPDATE_LANE_ATTRIB]]や[[02_CHANGE ROAD]]の`nearpoint(1, "@road_id=" + itoa(i@road_id), @P)`と同じ仕組み。

## 置き換えた：04_SIGNAL DETECT / 05_HUMAN DETECT

条件がすべて「相手側（Input 2 / Input 3）が持つ静的な属性の一致」だけで表現できた：
- SIGNAL DETECT: `road_name`が自分と一致する信号（1レーンにつき信号は1個の前提）
- HUMAN DETECT: `road_name`が自分と一致 **かつ** `ped_crossing==1`（横断中）の歩行者

自車側の状態に依存する条件（前方判定`dot(dir, →相手)`、検出半径`search_radius`）だけは`nearpoint`のグループ指定に埋め込めないので、1点見つけた後に別途チェックする形にした。

```c
int ptnum = nearpoint(3, "@road_name=" + s@road_name + " && @ped_crossing=1", @P);
if (ptnum >= 0) {
    // ここで検出半径・前方判定(dot)をチェックしてから採用
}
```

## 置き換えなかった：06_CAR DETECT

**理由1: 自己参照になっている**
Input 0 はこの車自身を含む全車の点群。`nearpoint`でグループ検索すると、自分自身が距離0で必ず最優先ヒットしてしまう。元コードの `if (pt == @ptnum) continue;`（自己除外）は、点番号という「その都度変わる自分のインデックス」に対する除外であり、静的な `"@属性=値"` のグループパターンでは表現できない。

**理由2: 判定が「2種類の別ルール」のORになっている**
- 同レーン先行車：`fwd_dot >= 0.5`
- 交差/右左折車：`fwd_dot >= 0.7` **かつ** 横オフセット `<= 2m`

というまったく別の条件のどちらかに当てはまる車の中から最寄りを探している。`nearpoint`は「1つの属性条件に一致する最寄り点」しか返せないので、この「OR＋それぞれ別の追加チェック」はそのまま渡せない。

## 判断基準（まとめ）

- 条件が全部、**相手側の静的な属性の一致・比較だけ**で表現できて、欲しいのは**最寄りの1点だけ** → `nearpoint` + グループ指定が使える・速い
- **自己除外が要る**／**複数の異なるルールのOR**を取る／見つけた後でないと判定できない動的な条件が複数絡む → `nearpoints` で候補を出してVEX側でループするしかない

## 関連ファイル
- [[04_SIGNAL DETECT]] — `nearpoints`→`nearpoint`に変更済み
- [[05_HUMAN DETECT]] — 同上（road_name＋ped_crossingの複合条件）
- [[06_CAR DETECT]] — 自己参照＋ORルールのため`nearpoints`のまま
- [[03_UPDATE_LANE_ATTRIB]] / [[02_CHANGE ROAD]] — 同じ`nearpoint`＋属性パターンの先行例（`road_id`）

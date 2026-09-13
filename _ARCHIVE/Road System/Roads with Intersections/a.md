//後に、0から1にsortするためにここでは、とりあえず0と1を入れておく

f[]@coords = array(0,1);

  

for(int i; i < npoints(i); i++) {

    //sourceprim: the number of primitives incident to the intersection point

    //ポイント i がどのプリミティブ由来かを配列で取得。

    int src_prim [] = point(1, 'sourceprim', i);

    //ポイント i のソースプリミティブ上のUV座標を配列で取得。

    vector src_uv [] = point(1, 'sourceprimuv', i);

    //src_prim 配列の中から、現在のプリミティブ番号 が何番目にあるか検索。

    int prim_id = find(src_prim, @primnum);

    // if no id, skip this process

    if (prim_id < 0) continue;

    //このプリミティブ上に落ちているポイントのU座標を coords に追加

    append(f[]@coords, src_uv[prim_id].x)

}

  

f[]@coords = sort(f[]@coords);
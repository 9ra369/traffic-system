vector center = point(1, 'P', 0);
vector dir = normalize(@P - center);
//houdiniの座標系を考慮し、2次元に落とし込むとy=dir.x, x = dir.z
//VEXを含め多くの言語で引数の順番は `atan2(y, x)`（yが先）
f@__atan = atan2(dir.x, dir.z);
if (inpointgroup(0, "intersect", @ptnum) == 0) return;

int prims[] = pointprims(0, @ptnum);

foreach (int prim; prims) {
    setprimattrib(0, "__id", prim, @ptnum);
}
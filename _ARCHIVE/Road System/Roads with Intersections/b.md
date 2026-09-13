function int prim_from_array (int primnum; float coords[]) {

    int prim = addprim(0, 'polyline');

    vector pos;

    int pt;

  

    foreach(int i; float coord; coords) {

        pos = primuv(0, 'P', primnum, coord);

        pt = addpoint(0, pos);

        addvertex(0, prim, pt);

    }

  

    return prim;

}

  

float coords[] = f[]@coords;

  

for (int i = 0; i < (len(coords)-1); i++) {

    int iters = chi('iters');

    float v0 = coords[i];

    float v1 = coords[i+1];

  

    float new_list[] = resample_linear(array(v0, v1), iters);

    prim_from_array(@primnum, new_list);

}

  

removeprim(0, @primnum, 1);
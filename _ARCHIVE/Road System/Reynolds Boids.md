**Repulsion**
int handle = pcopen(0, "P", @P, chf("radius"), chi("num_points"));
int neighbour_ptnum;
vector neighbour_pos;

if(pcnumfound(handle)>0)
{
    while(pciterate(handle))
    {
        pcimport(handle, "point.number", neighbour_ptnum);
        //if the current point is not myself...
        if (@ptnum != neighbour_ptnum)
        {
            pcimport(handle, "P", neighbour_pos);

            //make vector pointing away from neighbour
            vector direction = normalize(@P - neighbour_pos);
            //make distance ramp, closer points get higher mult
            float dist = chramp("distance_ramp", fit(distance(@P, neighbour_pos), 0, chf("radius"), 0, 1));
            v@force += direction * dist * chf("repulsion_strength");
        }
    }
}


**Alignment**
int handle = pcopen(0, "P", @P, chf("radius"), chi("num_points"));

if(pcnumfound(handle)>0)
{
    //get average v of found points
    vector average_v = pcfilter(handle, "v");
    //make me go in this average v
    vector direction = normalize(average_v - @v);
    v@force += direction * chf("alignment_strength");
}

**Cohesion**
int handle = pcopen(0, "P", @P, chf("radius"), chi("num_points"));

if(pcnumfound(handle)>0)
{
    //get average position of found points
    vector average_pos = pcfilter(handle, "P");
    //make a vector that points to this average position
    vector direction = normalize(average_pos - @P);
    //add force into this direction
    float distance = chramp("distance_ramp", fit(distance(@P, average_pos), 0, chf("radius"), 0, 1));
    v@force += direction * distance * chf("cohesion_strength");
}

**Stay away from bounds**
int handle = pcopen(1, "P", @P, chf("radius"), chi("num_points"));

if(pcnumfound(handle)>0)
{
    vector close_point_pos = pcfilter(handle, "P");
    vector direction = @P - close_point_pos;
    float distance = fit(distance(@P, close_point_pos), 0, chf("radius"), 1, 0);
    v@force = normalize(direction) * distance * chf("push_strength");
}

**Color**
int handle = pcopen(1, "P", @P, chf("radius"), chi("num_points"));

if(pcnumfound(handle)>0)
{
    vector close_point_pos = pcfilter(handle, "P");
    vector direction = @P - close_point_pos;
    float distance = fit(distance(@P, close_point_pos), 0, chf("radius"), 1, 0);
    v@force = normalize(direction) * distance * chf("push_strength");
}
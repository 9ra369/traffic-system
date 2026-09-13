// xyzdistを使って、自分と同じroad nameを持っているスプラインを見つけ
// その中におけるパラメトリック座標で、末端に近づいたら、次のスプラインへ

int hitprim;
vector uvw;
float d = xyzdist(1, s@lane_name, @P, hitprim, uvw);

if (hitprim >= 0) {          // 見つかったか確認
    @u = uvw.x;
    if (uvw.x >= 0.99) {
        s@lane_name = s@next_lane_name;
        i@lane_changed = 1;
    }
}
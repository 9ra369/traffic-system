vector colors[] = {{0,1,0}, {1,1,0}, {1,0,0}};
int s = i@car_state;

if (s == 0 && i@car_ptnum >= 0 && f@vel < 0.1) {
    // IDM中で停止 → 赤
    @Cd = colors[2];
} else if (s == 0 && i@car_ptnum >= 0 && f@accel < 0) {
    // IDM中で減速 → 黄色
    @Cd = colors[1];
} else {
    @Cd = (s >= 0 && s < len(colors)) ? colors[s] : {1,1,1};
}
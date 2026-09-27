#!/usr/bin/env bash
# compare_ab_fastlio.sh —— 离线回放 A/B 对比: 同一 bag 分别用 雷达内置IMU / CJ02 跑 FAST-LIO
#
# 用法:  ./compare_ab_fastlio.sh <bag文件> [输出目录]
# 前提:  外参已标定并 --apply 写入 mid360_cj02.yaml
set -u
set +u; source /opt/ros/noetic/setup.bash; set -u
source /mnt/nvme/ws/cj02_ws/devel/setup.bash   # overlay, 已含 fastlio_ws

BAG=${1:?用法: ./compare_ab_fastlio.sh <bag> [outdir]}
OUT=${2:-/mnt/nvme/fastlio_bags/ab_result_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$OUT"

FASTLIO_WS_LAUNCH=/mnt/nvme/ws/fastlio_ws/src/FAST_LIO/launch

# roscore
if ! pgrep -x rosmaster >/dev/null; then
  nohup roscore > /tmp/roscore_ab.log 2>&1 &
  for i in $(seq 1 20); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
fi

run_pass () {  # $1=名称 $2=launch文件
  local NAME=$1; local LAUNCH=$2
  echo ""
  echo "===== [$NAME] 回放并运行 FAST-LIO ====="
  rosparam set use_sim_time true
  nohup roslaunch "$LAUNCH" > "$OUT/$NAME.log" 2>&1 &
  for i in $(seq 1 20); do
    rostopic list 2>/dev/null | grep -q '^/drone_0/Odometry$' && break; sleep 1
  done
  nohup rostopic echo -p /drone_0/Odometry > "$OUT/$NAME.csv" 2>/dev/null &
  local EPID=$!
  rosbag play --clock --delay 3 "$BAG"
  sleep 3
  kill $EPID 2>/dev/null
  pkill -f 'fastlio_mappin[g]' 2>/dev/null
  sleep 2
  rosparam set use_sim_time false
  echo "[$NAME] 完成 -> $OUT/$NAME.csv ($(wc -l < "$OUT/$NAME.csv") 行)"
}

run_pass builtin "$FASTLIO_WS_LAUNCH/mapping_mid360.launch"
run_pass cj02    "$FASTLIO_WS_LAUNCH/mapping_mid360_cj02.launch"

echo ""
echo "===== 对比结果 ====="
python3 - "$OUT" <<'PYEOF'
import csv, math, sys, os
out = sys.argv[1]
def load(name):
    path = os.path.join(out, name + '.csv')
    if not os.path.exists(path): return None
    pts, t0 = [], None
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                def g(k):
                    return row.get(k, row.get('field.' + k, ''))
                t = float(g('%time')) / 1e9
                p = (float(g('pose.pose.position.x')),
                     float(g('pose.pose.position.y')),
                     float(g('pose.pose.position.z')))
            except (KeyError, ValueError, TypeError):
                continue
            if t0 is None: t0 = t
            pts.append((t - t0, p))
    return pts
res = {}
for name in ('builtin', 'cj02'):
    res[name] = load(name)
    if not res[name]:
        print(f"  {name:8s}: CSV 为空/不存在")
def summary(pts):
    n = len(pts)
    if n < 10: return None
    dur = pts[-1][0] - pts[0][0]
    length = sum(math.dist(pts[i-1][1], pts[i][1]) for i in range(1, n))
    s = pts[0][1]; e = pts[-1][1]
    end = e
    ret = math.dist(s, e)              # 回到起点的闭环误差
    zmin = min(p[2] for _, p in pts); zmax = max(p[2] for _, p in pts)
    return dict(n=n, dur=dur, length=length, end=end, ret=ret, dz=zmax - zmin)
S = {}
for k, v in res.items():
    S[k] = summary(v) if v else None
if S.get('builtin') and S.get('cj02'):
    print(f"  {'指标':<22s}{'内置IMU':>16s}{'CJ02':>16s}")
    b, c = S['builtin'], S['cj02']
    print(f"  {'样本数':<22s}{b['n']:>16d}{c['n']:>16d}")
    print(f"  {'时长(s)':<22s}{b['dur']:>16.1f}{c['dur']:>16.1f}")
    print(f"  {'轨迹长度(m)':<24s}{b['length']:>14.1f}{c['length']:>14.1f}")
    print(f"  {'终点(x,y,z)':<26s}{str(tuple(round(x,3) for x in b['end'])):>24s}{str(tuple(round(x,3) for x in c['end'])):>24s}")
    print(f"  {'闭环误差(m)':<24s}{b['ret']:>14.3f}{c['ret']:>14.3f}")
    print(f"  {'z 变化(m)':<24s}{b['dz']:>14.3f}{c['dz']:>14.3f}")
    print()
    if c['ret'] < b['ret'] * 1.3:
        print("  [结论] CJ02 版闭环误差与内置IMU同量级, 可进入手持/飞行验证")
    else:
        print("  [结论] CJ02 版闭环误差明显偏大 → 检查: 外参是否已标定写入 / 时间同步 / acc_cov")
else:
    print("  (有一路 CSV 缺失, 检查对应 .log)")
PYEOF
echo ""
echo "详细结果: $OUT  (含两路 CSV 轨迹与运行日志)"

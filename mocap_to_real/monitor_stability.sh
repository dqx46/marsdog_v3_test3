#!/bin/bash
# 持续监控 USB 总线稳定性: 记录 dmesg 里的 disconnect 事件 + 定期跑 static_test.py
# 用法: ./monitor_stability.sh <持续时间秒数>
set -u
DURATION="${1:-600}"
OUTDIR="/home/cat/公共的/20260705_1520/mocap_to_real"
LOG="$OUTDIR/stability_log_$(date +%H%M%S).txt"
cd "$OUTDIR" || exit 1

echo "=== 稳定性监控开始 $(date) 持续 ${DURATION}s ===" | tee -a "$LOG"
START=$(date +%s)
LAST_DMESG_COUNT=$(dmesg | grep -c "USB disconnect")

while true; do
    NOW=$(date +%s)
    ELAPSED=$((NOW - START))
    if [ "$ELAPSED" -ge "$DURATION" ]; then
        break
    fi

    CUR_COUNT=$(dmesg | grep -c "USB disconnect")
    if [ "$CUR_COUNT" -ne "$LAST_DMESG_COUNT" ]; then
        echo "[$(date +%H:%M:%S)] *** 检测到新的 USB disconnect 事件 (总计 $CUR_COUNT 次) ***" | tee -a "$LOG"
        dmesg -T | grep "USB disconnect" | tail -5 | tee -a "$LOG"
        LAST_DMESG_COUNT=$CUR_COUNT
    fi

    echo "[$(date +%H:%M:%S)] --- static_test 快速检查 ---" >> "$LOG"
    timeout 15 python3 static_test.py 2>&1 | grep -E "电机在线|IMU:|离线" >> "$LOG"

    sleep 20
done

echo "=== 监控结束 $(date) ===" | tee -a "$LOG"
echo "最终 disconnect 计数: $(dmesg | grep -c 'USB disconnect')" | tee -a "$LOG"

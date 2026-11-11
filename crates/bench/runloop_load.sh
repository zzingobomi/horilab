#!/bin/sh
# 실행 루프 부하 시험을 부하를 건 채 한 판 돌린다 (docs/decisions/0026-pi-runtime-deploy-records.md의
# 미리 재기 2). 파이에서, runloop 실행 파일과 첫 연결 통신 시간 기록(timing-hori3.csv) 옆에서 돌린다.
#
#     sh runloop_load.sh <iperf3 상대 주소> <이름> <시간> [runloop 옵션...]
#     예: sh runloop_load.sh 192.168.0.101 fifo80-load-3h 3h --fifo 80 --mlock
#
# - 부하: stress-ng(CPU 4코어, 메모리 256 MB, SD에 fsync 쓰기, sync)와 iperf3(상대 파이에 보내기와 받기
#   각 200 Mbit/s). 상대 파이에 서버 둘을 미리 띄운다: iperf3 -s -D -p 5201; iperf3 -s -D -p 5202
#   (iperf3 3.9의 --bidir은 -b를 주면 거의 보내지 않아서 둘로 나눈다.)
# - stress-ng는 스크립트에서 띄우면 SIGTERM으로 끝나지 않아서, 부하 시간을 판 길이에 맞추고 끝나면
#   SIGKILL로 끈다.
# - 실시간 우선순위와 메모리 잠금 때문에 runloop은 sudo로 돈다. 끝나면 기록의 주인을 돌려놓는다.
set -eu
[ $# -ge 3 ] || { sed -n '5,6p' "$0"; exit 2; }
peer=$1; name=$2; dur=$3; shift 3
cd "$(dirname "$0")"
case "$dur" in
  *h) seconds=$(( ${dur%h} * 3600 )) ;;
  *m) seconds=$(( ${dur%m} * 60 )) ;;
  *s) seconds=${dur%s} ;;
  *) seconds=$dur ;;
esac
load_s=$(( seconds + 60 ))
load="stress-ng --cpu 4 --vm 1 --vm-bytes 256M --hdd 1 --hdd-bytes 16M --hdd-opts fsync --io 1; iperf3로 $peer에 보내기와 받기 각 200 Mbit/s"
mkdir -p tmp
stress-ng --cpu 4 --vm 1 --vm-bytes 256M --hdd 1 --hdd-bytes 16M --hdd-opts fsync --io 1 \
  --temp-path "$PWD/tmp" --timeout "${load_s}s" >/dev/null 2>&1 &
iperf3 -c "$peer" -p 5201 -t "$load_s" -b 200M >/dev/null 2>&1 &
iperf3 -c "$peer" -p 5202 -t "$load_s" -b 200M -R >/dev/null 2>&1 &
sleep 15
echo "부하 켬 $(date '+%F %T'), 스로틀 $(vcgencmd get_throttled 2>/dev/null || echo 모름)"
status=0
sudo -n ./runloop --duration "$dur" --bus-csv timing-hori3.csv --name "$name" --note "부하: $load" "$@" || status=$?
echo "끝 $(date '+%F %T'), 스로틀 $(vcgencmd get_throttled 2>/dev/null || echo 모름)"
pkill -9 -x 'stress-ng.*' || true
pkill -x iperf3 || true
sudo -n chown -R "$(id -u):$(id -g)" records
rm -rf tmp
exit "$status"

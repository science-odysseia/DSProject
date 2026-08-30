#!/usr/bin/env bash
# v1_3 통합 시연 실행기 — 모드 3가지뿐이다.
#
#   ./run_v13.sh floor1     아래층만 (T 분기 정찰)
#   ./run_v13.sh floor2     윗층만   (결함 2곳 용접 수리)
#   ./run_v13.sh both       두 층 동시   ← 기본값
#
# 뒤에 붙이는 인자는 그대로 넘어간다:
#   ./run_v13.sh floor2 --headless        창 없이
#   ./run_v13.sh both --nocam             카메라 끄고(가볍게)
#   ./run_v13.sh floor2 --steps 50000     스텝 수 지정
#   ./run_v13.sh floor2 --glass2          floor2 배관 유리(v1_2 --glass 와 같음)
#   ./run_v13.sh floor2 --detect          검출 창(센터링·관경 링·결함) 띄우기
#
# 🚨 배관 유리·검증 레시피·ROS 도메인은 전부 내장이다 — 따로 줄 것이 없다.
set -e

COURSE="${1:-both}"
shift || true

# ── Isaac Sim 설치 자리 ────────────────────────────────────────────
# 🔑 **설치 경로를 못박지 않는다.** 사람마다 자리가 다르므로 순서로 찾는다:
#      ① 환경변수 ISAAC_PATH — 명시가 항상 이긴다
#           ISAAC_PATH=/my/isaacsim ./run_v13.sh floor2
#      ② 흔한 설치 자리를 훑어 python.sh 가 있는 첫 곳
#    찾은 자리는 시작할 때 한 줄로 찍는다.
if [ -z "${ISAAC_PATH:-}" ]; then
  for _c in "$HOME/isaacsim" \
            "$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release" \
            "$HOME/.local/share/ov/pkg"/isaac*sim*/ \
            "/isaac-sim"; do
    [ -x "$_c/python.sh" ] && { ISAAC_PATH="${_c%/}"; break; }
  done
fi
if [ ! -x "${ISAAC_PATH:-}/python.sh" ]; then
  echo "[중단] Isaac Sim 을 못 찾았다. 설치 자리를 알려줄 것:" >&2
  echo "       ISAAC_PATH=/경로/isaacsim ./run_v13.sh $COURSE" >&2
  exit 1
fi
ISAAC="$ISAAC_PATH"
HUMBLE="$ISAAC/exts/isaacsim.ros2.bridge/humble"
echo "[준비] Isaac Sim: $ISAAC"

# Isaac(3.11) 내장 rclpy — ROS 발행에 필요. 런처 환경변수로 줘야 로드된다.
export LD_LIBRARY_PATH="$HUMBLE/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$HUMBLE/rclpy:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-143}"      # 팀 규격서 값
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1
export DISPLAY="${DISPLAY:-:1}"

cd "$(dirname "$0")"

# 🎯 `--detect` 면 검출 뷰어를 **같이** 띄운다.
#    🚨 뷰어는 반드시 **시스템 python3** — Isaac 내장 cv2 는 headless
#       빌드라 imshow 가 없다(tkinter·PyQt5 도 없음, 실측).
case " $* " in
  *" --detect "*)
    rm -f /dev/shm/cobot3_detect.jpg
    python3 tools/detect_view.py &
    VIEWER=$!
    trap 'kill $VIEWER 2>/dev/null' EXIT
    ;;
esac

"$ISAAC/python.sh" real_map_demo_v1_3.py \
     --course "$COURSE" --hold --steps 220000 "$@"

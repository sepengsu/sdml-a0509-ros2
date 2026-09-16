#!/usr/bin/env bash
# Docker 컨테이너 안에서 사용: /ros2_ws 에서 colcon 빌드 (볼륨 캐시 문제 완화)
# 사용 예:
#   ./scripts/ros2_colcon_docker.sh
#   ./scripts/ros2_colcon_docker.sh --pkgs sdml_a0509_camera dh_ag145_description
#   ./scripts/ros2_colcon_docker.sh --clean-pkgs dh_ag145_description
#   ./scripts/ros2_colcon_docker.sh --clean-all
set -euo pipefail

WS="${COLCON_WS:-/ros2_ws}"
cd "${WS}"

CLEAN_ALL=false
CLEAN_PKGS=()
PKGS=()
EXTRA_COLCON=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --clean-all)
      CLEAN_ALL=true
      shift
      ;;
    --clean-pkgs)
      shift
      while [[ $# -gt 0 && "$1" != --* ]]; do
        CLEAN_PKGS+=("$1")
        shift
      done
      ;;
    --pkgs|--packages-select)
      shift
      while [[ $# -gt 0 && "$1" != --* ]]; do
        PKGS+=("$1")
        shift
      done
      ;;
    --)
      shift
      EXTRA_COLCON+=("$@")
      break
      ;;
    -h|--help)
      echo "Usage: $0 [--clean-all | --clean-pkgs PKG ...] [--pkgs PKG ...] [-- COLCON_ARGS...]"
      exit 0
      ;;
    *)
      EXTRA_COLCON+=("$1")
      shift
      ;;
  esac
done

if "${CLEAN_ALL}"; then
  echo "[ros2_colcon_docker] Removing build/, install/, log/"
  rm -rf build install log
fi

for p in "${CLEAN_PKGS[@]}"; do
  echo "[ros2_colcon_docker] Removing build/install for: ${p}"
  rm -rf "build/${p}" "install/${p}"
done

CMD=(colcon build --symlink-install)
if [[ ${#PKGS[@]} -gt 0 ]]; then
  CMD+=(--packages-select "${PKGS[@]}")
fi

# 설치 규칙(urdf/launch)만 바꿔도 CMake가 가끔 갱신을 안 할 때가 있어 캐시 무효화 옵션
if [[ "${COLCON_CMAKE_CLEAN_CACHE:-}" == "1" ]]; then
  CMD+=(--cmake-clean-cache)
fi

if [[ ${#EXTRA_COLCON[@]} -gt 0 ]]; then
  CMD+=("${EXTRA_COLCON[@]}")
fi

echo "[ros2_colcon_docker] Running: ${CMD[*]}"
"${CMD[@]}"

echo ""
echo "[ros2_colcon_docker] 완료. 새 셸에서는 반드시:"
echo "  source ${WS}/install/setup.bash"
echo "이미 연 터미널에서도 빌드 후 위 명령을 다시 실행해야 패키지 변경이 반영됩니다."
echo "stubborn 캐시가 있으면: COLCON_CMAKE_CLEAN_CACHE=1 $0 --pkgs <패키지이름>"

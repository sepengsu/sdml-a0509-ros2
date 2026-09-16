#!/usr/bin/env bash
# doosan-robot2 서브모듈만: export_doosan_changes.sh로 만든 tar.gz 를 src/doosan-robot2에 풀어 덮어쓰기
# Git Bash에서 repo 루트/다른 디렉터리 어디서 실행해도 archive 상대경로가 깨지지 않게 처리한다.
set -euo pipefail

_tar_path_for_gnu_tar() {
  if command -v cygpath >/dev/null 2>&1; then
    cygpath -u "$1"
  else
    printf '%s\n' "$1"
  fi
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd -P)"
DOOSAN_DIR="$REPO_ROOT/src/doosan-robot2"
CALL_DIR="$PWD"
ARCHIVE_INPUT="${1:-doosan_changes_export.tar.gz}"

if [[ ! -d "$DOOSAN_DIR" ]]; then
  echo "오류: src/doosan-robot2 없음."
  exit 1
fi

ARCHIVE="$(_tar_path_for_gnu_tar "$ARCHIVE_INPUT")"
if [[ "$ARCHIVE" != /* ]]; then
  if [[ -f "$CALL_DIR/$ARCHIVE" ]]; then
    ARCHIVE="$CALL_DIR/$ARCHIVE"
  elif [[ -f "$REPO_ROOT/$ARCHIVE" ]]; then
    ARCHIVE="$REPO_ROOT/$ARCHIVE"
  else
    ARCHIVE="$CALL_DIR/$ARCHIVE"
  fi
fi

if [[ ! -f "$ARCHIVE" ]]; then
  echo "파일 없음: $ARCHIVE"
  echo "사용법: $0 [doosan_changes_export.tar.gz 경로]"
  exit 1
fi

ARCHIVE_DIR="$(cd "$(dirname "$ARCHIVE")" && pwd -P)"
ARCHIVE="$ARCHIVE_DIR/$(basename "$ARCHIVE")"

cd "$DOOSAN_DIR"
tar -xzf "$(_tar_path_for_gnu_tar "$ARCHIVE")"
echo "doosan-robot2 적용 완료: $ARCHIVE"

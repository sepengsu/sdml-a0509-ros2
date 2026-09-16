#!/usr/bin/env bash
# doosan-robot2 서브모듈만: Git과 비교해서 바뀐 파일만 tar.gz 으로 묶기 (zip 대신 tar, Git Bash에서도 동작)
set -e

# Git Bash/MSYS: GNU tar 이 "C:/..." 를 원격 호스트(C:)로 해석해 실패함 → /c/... 로 변환
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

if [[ ! -d "$DOOSAN_DIR" ]]; then
  echo "오류: src/doosan-robot2 없음."
  exit 1
fi

cd "$DOOSAN_DIR"
OUT_ARCHIVE="${1:-doosan_changes_export.tar.gz}"
if [[ "$OUT_ARCHIVE" != /* ]]; then
  OUT_ARCHIVE="$REPO_ROOT/$OUT_ARCHIVE"
fi

LIST=$(mktemp)
{
  git diff --name-only HEAD
  git diff --cached --name-only
  git ls-files --others --exclude-standard
} | sort -u > "$LIST"

COUNT=$(wc -l < "$LIST")
if [[ "$COUNT" -eq 0 ]]; then
  echo "doosan-robot2 변경된 파일이 없습니다."
  rm -f "$LIST"
  exit 0
fi

tar -czf "$(_tar_path_for_gnu_tar "$OUT_ARCHIVE")" -T "$LIST"
rm -f "$LIST"

echo "저장 완료: $OUT_ARCHIVE (doosan-robot2 파일 $COUNT 개)"
echo "복원: bash \"$SCRIPT_DIR/apply_doosan_changes.sh\" \"$OUT_ARCHIVE\""

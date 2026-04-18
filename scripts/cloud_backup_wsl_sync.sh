#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "用法: bash scripts/cloud_backup_wsl_sync.sh <项目目录> <云端备份目录> [WSL同步目录]"
  exit 1
fi

SOURCE_DIR="$(realpath "$1")"
CLOUD_BACKUP_DIR="$(realpath -m "$2")"
WSL_SYNC_DIR="${3:-}"

if [[ ! -d "$SOURCE_DIR" ]]; then
  echo "错误: 项目目录不存在: $SOURCE_DIR"
  exit 1
fi

mkdir -p "$CLOUD_BACKUP_DIR"

PROJECT_NAME="$(basename "$SOURCE_DIR")"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
ARCHIVE_PATH="$CLOUD_BACKUP_DIR/${PROJECT_NAME}_${TIMESTAMP}.tar.gz"
BACKUP_EXCLUDE_FILE="${BACKUP_EXCLUDE_FILE:-}"
SYNC_EXCLUDES="${SYNC_EXCLUDES:-.git/}"

if [[ -n "$BACKUP_EXCLUDE_FILE" ]]; then
  if [[ ! -f "$BACKUP_EXCLUDE_FILE" ]]; then
    echo "错误: BACKUP_EXCLUDE_FILE 不存在: $BACKUP_EXCLUDE_FILE"
    exit 1
  fi
  tar --exclude-from "$BACKUP_EXCLUDE_FILE" -czf "$ARCHIVE_PATH" -C "$SOURCE_DIR" .
else
  tar -czf "$ARCHIVE_PATH" -C "$SOURCE_DIR" .
fi
echo "已完成云端备份: $ARCHIVE_PATH"

if [[ -n "$WSL_SYNC_DIR" ]]; then
  WSL_SYNC_DIR="$(realpath -m "$WSL_SYNC_DIR")"
  mkdir -p "$WSL_SYNC_DIR"
  RSYNC_EXCLUDE_ARGS=()
  IFS=',' read -r -a EXCLUDE_PATTERNS <<< "$SYNC_EXCLUDES"
  for pattern in "${EXCLUDE_PATTERNS[@]}"; do
    # 去除每个排除项前后空白，兼容 "a, b ,c" 这类输入
    pattern="${pattern#"${pattern%%[![:space:]]*}"}"
    pattern="${pattern%"${pattern##*[![:space:]]}"}"
    if [[ -n "$pattern" ]]; then
      RSYNC_EXCLUDE_ARGS+=("--exclude" "$pattern")
    fi
  done
  if [[ ${#RSYNC_EXCLUDE_ARGS[@]} -gt 0 ]]; then
    rsync -a "${RSYNC_EXCLUDE_ARGS[@]}" "$SOURCE_DIR"/ "$WSL_SYNC_DIR"/
  else
    rsync -a "$SOURCE_DIR"/ "$WSL_SYNC_DIR"/
  fi
  echo "已完成 WSL 同步: $SOURCE_DIR -> $WSL_SYNC_DIR"
fi

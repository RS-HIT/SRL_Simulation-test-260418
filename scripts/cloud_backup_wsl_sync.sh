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

tar -czf "$ARCHIVE_PATH" -C "$SOURCE_DIR" .
echo "已完成云端备份: $ARCHIVE_PATH"

if [[ -n "$WSL_SYNC_DIR" ]]; then
  WSL_SYNC_DIR="$(realpath -m "$WSL_SYNC_DIR")"
  mkdir -p "$WSL_SYNC_DIR"
  rsync -a --exclude ".git/" "$SOURCE_DIR"/ "$WSL_SYNC_DIR"/
  echo "已完成 WSL 同步: $SOURCE_DIR -> $WSL_SYNC_DIR"
fi

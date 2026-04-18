# SRL_Simulation-test-260418

用于在 Linux/WSL 环境中完成项目云端备份，并可选同步到 Windows 目录。

## 使用方式

运行脚本：

```bash
bash scripts/cloud_backup_wsl_sync.sh <项目目录> <云端备份目录> [WSL同步目录]
```

如需排除大文件或敏感目录，可传入 `BACKUP_EXCLUDE_FILE`（tar exclude 规则文件）：

```bash
BACKUP_EXCLUDE_FILE=/path/to/exclude.txt \
bash scripts/cloud_backup_wsl_sync.sh <项目目录> <云端备份目录> [WSL同步目录]
```

如需自定义同步排除规则，可传入 `SYNC_EXCLUDES`（逗号分隔，默认 `.git/`）：

```bash
SYNC_EXCLUDES=".git/,node_modules/,dist/" \
bash scripts/cloud_backup_wsl_sync.sh <项目目录> <云端备份目录> [WSL同步目录]
```

示例：

```bash
bash scripts/cloud_backup_wsl_sync.sh \
  /home/runner/work/SRL_Simulation-test-260418/SRL_Simulation-test-260418 \
  /mnt/c/Users/<你的用户名>/OneDrive/project_backups \
  /mnt/c/Users/<你的用户名>/Desktop/SRL_Simulation-test-260418
```

执行后会：

1. 在云端备份目录中创建时间戳 `.tar.gz` 备份文件（默认包含完整项目内容）  
2. 如果提供了第 3 个参数，则将项目内容同步到对应目录（适合 WSL 与 Windows 目录同步，不会删除仅存在于目标目录中的文件，但同名文件会被更新覆盖）  
3. 同步默认排除 `.git/`，用于避免覆盖目标端版本库元数据；完整版本库仍会保留在压缩备份中

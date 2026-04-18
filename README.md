# SRL_Simulation-test-260418

用于在 WSL 环境中完成项目云端备份，并可选同步到 Windows 目录。

## 使用方式

运行脚本：

```bash
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

1. 在云端备份目录中创建时间戳 `.tar.gz` 备份文件（包含完整项目内容）  
2. 如果提供了第 3 个参数，则将项目内容同步到对应目录（适合 WSL 与 Windows 目录同步，不会删除目标目录已有文件）

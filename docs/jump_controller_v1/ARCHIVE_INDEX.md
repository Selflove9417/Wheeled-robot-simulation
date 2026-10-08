# 历史跳跃数据归档索引

本批次仅包含以下4项。原压缩流程均为gzip -1，tar -tzf与tar -dzf逐文件比对通过，状态VERIFIED。2026-10-08删除前重新检查gzip -t与tar -tzf，全部返回0，并记录压缩包SHA256。未重新压缩、覆盖或移动压缩包。

**原目录清理状态：COMPLETED（2026-10-08，用户最终确认后仅删除本批4个原始目录）**。此前thrust_early_repeat_20261008不在本批次范围。

归档验证记录（本地资料）：`/home/xy/bbot_jump_archives/batch_four_20261008_210618_ff270e/results.json`。删除前大小、完整性与SHA256见[本批次预检查](inventory/ARCHIVE_FOUR_20261008_PREFLIGHT.json)。历史实验报告和原验收记录保持原内容；原目录清理后，旧报告中的本地数据路径需按本表恢复。

| 实验 | 原路径 | 压缩包路径 | 校验状态 |
|---|---|---|---|
| clearance_apex_ab_20261008 | `/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/clearance_apex_ab_20261008` | `/home/xy/bbot_jump_archives/clearance_apex_ab_20261008.tar.gz` | VERIFIED；删除前完整性复查通过 |
| hip_momentum_ab_20261007 | `/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/hip_momentum_ab_20261007` | `/home/xy/bbot_jump_archives/hip_momentum_ab_20261007.tar.gz` | VERIFIED；删除前完整性复查通过 |
| landing_capture_gain_ab_20261007 | `/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/landing_capture_gain_ab_20261007` | `/home/xy/bbot_jump_archives/landing_capture_gain_ab_20261007.tar.gz` | VERIFIED；删除前完整性复查通过 |
| arrest_early_decel_ab_20261007 | `/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/arrest_early_decel_ab_20261007` | `/home/xy/bbot_jump_archives/arrest_early_decel_ab_20261007.tar.gz` | VERIFIED；删除前完整性复查通过 |

## 恢复命令

在需要恢复该实验时执行对应命令。`--keep-old-files`阻止覆盖已经存在的同名文件；每个压缩包内包含实验顶层目录，因此解压目标必须是原父目录。以下仅记录命令，本轮未执行恢复。

```bash
tar --keep-old-files -xzf '/home/xy/bbot_jump_archives/clearance_apex_ab_20261008.tar.gz' -C '/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials'
tar --keep-old-files -xzf '/home/xy/bbot_jump_archives/hip_momentum_ab_20261007.tar.gz' -C '/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials'
tar --keep-old-files -xzf '/home/xy/bbot_jump_archives/landing_capture_gain_ab_20261007.tar.gz' -C '/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials'
tar --keep-old-files -xzf '/home/xy/bbot_jump_archives/arrest_early_decel_ab_20261007.tar.gz' -C '/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials'
```

## 清理空间记录

用户最终确认后完成限定删除，4个原目录均已不存在；压缩包及此前校验记录均保留。删除前逐包SHA256与完整性复查记录一致，没有重新压缩或重新实验分析。

- 删除前4目录合计分配空间：12132585472字节。
- 删除前后文件系统可用空间净增加：12132585472字节（可能受同时发生的其他磁盘活动影响）。
- 删除后文件系统可用空间：398158831616字节。
- [删除与空间记录](inventory/ARCHIVE_FOUR_20261008_DELETION.json)。

## 第二批7项归档与清理（2026-10-08）

按用户明确批准，仅处理本节7项。删除前逐项真实路径/非符号链接、原VERIFIED记录、tar -tzf及tar -dzf复核；异常项不删除。压缩包及原校验记录均保留，未重新压缩或分析实验。

| 原路径 | 压缩包路径 | 本轮状态 |
|---|---|---|
| `/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_release_073_single_20261008` | `/home/xy/bbot_jump_archives/thrust_release_073_single_20261008.tar.gz` | DELETED |
| `/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/ground_motion_probe_20261006_134820` | `/home/xy/bbot_jump_archives/ground_motion_probe_20261006_134820.tar.gz` | DELETED |
| `/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_origin_diagnostic_20261007` | `/home/xy/bbot_jump_archives/thrust_origin_diagnostic_20261007.tar.gz` | DELETED |
| `/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/native_engine_standing_probe_20261006_105035` | `/home/xy/bbot_jump_archives/native_engine_standing_probe_20261006_105035.tar.gz` | DELETED |
| `/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/angular_momentum_20261007` | `/home/xy/bbot_jump_archives/angular_momentum_20261007.tar.gz` | DELETED |
| `/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_height_032_ab_20261008` | `/home/xy/bbot_jump_archives/thrust_height_032_ab_20261008.tar.gz` | DELETED |
| `/home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials/thrust_terminal_forward_single_20261008` | `/home/xy/bbot_jump_archives/thrust_terminal_forward_single_20261008.tar.gz` | DELETED |

恢复对应实验（替换NAME为本表完整目录名；命令仅记录，未执行）：

```bash
NAME=angular_momentum_20261007
tar --keep-old-files -xzf "/home/xy/bbot_jump_archives/${NAME}.tar.gz" -C /home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/flat_jump_trials
```

删除目录原分配空间合计5059461120字节，文件系统可用空间净增加5059461120字节；后者可能受其他磁盘活动影响。当前可用空间402200727552字节。[本轮逐项删除及空间记录](inventory/ARCHIVE_SEVEN_20261008_DELETION.json)。

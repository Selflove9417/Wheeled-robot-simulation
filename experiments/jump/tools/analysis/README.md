# 飞行角动量离线分析

`analyze_flight_momentum.py` 是通用实现，原路径
`src/bbot_balance_controller/scripts/analyze_flight_momentum.py` 保留可导入兼容模块。
仅使用 Python 标准库，不需要 ROS 或 Gazebo。

## 使用与含义

从工作区根目录执行以下任一入口，传入相同的现有日志路径：

```bash
python3 experiments/jump/tools/analysis/analyze_flight_momentum.py /path/to/trial_log.csv
python3 src/bbot_balance_controller/scripts/analyze_flight_momentum.py /path/to/trial_log.csv
```

支持一个或多个位置参数 `logs` 和原有 `--help`；没有默认输入文件，输出到标准输出，不写实验数据。
CLI 从第一条 FLIGHT 状态取时间基准，按原有时间偏移及 IMU/关节采样配对规则报告结果。
`momentum(row)` 返回机身、腿、轮轨道、轮自转及总角动量，单位 kg·m²/s。
这是固定七刚体模型的诊断估计，省略 10 g IMU 和载荷，**不是 Gazebo 原生物理真值**。
直接调用 `momentum()` 本身不执行 `sample()` 的采样时效和配对检查；调用者原有检查职责保持不变。

## 兼容约定

新实现与迁移前原文件字节一致，质量、惯量、公式、坐标系及单位没有修改。
旧模块单向加载新实现并导出所有原公开名称，包括 `momentum`、`sample`、
`add`、`scale`、`perp`、`rotate`、`main`、`MASS`、`IXX` 及原导入名称。
新实现不反向导入旧模块，不形成循环依赖。
原入口需要与工作区相对目录结构一起保留；独立复制旧入口并不是受支持的新部署方式。
历史冻结脚本中的 `/home/admin/...` 绝对路径原样保留，本次不解决其跨机器可移植性。

## 验证

详见 [验证记录](VALIDATION.json) 和 [原入口变更 diff](MIGRATION.diff)。
使用本地资料 `src/bbot_balance_controller/src/data_logs/flat_jump_trials/leg_chain_repeat_20260929/`
的三份现有日志，共 17,188 行，38 项检查通过：

- 新实现与原实现字节一致；全部公开名称、函数签名、模型常量一致。
- 全部日志行的五分量角动量结果精确一致；CLI 时间偏移采样结果一致。
- 迁移前、新入口、旧入口的多日志 CLI、帮助、缺失参数、未知参数、缺失文件、空日志、无效时间戳行为一致。
- 两个已知调用者原导入方式、全部日志行的角动量调用、CLI 帮助通过；推力窗口工具的 Trial 和 print_momentum 路径正常执行。
- 三份输入文件前后 SHA-256 不变。

异常调用栈中的文件路径和额外兼容入口栈帧随迁移变化；比较保留了异常类型、消息、退出码及标准输出。
未运行 Gazebo，未重跑完整关节力矩绘图流程；本记录是离线兼容性验证，不是物理验收。
没有解压归档实验。对照实现、验证驱动及输出保存在本机临时目录：
`/tmp/bbot_momentum_migration_20261008_220524/`（临时目录可能被系统清理）。

## 回退

将本目录 `analyze_flight_momentum.py` 原样复制回
`src/bbot_balance_controller/scripts/analyze_flight_momentum.py` 即恢复原独立实现；
其字节已与迁移前备份核对一致。也可使用上述临时备份恢复。
确认恢复后，只移除本次新增的四个文件：本目录下的实现、README.md、VALIDATION.json、MIGRATION.diff。
不要删除整个 analysis 目录或任何实验资料。

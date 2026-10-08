# 跳跃响应绘图工具

通用实现为本目录`plot_jump_log.py`。原入口`src/bbot_balance_controller/src/data_logs/plot_jump_log.py`保留为兼容包装，命令行参数和公开绘图函数可继续使用。无需修改控制器、launch、CMake或历史实验脚本；没有移动实验数据。

## 使用与默认行为

从工作区根目录运行任一入口，参数相同：

```bash
python3 experiments/jump/tools/plotting/plot_jump_log.py [CSV] [-o PNG]
python3 src/bbot_balance_controller/src/data_logs/plot_jump_log.py [CSV] [-o PNG]
```

可选参数保持`--geometry`、`--landing-metrics`、`--time-window START END`。geometry与landing-metrics须成对且同试次；校验与错误处理不变。显式相对路径仍相对于调用者当前工作目录。

两个入口无论从哪里启动，默认目录均为工作区的`src/bbot_balance_controller/src/data_logs/`：优先读取`jump_velocity_control_log.csv`，不存在则选择`jump_control_log.csv`；默认输出`jump_performance_single.png`，已存在时按原逻辑递增后缀，保留旧图。

实现迁移只改变`main()`定位上述默认目录的方式。Effort有效性/NaN断线、左右髋膝角度、COM/净空/姿态/轮速及真实轮轴位姿投影、物理首触相对位移全部保留原算法与语义。没有用轮速积分替换真实位移。

## 本轮兼容性验证（2026-10-08）

使用仍在本地的`catch_world_geometry_single_20261008/physical_runs/T1/{velocity_log.csv,geometry.csv}`和同实验`analysis/T1/metrics.json`。这些是本地资料；未解压已归档实验，未启动Gazebo，也不作新的性能分析或验收。

迁移前脚本、旧路径兼容入口、新实现三者对照：

| 检查 | 结果 |
|---|---|
| 普通7联图 | 三份PNG字节/SHA256完全一致 |
| 原生轮轴位移8联图 | 三份PNG字节/SHA256完全一致 |
| 指定时间窗口8联图 | 三份PNG字节/SHA256完全一致 |
| `--help` | 退出码、stdout、stderr一致 |
| 只提供geometry而缺metrics | 均按原规则拒绝，退出码和错误信息一致 |
| 默认输入、输出及已有图后缀 | 路径选择一致；拦截渲染核对，不写默认数据目录 |
| 默认velocity CSV缺失时的fallback | 路径选择一致；只模拟exists查询，不改/创建/移动CSV |
| 输入保护 | 使用的CSV、geometry、metrics SHA256前后一致 |
| 实现内容 | 除默认目录定位外，与迁移前源码逐字一致 |

详细输入、图像哈希、默认路径和结果见[VALIDATION.json](VALIDATION.json)。9张验证图仅生成于`/tmp/bbot_plot_migration_20261008/`，没有覆盖或移动任何已有PNG。CLI对照使用临时可写MPLCONFIGDIR，避免本机缓存目录不可写导致随机临时路径提示；没有改产品环境或绘图设置。这是离线兼容性验证，不是新增仿真实验。

## 回退方法

本轮修改前的完整脚本备份为`/tmp/bbot_plot_migration_20261008/plot_jump_log.before.py`，SHA256为`4151757ffb9cb6002b6a10f215ab06955eb179134fdacdf08224b12dfb43d1d4`。该备份保留迁移前用户已有绘图修改；不要用Git HEAD旧版本覆盖它。

备份仍在时，将其复制回原入口即可回退，通用实现可原位保留，不必删除：

```bash
cp /tmp/bbot_plot_migration_20261008/plot_jump_log.before.py /home/xy/bbot_ws_new/src/bbot_balance_controller/src/data_logs/plot_jump_log.py
```

若临时备份已消失，可从新实现还原；以下检查原始哈希后才写回，若新实现已有其他改动则停止：

```python
from pathlib import Path
import hashlib
root = Path('/home/xy/bbot_ws_new')
source = (root / 'experiments/jump/tools/plotting/plot_jump_log.py').read_text()
relocated = ("    directory = (Path(__file__).resolve().parents[4] /\n"
             "                 'src/bbot_balance_controller/src/data_logs')\n")
assert source.count(relocated) == 1
source = source.replace(relocated, '    directory = Path(__file__).resolve().parent\n')
assert hashlib.sha256(source.encode()).hexdigest() == '4151757ffb9cb6002b6a10f215ab06955eb179134fdacdf08224b12dfb43d1d4'
(root / 'src/bbot_balance_controller/src/data_logs/plot_jump_log.py').write_text(source)
```

本节仅记录回退方法，本轮未执行回退。未提交、未推送，其他工具及运行配置保持原位。

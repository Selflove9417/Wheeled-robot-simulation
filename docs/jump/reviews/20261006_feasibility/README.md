# 离线可行性复核材料

从工作区根目录执行。以下命令不启动仿真。

## 飞行边界复算

输入来自原生对照 `velocity_clearance_landing_20261005_224308` 的 4.297 s 同物理步后状态。`raw_takeoff_state.json` 保存九维 q/v、输入有效计数和连续无接触确认；`takeoff_input.txt` 是同一组 q/v 的桥接输入。

```bash
g++ -std=c++17 -O1 -UNDEBUG \
  -I src/bbot_balance_controller/include \
  -I src/bbot_kinematics/include -I /usr/include/eigen3 \
  docs/jump/reviews/20261006_feasibility/flight_review.cpp \
  -o /tmp/bbot_flight_feasibility_review
/tmp/bbot_flight_feasibility_review < docs/jump/reviews/20261006_feasibility/takeoff_input.txt
```

`flight_review.json` 是本轮输出。实际边界拒绝、人工已腾空边界接受均为离线结果。使用源码哈希一致的版本复算；不能用未来变化的模型输出覆盖本轮记录。

## 非有限伺服输入复现

只提取原 Python 文件的常量和纯函数，避开 ROS 节点初始化：

```bash
python3 - <<'PY'
import ast, math
from pathlib import Path
p = Path('src/bbot_balance_controller/scripts/wheel_effort_velocity_servo.py')
tree = ast.parse(p.read_text())
subset = ast.Module(body=[n for n in tree.body if isinstance(n, (ast.Assign, ast.FunctionDef))], type_ignores=[])
namespace = {}
exec(compile(subset, str(p), 'exec'), namespace)
print('NaN target:', namespace['wheel_targets'](math.nan, 0.0))
print('NaN measured:', namespace['commanded_effort']((0., 0.), (math.nan, math.nan), 1., True, True, False))
PY
```

现版本分别输出 `(30.0, 30.0)` 和 `((10.0, 10.0), (nan, nan))`。

## 60 ms 运动学界限

`current_tuck_kinematic_bound.json` 使用真实初始位置/速度及当前目标。髋施加最大负加速度至 −11 rad/s，之后保持；膝施加最大正加速度至 +15 rad/s，之后保持，时间均为 60 ms。即使放弃终点速度为零的约束，仍无法到达目标；因此不需要依靠五次多项式的具体形状来解释拒绝。

## 既有测试

```bash
source setup_env.sh
python3 src/bbot_balance_controller/test/test_wheel_effort_velocity_servo.py
ctest --test-dir build_support_allocator/bbot_balance_controller -R '^velocity_launch_capture$' --output-on-failure
ctest --test-dir build_native_allocator/bbot_balance_controller -R '^(allocator_flight_plan|allocator_contact_history|allocator_leg_symmetry|thrust_support_dynamics|thrust_support_allocator)$' --output-on-failure
```

`verification.json` 记录本轮范围。`source_preservation.json` 是 257 个文件的前后哈希一致核对，`evidence_hashes.json` 绑定历史原始输入和关键测试源码。本轮没有重新构建这些既有 CTest 程序，测试结果属于现有构建产物；飞行边界桥接程序则以当前头文件单独编译。

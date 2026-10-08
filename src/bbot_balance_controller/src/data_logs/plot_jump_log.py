#!/usr/bin/env python3
"""兼容入口；通用实现位于 experiments/jump/tools/plotting/plot_jump_log.py。

命令行参数、默认data_logs输入输出及公开绘图函数保持可用。
"""
from pathlib import Path as _Path
from runpy import run_path as _run_path

_implementation = (_Path(__file__).resolve().parents[4] /
                   'experiments/jump/tools/plotting/plot_jump_log.py')
globals().update({name: value for name, value in
                  _run_path(str(_implementation)).items()
                  if not name.startswith('__')})

if __name__ == '__main__':
    main()

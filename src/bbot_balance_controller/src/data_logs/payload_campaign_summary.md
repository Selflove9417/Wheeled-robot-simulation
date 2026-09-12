# Real Physical Payload Experiment (P0 - P4) Results

| 编号 | 附加载荷工况 | 控制方法 | 首次粗捕 $T_{\rm est}$ | 最大位置漂移 $|e_x|_{\max}$ | 总收敛 $T_c$ | 末端平均位置误差 $e_{x,{\rm end}}$ (末15s) | 最大姿态偏差 $\max|e_\theta|$ | 峰值力矩 $\max|u|$ | 最终等效补偿 $\hat{\Delta y}_c$ |
| :--- | :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **P0** | 0kg Baseline (Nominal) | Nominal GS-LQR | — | 1.98 mm | — (未补偿) | +1.98 ± 0.00 mm | 0.03° | 0.10 Nm | +0.000 mm |
| **P1** | 1kg Front (+0.10m) Nominal | Nominal GS-LQR | — | 629.45 mm | — (未补偿) | +629.41 ± 0.04 mm | 1.07° | 2.83 Nm | +0.000 mm |
| **P2** | 1kg Front (+0.10m) Adaptive | Adaptive GS-LQR | 4.09 s | 368.50 mm | 36.53 s | -1.41 ± 0.02 mm | 1.09° | 5.40 Nm | +9.055 mm |
| **P3** | 1kg Rear (-0.10m) Nominal | Nominal GS-LQR | — | 305.07 mm | — (未补偿) | -305.06 ± 0.01 mm | 0.53° | 1.76 Nm | +0.000 mm |
| **P4** | 1kg Rear (-0.10m) Adaptive | Adaptive GS-LQR | 3.54 s | 147.28 mm | 34.19 s | -0.87 ± 0.13 mm | 0.54° | 3.24 Nm | -4.372 mm |

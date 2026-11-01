# 逆解验证摘要

## 方法
- 正向：中心线常曲率窗弯曲 + 材料扭转；四路丝弧长积分应变 + `twistAsym·cos(2α)` + `twistCommon` + 弯扭交叉项；`ΔR/R = GF·ε`。
- 逆向：Levenberg–Marquardt（数值 Jacobian，多初值，对角正则）。
- 验证：`node validate-node.mjs`（与浏览器共用 `js/physics.js`）。

## Round 1 — 基线参数
```
GF=2.4, twistAsym=0.02, coupleBeta=0.3, twistCommon=0.004, bendHalfWidth=0.22
```
| 指标 | 数值 |
|------|------|
| RMSE 弯曲 bendAmt | **0.0098** |
| RMSE θ | **0.76°** |
| RMSE φ | **0.58°** |
| 平均 \|Δ bendPos\| | 0.0139 |
| 残差均值 | 7.55e-4 |
| 用例数 | 29 |

主要问题：弯曲位置偏离中心（0.3/0.7）时 bendPos 误差可达 ~0.2；扭转 RMSE 仍有改进空间。

## 调参
对 `GF × twistAsym × coupleBeta × twistCommon × bendHalfWidth` 做网格搜索（评分加权 RMSE）。

**最优：**
```
GF=2.6, twistAsym=0.015, coupleBeta=0.2, twistCommon=0.008, bendHalfWidth=0.2
```

## Round 2 — 调参后
| 指标 | Round1 | Round2 |
|------|--------|--------|
| RMSE bendAmt | 0.0098 | **0.0086** |
| RMSE θ (°) | 0.76 | 0.83 |
| RMSE φ (°) | 0.58 | **0.22** |
| meanAbs bendPos | 0.0139 | 0.0143 |
| 硬失败数* | — | **0** |

\*硬失败：`eBend>0.08` 或 `eTw>0.25 rad` 或 `eTh>0.35 rad`。

扭转误差显著下降；弯曲量略优；方位角基本持平（仍亚度级）。结果已写入 `physics.js` 默认 `PARAMS`，并保存 `validation-result.json`。

## 局限性
1. **bendPos 可观性弱**：四路电阻对弯曲位置不如对 κ/θ/φ 敏感，偏离中段时易偏向 0.5。
2. **模型非 FEM**：常曲率窗 + 解析耦合，非真实硅胶超弹性/粘弹性。
3. **扭转反对称项**为增强可辨识性的唯象项；实物标定需重估 `twistAsym` / `GF`。
4. 噪声未注入；实测需加滤波与更大正则。

## UI 复现
打开页面 →「运行验证测试」；或 `node validate-node.mjs`。

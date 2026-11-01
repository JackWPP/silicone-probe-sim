# 硅胶探头形变与压力实时仿真

浏览器端 Three.js 单页应用：φ26×210 mm 医用硅胶柱体（圆滑头部）、四路轴向电阻丝可视化、压力热力图，以及基于 **Levenberg–Marquardt** 的形变逆解。

## 在线演示

公开地址（GitHub Pages，`main` 分支根目录）：

https://jackwpp.github.io/silicone-probe-sim/

样式与脚本使用相对路径（`css/`、`js/`），Three.js 仍从 HTTPS CDN 加载。

## 快速运行

```bash
cd silicone-probe-sim
# 任选一种静态服务（推荐，避免部分浏览器对模块/CORS 限制）
python3 -m http.server 8765
# 然后打开 http://localhost:8765/
```

也可直接用浏览器打开 `index.html`（需能访问 CDN：Three.js r128）。

## 目录

| 路径 | 说明 |
|------|------|
| `index.html` | 页面入口 |
| `css/style.css` | UI |
| `js/physics.js` | 正向/LM 逆解/验证/压力场 |
| `js/geometry3d.js` | 可变形网格与电阻丝 |
| `js/app.js` | 场景与面板 |
| `ARCHITECTURE.md` | 架构与算法选型 |
| `VALIDATION.md` | 调参前后误差 |
| `validate-node.mjs` | 无头验证与网格调参 |

## 使用要点

1. **正向**：左侧调弯曲程度/方向/扭转/位置 → 右侧看四路 ΔR/R；3D 中彩线为电阻丝，表面色为压力。
2. **逆向**：切换模式，输入四路 ΔR/R，或点「正向→逆解演示」→ LM 更新 3D。
3. **验证**：点「运行验证测试」或 `node validate-node.mjs`。
4. **截图 / 重置**：顶栏操作区按钮。

## 逆解一句话

最小化 `Σ(ΔR_pred − ΔR_meas)²`，状态为 `(bendAmt, θ, φ, bendPos)`，数值差分 Jacobian + 阻尼最小二乘，并计入弯扭耦合与多初值。

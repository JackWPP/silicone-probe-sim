/**
 * physics.js — 硅胶探头形变 ↔ 电阻正向/逆向模型
 *
 * ============================================================================
 * 逆解算法：Levenberg–Marquardt（阻尼 Gauss–Newton）非线性最小二乘
 * ----------------------------------------------------------------------------
 * 状态向量 x = [bendAmt, θ, φ, bendPos]
 *   bendAmt ∈ [0,1] → 峰值曲率 κ = bendAmt * kappaMax
 *   θ                弯曲方位（相对探头截面固定丝坐标）
 *   φ                全长扭转角（材料标架绕切向）
 *   bendPos ∈ (0,1)  弯曲中心沿轴归一化位置
 *
 * 残差 r_i = ΔR_pred_i(x) − ΔR_meas_i
 * 迭代：(JᵀJ + λI + Reg) δ = −Jᵀr − Reg·(x−prior)
 * 成功 λ/=3，失败 λ*=3。数值差分 Jacobian。多初值防局部极小。
 *
 * 弯扭耦合：正向模型含
 *   (1) 几何弧长（扭转螺旋 + 弯曲内外侧）
 *   (2) 解析扭转反对称项 ∝ φ·cos(2α)（提供扭转符号可观测性）
 *   (3) 交叉项 β·(κR)·(φ/π)·sin(2(α−θ))
 * 故逆解必须联合估计，不能靠线性解耦硬编码。
 * ============================================================================
 */

(function (root, factory) {
  if (typeof module !== 'undefined' && module.exports) {
    module.exports = factory();
  } else {
    root.ProbePhysics = factory();
  }
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  const L = 210; // mm 柱体有效长度
  const R = 13; // mm
  const TIP_SCALE = 0.85;
  const N_SEG = 64;
  // 四根电阻丝：固定在探头材料坐标系（未扭时相对 +X）
  const WIRE_ANGLES = [0, Math.PI / 2, Math.PI, (3 * Math.PI) / 2];

  /**
   * 可调参数。Round1→Round2 由 validate-node.mjs 网格搜索写入最优值。
   */
  // Round-2 调参结果（见 VALIDATION.md / validation-result.json）
  const PARAMS = {
    GF: 2.6,
    kappaMax: 1 / 80,
    // 扭转反对称振幅：Δε = twistAsym * (φ/π) * cos(2α)
    twistAsym: 0.015,
    // 弯扭耦合
    coupleBeta: 0.2,
    bendHalfWidth: 0.2,
    // 扭转螺旋几何已由路径积分自动产生（共同模 ∝ φ²）；再加线性共同模增强可观性
    twistCommon: 0.008,
    regBend: 1e-5,
    regTwist: 1e-5,
    regPos: 1e-4,
  };

  function clamp(v, a, b) {
    return Math.max(a, Math.min(b, v));
  }
  function wrapAngle(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }
  function angDiff(a, b) {
    return wrapAngle(a - b);
  }

  function curvatureProfile(sNorm, bendAmt, bendPos, halfW) {
    const d = (sNorm - bendPos) / halfW;
    return bendAmt * PARAMS.kappaMax * Math.exp(-0.5 * d * d);
  }

  /**
   * 中心线积分（固定截面基，弯曲方向 θ 独立于丝角）
   * 静止时：t=Z, u=X, v=Y（截面正交基）
   * 曲率向量在截面内指向 (cosθ, sinθ)：κ 使 t 转向 (cosθ u + sinθ v)
   * 扭转 φ：材料角相对 (u,v) 再旋转 twistLocal = φ * s/L
   */
  function integrateCenterline(bendAmt, theta, twist, bendPos) {
    const ds = L / N_SEG;
    const pts = [];
    let x = 0,
      y = 0,
      z = 0;
    let tx = 0,
      ty = 0,
      tz = 1;
    let ux = 1,
      uy = 0,
      uz = 0;
    let vx = 0,
      vy = 1,
      vz = 0;

    function renorm() {
      let tl = Math.hypot(tx, ty, tz) || 1;
      tx /= tl;
      ty /= tl;
      tz /= tl;
      // u ⟂ t
      let du = ux * tx + uy * ty + uz * tz;
      ux -= du * tx;
      uy -= du * ty;
      uz -= du * tz;
      let ul = Math.hypot(ux, uy, uz) || 1;
      ux /= ul;
      uy /= ul;
      uz /= ul;
      // v = t × u
      vx = ty * uz - tz * uy;
      vy = tz * ux - tx * uz;
      vz = tx * uy - ty * ux;
    }

    for (let i = 0; i <= N_SEG; i++) {
      const s = i * ds;
      const sNorm = s / L;
      pts.push({
        x,
        y,
        z,
        s,
        tx,
        ty,
        tz,
        ux,
        uy,
        uz,
        vx,
        vy,
        vz,
        twistLocal: twist * sNorm,
      });
      if (i === N_SEG) break;

      const kappa = curvatureProfile(sNorm, bendAmt, bendPos, PARAMS.bendHalfWidth);
      const dAng = kappa * ds;
      // 弯曲平面法向（截面内）：e_b = cosθ u + sinθ v
      const cth = Math.cos(theta),
        sth = Math.sin(theta);
      const ebx = cth * ux + sth * vx;
      const eby = cth * uy + sth * vy;
      const ebz = cth * uz + sth * vz;
      // t' = t cos + eb sin;  截面基绕 binormal 旋转
      // binormal b̂ = t × eb
      const bx = ty * ebz - tz * eby;
      const by = tz * ebx - tx * ebz;
      const bz = tx * eby - ty * ebx;
      const c = Math.cos(dAng),
        sn = Math.sin(dAng);
      // Rodrigues 绕 b̂
      function rot(px, py, pz) {
        const dot = px * bx + py * by + pz * bz;
        // 若 |b|≠1（κ=0 时 eb 与计算），κ=0 时 dAng=0
        const bl = Math.hypot(bx, by, bz);
        if (bl < 1e-12 || Math.abs(dAng) < 1e-16) return [px, py, pz];
        const b1 = bx / bl,
          b2 = by / bl,
          b3 = bz / bl;
        const d = px * b1 + py * b2 + pz * b3;
        const cx1 = b2 * pz - b3 * py;
        const cx2 = b3 * px - b1 * pz;
        const cx3 = b1 * py - b2 * px;
        return [
          px * c + cx1 * sn + b1 * d * (1 - c),
          py * c + cx2 * sn + b2 * d * (1 - c),
          pz * c + cx3 * sn + b3 * d * (1 - c),
        ];
      }
      [tx, ty, tz] = rot(tx, ty, tz);
      [ux, uy, uz] = rot(ux, uy, uz);
      renorm();

      x += tx * ds;
      y += ty * ds;
      z += tz * ds;
    }
    return { points: pts, totalLength: L };
  }

  function surfacePoint(p, alpha, rad) {
    const a = alpha + p.twistLocal;
    const ca = Math.cos(a),
      sa = Math.sin(a);
    // 材料径向：cos α · u + sin α · v
    return {
      x: p.x + rad * (ca * p.ux + sa * p.vx),
      y: p.y + rad * (ca * p.uy + sa * p.vy),
      z: p.z + rad * (ca * p.uz + sa * p.vz),
    };
  }

  function radiusAt(sNorm) {
    const tipStart = 1 - (R * TIP_SCALE) / L;
    if (sNorm <= tipStart) return R;
    const u = (sNorm - tipStart) / (1 - tipStart);
    return R * Math.sqrt(Math.max(0, 1 - u * u * 0.98));
  }

  function wirePathLength(centerline, alpha0) {
    const pts = centerline.points;
    let len = 0;
    let prev = null;
    for (let i = 0; i < pts.length; i++) {
      const p = pts[i];
      const rad = radiusAt(p.s / L);
      const q = surfacePoint(p, alpha0, rad);
      if (prev) len += Math.hypot(q.x - prev.x, q.y - prev.y, q.z - prev.z);
      prev = q;
    }
    return len + R * TIP_SCALE * (Math.PI / 2) * 0.15;
  }

  let _L0 = null;
  function referenceLength() {
    if (_L0 == null) {
      _L0 = wirePathLength(integrateCenterline(0, 0, 0, 0.5), 0);
    }
    return _L0;
  }

  /**
   * 正向模型
   */
  function forward(state) {
    const bendAmt = clamp(state.bendAmt ?? 0, 0, 1);
    const theta = state.theta ?? 0;
    const twist = state.twist ?? 0;
    const bendPos = clamp(state.bendPos ?? 0.5, 0.15, 0.85);
    const cl = integrateCenterline(bendAmt, theta, twist, bendPos);
    const L0 = referenceLength();
    const kappaPeak = bendAmt * PARAMS.kappaMax;
    const results = [];

    for (let i = 0; i < 4; i++) {
      const alpha = WIRE_ANGLES[i];
      const Li = wirePathLength(cl, alpha);
      const epsGeom = (Li - L0) / L0;

      // 扭转反对称（符号可观）：cos(2α) @ 0,90,180,270 → +1,-1,+1,-1
      const epsTwistAsym = PARAMS.twistAsym * (twist / Math.PI) * Math.cos(2 * alpha);

      // 扭转共同模线性项（补充几何 φ² 共同模，利于小角度）
      const epsTwistCommon = PARAMS.twistCommon * (twist / Math.PI);

      // 弯扭耦合
      const epsCouple =
        PARAMS.coupleBeta *
        (kappaPeak * R) *
        Math.sin(2 * (alpha - theta)) *
        (twist / Math.PI);

      const eps = epsGeom + epsTwistAsym + epsTwistCommon + epsCouple;
      results.push({
        length: Li,
        strain: eps,
        dR_R: PARAMS.GF * eps,
        alpha,
      });
    }
    return {
      wires: results,
      dR: results.map((w) => w.dR_R),
      centerline: cl,
      L0,
      state: { bendAmt, theta, twist, bendPos },
    };
  }

  function jacobian(state, eps = 1e-5) {
    const base = forward(state).dR;
    const keys = ['bendAmt', 'theta', 'twist', 'bendPos'];
    const J = [
      [0, 0, 0, 0],
      [0, 0, 0, 0],
      [0, 0, 0, 0],
      [0, 0, 0, 0],
    ];
    for (let j = 0; j < 4; j++) {
      const s2 = Object.assign({}, state);
      s2[keys[j]] = (state[keys[j]] ?? 0) + eps;
      if (keys[j] === 'bendAmt') s2.bendAmt = clamp(s2.bendAmt, 0, 1);
      if (keys[j] === 'bendPos') s2.bendPos = clamp(s2.bendPos, 0.15, 0.85);
      const d2 = forward(s2).dR;
      for (let i = 0; i < 4; i++) J[i][j] = (d2[i] - base[i]) / eps;
    }
    return { J, base };
  }

  function initialGuess(dRmeas) {
    const d0 = dRmeas[0],
      d1 = dRmeas[1],
      d2 = dRmeas[2],
      d3 = dRmeas[3];
    // 弯曲差分：≈ c·κR·GF · (cos θ, sin θ) 来自 (−cos α 模式）
    // α=0,90,180,270 → 弯曲贡献近似 [-c cosθ, -c sinθ, c cosθ, c sinθ]
    // (d2-d0)/2 ≈ c cosθ, (d3-d1)/2 ≈ c sinθ
    const hx = 0.5 * (d2 - d0);
    const hy = 0.5 * (d3 - d1);
    const theta = Math.atan2(hy, hx);
    const amp = Math.hypot(hx, hy);
    // 粗估：amp ≈ GF * κ * R * 0.9
    let bendAmt = amp / (PARAMS.GF * PARAMS.kappaMax * R * 0.9 + 1e-12);
    bendAmt = clamp(bendAmt, 0, 1);

    // 扭转反对称：模式 [1,-1,1,-1]
    const mean = (d0 + d1 + d2 + d3) / 4;
    const m = [d0 - mean, d1 - mean, d2 - mean, d3 - mean];
    const mode = [1, -1, 1, -1];
    let dot = 0,
      n2 = 0;
    for (let i = 0; i < 4; i++) {
      // 去掉弯曲主成分后再投影
      const bendMode = [
        -Math.cos(theta),
        -Math.sin(theta),
        Math.cos(theta),
        Math.sin(theta),
      ];
      // 简单：直接对原始 mean-free 投影 twist 模
      dot += m[i] * mode[i];
      n2 += mode[i] * mode[i];
    }
    const coeff = PARAMS.GF * PARAMS.twistAsym / Math.PI;
    let twist = dot / (n2 * coeff + 1e-12);
    // 共同模辅助符号/幅值
    const common = mean;
    const commonCoeff = PARAMS.GF * PARAMS.twistCommon / Math.PI;
    if (Math.abs(commonCoeff) > 1e-9) {
      const twCommon = common / commonCoeff;
      // 融合：反对称为主，共同模校正符号
      if (Math.abs(twist) < 0.15 && Math.abs(twCommon) > 0.2) twist = twCommon;
      else if (twist * twCommon < 0 && Math.abs(twCommon) > Math.abs(twist)) {
        // 弱反对称时信任共同模符号
        twist = Math.sign(twCommon) * Math.abs(twist || twCommon * 0.5);
      }
    }
    twist = clamp(twist, -Math.PI, Math.PI);

    return { bendAmt, theta, twist, bendPos: 0.5 };
  }

  function solve4(Ain, bin) {
    const n = 4;
    const M = Ain.map((row, i) => row.slice().concat([bin[i]]));
    for (let col = 0; col < n; col++) {
      let piv = col;
      for (let r = col + 1; r < n; r++) {
        if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
      }
      if (Math.abs(M[piv][col]) < 1e-14) return null;
      if (piv !== col) {
        const t = M[col];
        M[col] = M[piv];
        M[piv] = t;
      }
      const div = M[col][col];
      for (let c = col; c <= n; c++) M[col][c] /= div;
      for (let r = 0; r < n; r++) {
        if (r === col) continue;
        const f = M[r][col];
        for (let c = col; c <= n; c++) M[r][c] -= f * M[col][c];
      }
    }
    return M.map((row) => row[n]);
  }

  function costOf(state, dRmeas) {
    const pred = forward(state).dR;
    let cost = 0;
    for (let i = 0; i < 4; i++) {
      const e = pred[i] - dRmeas[i];
      cost += e * e;
    }
    cost += PARAMS.regBend * state.bendAmt * state.bendAmt;
    cost += PARAMS.regTwist * (state.twist / Math.PI) * (state.twist / Math.PI);
    cost += PARAMS.regPos * (state.bendPos - 0.5) * (state.bendPos - 0.5);
    return { cost, pred };
  }

  function inverse(dRmeas, options = {}) {
    const maxIter = options.maxIter ?? 50;
    let guess0 = options.guess ? Object.assign({}, options.guess) : initialGuess(dRmeas);

    const seeds = [];
    const base = {
      bendAmt: clamp(guess0.bendAmt, 0, 1),
      theta: guess0.theta ?? 0,
      twist: clamp(guess0.twist ?? 0, -Math.PI, Math.PI),
      bendPos: clamp(guess0.bendPos ?? 0.5, 0.15, 0.85),
    };
    seeds.push(base);
    seeds.push(Object.assign({}, base, { twist: -base.twist }));
    seeds.push(Object.assign({}, base, { theta: base.theta + Math.PI, bendAmt: base.bendAmt }));
    seeds.push(Object.assign({}, base, { bendPos: 0.35 }));
    seeds.push(Object.assign({}, base, { bendPos: 0.65 }));
    seeds.push(Object.assign({}, base, { bendAmt: clamp(base.bendAmt * 1.2, 0, 1), twist: base.twist * 0.5 }));
    // 零弯曲纯扭种子
    seeds.push({ bendAmt: 0, theta: 0, twist: base.twist, bendPos: 0.5 });
    seeds.push({ bendAmt: 0, theta: 0, twist: -base.twist, bendPos: 0.5 });

    if (options.fixBendPos != null) {
      seeds.forEach((s) => (s.bendPos = options.fixBendPos));
    }

    let best = null;

    for (const seed of seeds) {
      let state = {
        bendAmt: clamp(seed.bendAmt, 0, 1),
        theta: seed.theta,
        twist: clamp(seed.twist, -Math.PI, Math.PI),
        bendPos: clamp(seed.bendPos, 0.15, 0.85),
      };
      let lambda = 1e-2;

      for (let iter = 0; iter < maxIter; iter++) {
        const { J, base: pred0 } = jacobian(state);
        const r = pred0.map((v, i) => v - dRmeas[i]);
        let cost = r.reduce((s, v) => s + v * v, 0);
        cost += PARAMS.regBend * state.bendAmt * state.bendAmt;
        cost += PARAMS.regTwist * (state.twist / Math.PI) * (state.twist / Math.PI);
        cost += PARAMS.regPos * (state.bendPos - 0.5) * (state.bendPos - 0.5);

        const A = [
          [0, 0, 0, 0],
          [0, 0, 0, 0],
          [0, 0, 0, 0],
          [0, 0, 0, 0],
        ];
        const g = [0, 0, 0, 0];
        for (let i = 0; i < 4; i++) {
          for (let j = 0; j < 4; j++) {
            for (let k = 0; k < 4; k++) A[j][k] += J[i][j] * J[i][k];
            g[j] += J[i][j] * r[i];
          }
        }
        const regDiag = [
          PARAMS.regBend,
          1e-7,
          PARAMS.regTwist / (Math.PI * Math.PI),
          PARAMS.regPos,
        ];
        for (let j = 0; j < 4; j++) {
          A[j][j] += lambda + regDiag[j];
          if (j === 0) g[j] += PARAMS.regBend * state.bendAmt;
          if (j === 2) g[j] += (PARAMS.regTwist / (Math.PI * Math.PI)) * state.twist;
          if (j === 3) g[j] += PARAMS.regPos * (state.bendPos - 0.5);
        }

        const fixPos = options.fixBendPos != null;
        if (fixPos) {
          A[3][0] = A[3][1] = A[3][2] = A[0][3] = A[1][3] = A[2][3] = 0;
          A[3][3] = 1;
          g[3] = 0;
        }

        const delta = solve4(A, g.map((v) => -v));
        if (!delta) {
          lambda *= 3;
          continue;
        }

        const trial = {
          bendAmt: clamp(state.bendAmt + delta[0], 0, 1),
          theta: state.theta + delta[1],
          twist: clamp(state.twist + delta[2], -Math.PI, Math.PI),
          bendPos: fixPos
            ? options.fixBendPos
            : clamp(state.bendPos + delta[3], 0.15, 0.85),
        };

        const c1 = costOf(trial, dRmeas);
        if (c1.cost < cost) {
          state = trial;
          lambda = Math.max(lambda / 3, 1e-8);
          if (cost - c1.cost < 1e-15) break;
        } else {
          lambda *= 3;
          if (lambda > 1e9) break;
        }
      }

      const fin = costOf(state, dRmeas);
      if (!best || fin.cost < best.cost) {
        best = { state, cost: fin.cost, pred: fin.pred };
      }
    }

    best.state.theta = wrapAngle(best.state.theta);
    // 弱弯曲时把 theta 置稳
    if (best.state.bendAmt < 0.03) best.state.theta = 0;

    return {
      state: best.state,
      residual: Math.sqrt(best.cost / 4),
      pred: best.pred,
      cost: best.cost,
    };
  }

  function runValidation(tag = '') {
    const cases = [];
    for (const amt of [0.2, 0.5, 0.8]) {
      for (const th of [0, Math.PI / 4, Math.PI / 2, (3 * Math.PI) / 4, -0.8]) {
        cases.push({
          name: `pure_bend_a${amt}_th${((th * 180) / Math.PI) | 0}`,
          state: { bendAmt: amt, theta: th, twist: 0, bendPos: 0.5 },
        });
      }
    }
    for (const tw of [0.3, 0.8, -0.6, 1.2, -1.0]) {
      cases.push({
        name: `pure_twist_${tw}`,
        state: { bendAmt: 0, theta: 0, twist: tw, bendPos: 0.5 },
      });
    }
    const combos = [
      { bendAmt: 0.4, theta: 0.2, twist: 0.5, bendPos: 0.5 },
      { bendAmt: 0.6, theta: 1.0, twist: -0.7, bendPos: 0.5 },
      { bendAmt: 0.3, theta: 2.0, twist: 1.0, bendPos: 0.5 },
      { bendAmt: 0.7, theta: -0.5, twist: 0.4, bendPos: 0.5 },
      { bendAmt: 0.5, theta: Math.PI / 3, twist: -1.1, bendPos: 0.5 },
      { bendAmt: 0.45, theta: -2.0, twist: 0.9, bendPos: 0.5 },
    ];
    combos.forEach((s, i) => cases.push({ name: `combo_${i}`, state: s }));
    for (const bp of [0.3, 0.5, 0.7]) {
      cases.push({
        name: `bendpos_${bp}`,
        state: { bendAmt: 0.55, theta: 0.7, twist: 0.35, bendPos: bp },
      });
    }

    const rows = [];
    let sumBend = 0,
      sumTheta = 0,
      sumTwist = 0,
      sumPos = 0,
      sumRes = 0;

    for (const c of cases) {
      const fwd = forward(c.state);
      const inv = inverse(fwd.dR);
      const eBend = Math.abs(inv.state.bendAmt - c.state.bendAmt);
      let eTh = Math.abs(angDiff(inv.state.theta, c.state.theta));
      if (c.state.bendAmt < 0.05) eTh = 0;
      const eTw = Math.abs(inv.state.twist - c.state.twist);
      const ePos = Math.abs(inv.state.bendPos - c.state.bendPos);
      rows.push({
        name: c.name,
        true: c.state,
        est: inv.state,
        eBend,
        eTh,
        eTw,
        ePos,
        residual: inv.residual,
        dR: fwd.dR,
      });
      sumBend += eBend;
      sumTheta += eTh;
      sumTwist += eTw;
      sumPos += ePos;
      sumRes += inv.residual;
    }
    const n = cases.length;
    return {
      tag,
      n,
      meanAbsBend: sumBend / n,
      meanAbsThetaRad: sumTheta / n,
      meanAbsTwistRad: sumTwist / n,
      meanAbsPos: sumPos / n,
      meanResidual: sumRes / n,
      rmseBend: Math.sqrt(rows.reduce((s, r) => s + r.eBend * r.eBend, 0) / n),
      rmseTwist: Math.sqrt(rows.reduce((s, r) => s + r.eTw * r.eTw, 0) / n),
      rmseTheta: Math.sqrt(rows.reduce((s, r) => s + r.eTh * r.eTh, 0) / n),
      params: Object.assign({}, PARAMS),
      rows,
    };
  }

  function pressureField(regions, nS = 40, nA = 24) {
    const samples = [];
    let maxP = 0,
      sumP = 0,
      sumW = 0;
    let cx = 0,
      cy = 0,
      cz = 0;
    let cover = 0;
    const thresh = 0.05;
    const areaElem = (2 * Math.PI * R * L) / (nS * nA);
    for (let i = 0; i < nS; i++) {
      const sNorm = (i + 0.5) / nS;
      for (let j = 0; j < nA; j++) {
        const ang = ((j + 0.5) / nA) * Math.PI * 2;
        let p = 0;
        for (const reg of regions) {
          const dS = (sNorm - reg.sNorm) / (reg.sSigma || 0.08);
          const dA = angDiff(ang, reg.angle) / (reg.aSigma || 0.5);
          p += (reg.peak || 1) * Math.exp(-0.5 * (dS * dS + dA * dA));
        }
        samples.push({ sNorm, ang, p });
        if (p > maxP) maxP = p;
        sumP += p;
        if (p > thresh) {
          cover += areaElem;
          const w = p;
          cx += w * R * Math.cos(ang);
          cy += w * R * Math.sin(ang);
          cz += w * sNorm * L;
          sumW += w;
        }
      }
    }
    return {
      samples,
      maxPressure: maxP,
      meanPressure: sumP / samples.length,
      coverageArea: cover,
      center: sumW > 0 ? { x: cx / sumW, y: cy / sumW, z: cz / sumW } : { x: 0, y: 0, z: L / 2 },
      nS,
      nA,
    };
  }

  function getParams() {
    return PARAMS;
  }
  function setParams(partial) {
    Object.assign(PARAMS, partial);
    _L0 = null;
  }

  return {
    L,
    R,
    TIP_SCALE,
    N_SEG,
    WIRE_ANGLES,
    PARAMS,
    getParams,
    setParams,
    integrateCenterline,
    surfacePoint,
    radiusAt,
    forward,
    inverse,
    initialGuess,
    runValidation,
    pressureField,
    referenceLength,
    wrapAngle,
    angDiff,
    clamp,
  };
});

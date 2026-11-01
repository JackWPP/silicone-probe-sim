/**
 * geometry3d.js — Three.js 可变形硅胶探头网格 + 电阻丝 + 压力顶点色
 */
(function (global) {
  'use strict';

  const P = global.ProbePhysics;

  const WIRE_COLORS = [0xff4444, 0x44ff88, 0x4488ff, 0xffcc33];

  function createProbeGroup(THREE) {
    const group = new THREE.Group();
    group.name = 'probe';

    const radialSegs = 32;
    const axialSegs = P.N_SEG;
    // 柱体 + 半球头：轴向多一段用于头部
    const tipSegs = 12;
    const totalAxial = axialSegs + tipSegs;

    // —— 主体表面（BufferGeometry 手动构建以便变形）——
    const pos = [];
    const norm = [];
    const uv = [];
    const col = [];
    const idx = [];
    // 参数网格：(iAlong, jRadial)，含顶端极点
    const meta = []; // { kind:'side'|'tip', sNorm, ang } or pole

    function pushVertex(x, y, z, nx, ny, nz, u, v, metaObj) {
      const vi = pos.length / 3;
      pos.push(x, y, z);
      norm.push(nx, ny, nz);
      uv.push(u, v);
      col.push(0.75, 0.82, 0.88); // 默认硅胶青灰
      meta.push(metaObj);
      return vi;
    }

    // 侧面：s from 0..L，头部前
    const sideVerts = [];
    for (let i = 0; i <= axialSegs; i++) {
      const row = [];
      const s = (i / axialSegs) * P.L;
      const sNorm = s / P.L;
      for (let j = 0; j < radialSegs; j++) {
        const ang = (j / radialSegs) * Math.PI * 2;
        const x = P.R * Math.cos(ang);
        const y = P.R * Math.sin(ang);
        const z = s;
        row.push(
          pushVertex(x, y, z, Math.cos(ang), Math.sin(ang), 0, j / radialSegs, sNorm, {
            kind: 'side',
            sNorm,
            ang,
            i,
            j,
          })
        );
      }
      sideVerts.push(row);
    }
    for (let i = 0; i < axialSegs; i++) {
      for (let j = 0; j < radialSegs; j++) {
        const j2 = (j + 1) % radialSegs;
        const a = sideVerts[i][j],
          b = sideVerts[i][j2],
          c = sideVerts[i + 1][j],
          d = sideVerts[i + 1][j2];
        idx.push(a, c, b, b, c, d);
      }
    }

    // 半球头：球心在 z=L，稍扁
    const tipRx = P.R;
    const tipRz = P.R * P.TIP_SCALE;
    const tipRows = [];
    for (let i = 1; i <= tipSegs; i++) {
      const row = [];
      const phi = (i / tipSegs) * (Math.PI / 2); // 0 at equator already in side; start from first ring
      const ringR = tipRx * Math.cos(phi);
      const z = P.L + tipRz * Math.sin(phi);
      // sNorm 映射 >1 表示头部
      const sNorm = 1 + (i / tipSegs) * 0.08;
      for (let j = 0; j < radialSegs; j++) {
        const ang = (j / radialSegs) * Math.PI * 2;
        const x = ringR * Math.cos(ang);
        const y = ringR * Math.sin(ang);
        // 椭球法向
        const nx = Math.cos(phi) * Math.cos(ang);
        const ny = Math.cos(phi) * Math.sin(ang);
        const nz = Math.sin(phi);
        row.push(
          pushVertex(x, y, z, nx, ny, nz, j / radialSegs, sNorm, {
            kind: 'tip',
            phi,
            ang,
            sNorm: 1,
            tipU: i / tipSegs,
          })
        );
      }
      tipRows.push(row);
    }
    // 连接柱体末端到第一圈 tip
    const lastSide = sideVerts[axialSegs];
    for (let j = 0; j < radialSegs; j++) {
      const j2 = (j + 1) % radialSegs;
      const a = lastSide[j],
        b = lastSide[j2],
        c = tipRows[0][j],
        d = tipRows[0][j2];
      idx.push(a, c, b, b, c, d);
    }
    for (let i = 0; i < tipSegs - 1; i++) {
      for (let j = 0; j < radialSegs; j++) {
        const j2 = (j + 1) % radialSegs;
        const a = tipRows[i][j],
          b = tipRows[i][j2],
          c = tipRows[i + 1][j],
          d = tipRows[i + 1][j2];
        idx.push(a, c, b, b, c, d);
      }
    }
    // 极点
    const pole = pushVertex(0, 0, P.L + tipRz, 0, 0, 1, 0.5, 1.1, { kind: 'pole' });
    const lastTip = tipRows[tipSegs - 1];
    for (let j = 0; j < radialSegs; j++) {
      const j2 = (j + 1) % radialSegs;
      idx.push(lastTip[j], pole, lastTip[j2]);
    }

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    geometry.setAttribute('normal', new THREE.Float32BufferAttribute(norm, 3));
    geometry.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
    geometry.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
    geometry.setIndex(idx);
    geometry.computeBoundingSphere();

    const material = new THREE.MeshPhysicalMaterial({
      vertexColors: true,
      roughness: 0.35,
      metalness: 0.05,
      clearcoat: 0.4,
      clearcoatRoughness: 0.3,
      transparent: true,
      opacity: 0.92,
      side: THREE.DoubleSide,
    });
    const mesh = new THREE.Mesh(geometry, material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    group.add(mesh);

    // —— 电阻丝（Tube 用 Line 粗线近似：每次变形重建管道路径点）——
    const wireLines = [];
    for (let w = 0; w < 4; w++) {
      const wgeo = new THREE.BufferGeometry();
      const nPts = axialSegs + tipSegs + 2;
      const arr = new Float32Array(nPts * 3);
      wgeo.setAttribute('position', new THREE.BufferAttribute(arr, 3));
      const mat = new THREE.LineBasicMaterial({
        color: WIRE_COLORS[w],
        linewidth: 2,
      });
      // 用细管 Mesh 更清晰
      const tubeMat = new THREE.MeshStandardMaterial({
        color: WIRE_COLORS[w],
        emissive: WIRE_COLORS[w],
        emissiveIntensity: 0.25,
        roughness: 0.4,
      });
      // 占位：先放 Group，变形时替换 tube
      const wireGroup = new THREE.Group();
      wireGroup.userData = { index: w, color: WIRE_COLORS[w], tubeMat };
      group.add(wireGroup);
      wireLines.push(wireGroup);
    }

    // 底部端盖
    const capGeo = new THREE.CircleGeometry(P.R, radialSegs);
    const capMat = new THREE.MeshStandardMaterial({
      color: 0x9aa8b5,
      roughness: 0.5,
      side: THREE.DoubleSide,
    });
    const cap = new THREE.Mesh(capGeo, capMat);
    cap.rotation.x = Math.PI / 2;
    group.add(cap);

    const restPositions = new Float32Array(pos); // 拷贝

    return {
      group,
      mesh,
      geometry,
      meta,
      restPositions,
      wireLines,
      cap,
      radialSegs,
      axialSegs,
      tipSegs,
      tipRx,
      tipRz,
    };
  }

  /**
   * 根据物理状态变形网格与电阻丝
   */
  function deformProbe(probe, THREE, state, pressureSamples) {
    const cl = P.integrateCenterline(
      state.bendAmt || 0,
      state.theta || 0,
      state.twist || 0,
      state.bendPos ?? 0.5
    );
    const pts = cl.points;
    const posAttr = probe.geometry.attributes.position;
    const norAttr = probe.geometry.attributes.normal;
    const colAttr = probe.geometry.attributes.color;
    const meta = probe.meta;

    // 压力查找
    const pMap = new Map();
    if (pressureSamples) {
      for (const s of pressureSamples) {
        pMap.set(s.sNorm.toFixed(3) + '_' + s.ang.toFixed(3), s.p);
      }
    }
    function samplePressure(sNorm, ang) {
      if (!pressureSamples || !pressureSamples.length) return 0;
      // 最近邻粗采样
      let best = 0,
        bestD = Infinity;
      for (const s of pressureSamples) {
        const ds = s.sNorm - sNorm;
        let da = P.angDiff(s.ang, ang);
        const d = ds * ds * 4 + da * da;
        if (d < bestD) {
          bestD = d;
          best = s.p;
        }
      }
      return best;
    }

    function heatColor(p, maxP) {
      const t = maxP > 1e-6 ? Math.min(1, p / maxP) : 0;
      // 青灰 → 黄 → 红
      const baseR = 0.75,
        baseG = 0.82,
        baseB = 0.88;
      if (t < 1e-3) return [baseR, baseG, baseB];
      const r = baseR * (1 - t) + (t < 0.5 ? 1 : 1) * t + (t >= 0.5 ? 0.2 : 0);
      const g = baseG * (1 - t) + (t < 0.5 ? 1 * (t * 2) : 1 - (t - 0.5) * 2) * 0.9;
      const b = baseB * (1 - t) + 0.15 * (1 - t);
      return [
        Math.min(1, 0.75 + t * 0.5 + (t > 0.55 ? (t - 0.55) * 1.2 : 0)),
        Math.min(1, 0.82 * (1 - t * 0.85) + (t < 0.45 ? t * 0.6 : 0.2)),
        Math.min(1, 0.88 * (1 - t)),
      ];
    }

    let maxP = 0;
    if (pressureSamples) {
      for (const s of pressureSamples) if (s.p > maxP) maxP = s.p;
    }

    for (let vi = 0; vi < meta.length; vi++) {
      const m = meta[vi];
      let q, nx, ny, nz;
      if (m.kind === 'side') {
        const idx = Math.round(m.sNorm * (pts.length - 1));
        const p = pts[Math.max(0, Math.min(pts.length - 1, idx))];
        const rad = P.radiusAt(m.sNorm);
        q = P.surfacePoint(p, m.ang, rad);
        const a = m.ang + p.twistLocal;
        const ca = Math.cos(a),
          sa = Math.sin(a);
        nx = ca * p.ux + sa * p.vx;
        ny = ca * p.uy + sa * p.vy;
        nz = ca * p.uz + sa * p.vz;
        const pr = samplePressure(m.sNorm, m.ang);
        const hc = heatColor(pr, maxP);
        colAttr.setXYZ(vi, hc[0], hc[1], hc[2]);
      } else if (m.kind === 'tip') {
        const p = pts[pts.length - 1];
        // 头部：沿末端切向挤出椭球
        const phi = m.phi;
        const ringR = probe.tipRx * Math.cos(phi);
        const along = probe.tipRz * Math.sin(phi);
        const a = m.ang + p.twistLocal;
        const ca = Math.cos(a),
          sa = Math.sin(a);
        const rx = ca * p.ux + sa * p.vx;
        const ry = ca * p.uy + sa * p.vy;
        const rz = ca * p.uz + sa * p.vz;
        q = {
          x: p.x + p.tx * along + rx * ringR,
          y: p.y + p.ty * along + ry * ringR,
          z: p.z + p.tz * along + rz * ringR,
        };
        // 近似法向
        const nphiC = Math.cos(phi),
          nphiS = Math.sin(phi);
        nx = nphiC * rx + nphiS * p.tx;
        ny = nphiC * ry + nphiS * p.ty;
        nz = nphiC * rz + nphiS * p.tz;
        const pr = samplePressure(1, m.ang);
        const hc = heatColor(pr, maxP);
        colAttr.setXYZ(vi, hc[0], hc[1], hc[2]);
      } else {
        // pole
        const p = pts[pts.length - 1];
        q = {
          x: p.x + p.tx * probe.tipRz,
          y: p.y + p.ty * probe.tipRz,
          z: p.z + p.tz * probe.tipRz,
        };
        nx = p.tx;
        ny = p.ty;
        nz = p.tz;
        colAttr.setXYZ(vi, 0.75, 0.82, 0.88);
      }
      posAttr.setXYZ(vi, q.x, q.y, q.z);
      const nl = Math.hypot(nx, ny, nz) || 1;
      norAttr.setXYZ(vi, nx / nl, ny / nl, nz / nl);
    }
    posAttr.needsUpdate = true;
    norAttr.needsUpdate = true;
    colAttr.needsUpdate = true;
    probe.geometry.computeBoundingSphere();

    // 端盖：贴在起点截面
    const p0 = pts[0];
    probe.cap.position.set(p0.x, p0.y, p0.z);
    // 朝向 -t
    const zAxis = new THREE.Vector3(0, 0, 1);
    const tVec = new THREE.Vector3(p0.tx, p0.ty, p0.tz);
    probe.cap.quaternion.setFromUnitVectors(zAxis, tVec.clone().normalize());
    // Circle 默认在 XY，旋转后法向沿 Z；再绕使其法向 = -t? 原先 rotation.x=PI/2 被 quaternion 覆盖
    probe.cap.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), tVec);

    // 电阻丝管
    for (let w = 0; w < 4; w++) {
      const alpha = P.WIRE_ANGLES[w];
      const pathPts = [];
      for (let i = 0; i < pts.length; i++) {
        const p = pts[i];
        const rad = P.radiusAt(p.s / P.L) * 1.02;
        const q = P.surfacePoint(p, alpha, rad);
        pathPts.push(new THREE.Vector3(q.x, q.y, q.z));
      }
      // 延伸到圆滑头部
      const pLast = pts[pts.length - 1];
      for (let k = 1; k <= 8; k++) {
        const phi = (k / 8) * (Math.PI / 2) * 0.95;
        const ringR = probe.tipRx * Math.cos(phi) * 1.02;
        const along = probe.tipRz * Math.sin(phi);
        const a = alpha + pLast.twistLocal;
        const ca = Math.cos(a),
          sa = Math.sin(a);
        const rx = ca * pLast.ux + sa * pLast.vx;
        const ry = ca * pLast.uy + sa * pLast.vy;
        const rz = ca * pLast.uz + sa * pLast.vz;
        pathPts.push(
          new THREE.Vector3(
            pLast.x + pLast.tx * along + rx * ringR,
            pLast.y + pLast.ty * along + ry * ringR,
            pLast.z + pLast.tz * along + rz * ringR
          )
        );
      }
      const curve = new THREE.CatmullRomCurve3(pathPts);
      const tubeGeo = new THREE.TubeGeometry(curve, 48, 0.55, 6, false);
      const wg = probe.wireLines[w];
      while (wg.children.length) {
        const ch = wg.children.pop();
        if (ch.geometry) ch.geometry.dispose();
        wg.remove(ch);
      }
      wg.add(new THREE.Mesh(tubeGeo, wg.userData.tubeMat));
    }
  }

  global.ProbeGeometry3D = {
    createProbeGroup,
    deformProbe,
    WIRE_COLORS,
  };
})(typeof self !== 'undefined' ? self : this);

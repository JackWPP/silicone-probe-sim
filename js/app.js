/**
 * app.js — Three.js 场景、UI 绑定、正向/逆向模式、验证与截图
 */
(function () {
  'use strict';

  const P = window.ProbePhysics;
  const G = window.ProbeGeometry3D;

  const state = {
    mode: 'forward', // forward | inverse
    bendAmt: 0.35,
    theta: 0.6,
    twist: 0.4,
    bendPos: 0.5,
    // 逆解输入
    invDR: [0, 0, 0, 0],
    // 压力区
    pressureRegions: [
      { sNorm: 0.55, angle: 0.4, sSigma: 0.07, aSigma: 0.45, peak: 1.0 },
    ],
    lastForward: null,
    lastInverse: null,
  };

  let renderer, scene, camera, controls, probe, animId;
  let pressureCache = null;

  function $(id) {
    return document.getElementById(id);
  }

  function deg(r) {
    return (r * 180) / Math.PI;
  }
  function rad(d) {
    return (d * Math.PI) / 180;
  }

  function initThree() {
    const viewport = $('viewport');
    renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setSize(viewport.clientWidth, viewport.clientHeight);
    renderer.shadowMap.enabled = true;
    viewport.appendChild(renderer.domElement);

    scene = new THREE.Scene();
    scene.fog = new THREE.FogExp2(0x0c1218, 0.0012);

    camera = new THREE.PerspectiveCamera(
      42,
      viewport.clientWidth / Math.max(1, viewport.clientHeight),
      0.1,
      2000
    );
    camera.position.set(140, 110, 160);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0, P.L * 0.45);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;

    const amb = new THREE.AmbientLight(0xb0c4d8, 0.55);
    scene.add(amb);
    const key = new THREE.DirectionalLight(0xffffff, 0.95);
    key.position.set(120, 200, 80);
    key.castShadow = true;
    scene.add(key);
    const fill = new THREE.DirectionalLight(0x88aacc, 0.35);
    fill.position.set(-100, 40, -60);
    scene.add(fill);

    // 地面网格
    const grid = new THREE.GridHelper(400, 20, 0x2a3b4d, 0x1a2530);
    grid.position.y = -P.R - 2;
    scene.add(grid);

    // 坐标轴小助手
    const axes = new THREE.AxesHelper(30);
    axes.position.set(-40, 0, 0);
    scene.add(axes);

    probe = G.createProbeGroup(THREE);
    // 居中：探头沿 Z，稍微抬高
    probe.group.position.set(0, 0, 0);
    scene.add(probe.group);

    window.addEventListener('resize', onResize);
    animate();
  }

  function onResize() {
    const viewport = $('viewport');
    const w = viewport.clientWidth;
    const h = Math.max(1, viewport.clientHeight);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  }

  function animate() {
    animId = requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  }

  function refreshPressure() {
    pressureCache = P.pressureField(state.pressureRegions);
    return pressureCache;
  }

  function updateSceneFromState() {
    const pf = refreshPressure();
    G.deformProbe(probe, THREE, state, pf.samples);
    const fwd = P.forward(state);
    state.lastForward = fwd;
    renderDataPanels(fwd, pf);
  }

  function renderDataPanels(fwd, pf) {
    const wires = fwd.wires;
    const maxAbs = Math.max(...wires.map((w) => Math.abs(w.dR_R)), 1e-9);
    for (let i = 0; i < 4; i++) {
      const el = $('wire' + i);
      if (!el) continue;
      const dR = wires[i].dR_R;
      el.querySelector('.val').textContent =
        'ΔR/R=' + dR.toFixed(5) + '  ε=' + wires[i].strain.toFixed(5);
      el.classList.toggle('hot', Math.abs(dR) > 0.55 * maxAbs && maxAbs > 1e-4);
    }

    $('pMax').textContent = pf.maxPressure.toFixed(3);
    $('pMean').textContent = pf.meanPressure.toFixed(3);
    $('pArea').textContent = pf.coverageArea.toFixed(1) + ' mm²';
    $('pCenter').textContent =
      '(' +
      pf.center.x.toFixed(1) +
      ', ' +
      pf.center.y.toFixed(1) +
      ', ' +
      pf.center.z.toFixed(1) +
      ')';

    const setTxt =
      '设定: 弯=' +
      state.bendAmt.toFixed(3) +
      '  θ=' +
      deg(state.theta).toFixed(1) +
      '°  φ=' +
      deg(state.twist).toFixed(1) +
      '°  pos=' +
      state.bendPos.toFixed(3);
    let invTxt = '逆解: （尚未运行）';
    if (state.lastInverse) {
      const s = state.lastInverse.state;
      invTxt =
        '逆解: 弯=' +
        s.bendAmt.toFixed(3) +
        '  θ=' +
        deg(s.theta).toFixed(1) +
        '°  φ=' +
        deg(s.twist).toFixed(1) +
        '°  pos=' +
        s.bendPos.toFixed(3) +
        '\n残差RMSE=' +
        state.lastInverse.residual.toExponential(3);
      if (state.mode === 'forward' || state._compareTrue) {
        const t = state._compareTrue || state;
        const eB = Math.abs(s.bendAmt - t.bendAmt);
        const eTh = Math.abs(P.angDiff(s.theta, t.theta));
        const eTw = Math.abs(s.twist - t.twist);
        invTxt +=
          '\n对比误差: Δ弯=' +
          eB.toFixed(4) +
          '  Δθ=' +
          deg(eTh).toFixed(2) +
          '°  Δφ=' +
          deg(eTw).toFixed(2) +
          '°';
      }
    }
    $('compareBox').textContent = setTxt + '\n' + invTxt;
  }

  function syncSlidersToState() {
    $('bendAmt').value = state.bendAmt;
    $('bendAmtVal').textContent = state.bendAmt.toFixed(3);
    $('theta').value = deg(state.theta);
    $('thetaVal').textContent = deg(state.theta).toFixed(1) + '°';
    $('twist').value = deg(state.twist);
    $('twistVal').textContent = deg(state.twist).toFixed(1) + '°';
    $('bendPos').value = state.bendPos;
    $('bendPosVal').textContent = state.bendPos.toFixed(3);
    $('pressS').value = state.pressureRegions[0].sNorm;
    $('pressSVal').textContent = state.pressureRegions[0].sNorm.toFixed(3);
    $('pressA').value = deg(state.pressureRegions[0].angle);
    $('pressAVal').textContent = deg(state.pressureRegions[0].angle).toFixed(0) + '°';
    $('pressPeak').value = state.pressureRegions[0].peak;
    $('pressPeakVal').textContent = state.pressureRegions[0].peak.toFixed(2);
    $('pressSigS').value = state.pressureRegions[0].sSigma;
    $('pressSigA').value = state.pressureRegions[0].aSigma;
  }

  function bindUI() {
    const bindRange = (id, valId, fn) => {
      $(id).addEventListener('input', (e) => {
        fn(parseFloat(e.target.value));
        if (valId) {
          /* updated in fn via sync or locally */
        }
        // 压力在两模式都刷新；形变滑条仅正向模式驱动场景
        const isPress = id.startsWith('press');
        if (state.mode === 'forward' || isPress) updateSceneFromState();
      });
    };

    bindRange('bendAmt', 'bendAmtVal', (v) => {
      state.bendAmt = v;
      $('bendAmtVal').textContent = v.toFixed(3);
    });
    bindRange('theta', 'thetaVal', (v) => {
      state.theta = rad(v);
      $('thetaVal').textContent = v.toFixed(1) + '°';
    });
    bindRange('twist', 'twistVal', (v) => {
      state.twist = rad(v);
      $('twistVal').textContent = v.toFixed(1) + '°';
    });
    bindRange('bendPos', 'bendPosVal', (v) => {
      state.bendPos = v;
      $('bendPosVal').textContent = v.toFixed(3);
    });

    bindRange('pressS', null, (v) => {
      state.pressureRegions[0].sNorm = v;
      $('pressSVal').textContent = v.toFixed(3);
    });
    bindRange('pressA', null, (v) => {
      state.pressureRegions[0].angle = rad(v);
      $('pressAVal').textContent = v.toFixed(0) + '°';
    });
    bindRange('pressPeak', null, (v) => {
      state.pressureRegions[0].peak = v;
      $('pressPeakVal').textContent = v.toFixed(2);
    });
    bindRange('pressSigS', null, (v) => {
      state.pressureRegions[0].sSigma = v;
    });
    bindRange('pressSigA', null, (v) => {
      state.pressureRegions[0].aSigma = rad(v);
    });

    $('btnForward').addEventListener('click', () => setMode('forward'));
    $('btnInverse').addEventListener('click', () => setMode('inverse'));

    $('btnReset').addEventListener('click', () => {
      state.bendAmt = 0;
      state.theta = 0;
      state.twist = 0;
      state.bendPos = 0.5;
      state.pressureRegions = [
        { sNorm: 0.55, angle: 0.4, sSigma: 0.07, aSigma: 0.45, peak: 0.8 },
      ];
      state.lastInverse = null;
      state._compareTrue = null;
      syncSlidersToState();
      setMode('forward');
      updateSceneFromState();
    });

    $('btnScreenshot').addEventListener('click', () => {
      renderer.render(scene, camera);
      const url = renderer.domElement.toDataURL('image/png');
      const a = document.createElement('a');
      a.href = url;
      a.download = 'silicone-probe-' + Date.now() + '.png';
      a.click();
    });

    $('btnSolve').addEventListener('click', runInverseFromInputs);
    $('btnFillFromForward').addEventListener('click', () => {
      const fwd = P.forward(state);
      for (let i = 0; i < 4; i++) {
        $('dR' + i).value = fwd.dR[i].toFixed(6);
      }
    });

    $('btnValidate').addEventListener('click', () => {
      $('validationOut').textContent = '正在运行验证套件…';
      setTimeout(() => {
        const summary = P.runValidation('ui');
        const lines = [];
        lines.push('验证用例 n=' + summary.n);
        lines.push(
          'RMSE 弯=' +
            summary.rmseBend.toFixed(4) +
            '  θ=' +
            deg(summary.rmseTheta).toFixed(2) +
            '°  φ=' +
            deg(summary.rmseTwist).toFixed(2) +
            '°'
        );
        lines.push(
          '平均|误差| 弯=' +
            summary.meanAbsBend.toFixed(4) +
            '  θ=' +
            deg(summary.meanAbsThetaRad).toFixed(2) +
            '°  φ=' +
            deg(summary.meanAbsTwistRad).toFixed(2) +
            '°  pos=' +
            summary.meanAbsPos.toFixed(4)
        );
        lines.push('残差均值=' + summary.meanResidual.toExponential(3));
        lines.push('—— 抽样 ——');
        summary.rows.slice(0, 6).forEach((r) => {
          lines.push(
            r.name +
              '  eB=' +
              r.eBend.toFixed(3) +
              ' eθ=' +
              deg(r.eTh).toFixed(1) +
              '° eφ=' +
              deg(r.eTw).toFixed(1) +
              '°'
          );
        });
        $('validationOut').textContent = lines.join('\n');
        console.log('[validation]', summary);
      }, 30);
    });

    $('btnDemoInverse').addEventListener('click', () => {
      // 设定真值 → 正向 → 逆解 → 对比
      const truth = {
        bendAmt: state.bendAmt,
        theta: state.theta,
        twist: state.twist,
        bendPos: state.bendPos,
      };
      const fwd = P.forward(truth);
      for (let i = 0; i < 4; i++) $('dR' + i).value = fwd.dR[i].toFixed(6);
      state._compareTrue = Object.assign({}, truth);
      setMode('inverse');
      runInverseFromInputs();
    });
  }

  function setMode(mode) {
    state.mode = mode;
    $('btnForward').classList.toggle('active', mode === 'forward');
    $('btnInverse').classList.toggle('active', mode === 'inverse');
    $('forwardControls').classList.toggle('hidden', mode !== 'forward');
    $('inverseControls').classList.toggle('hidden', mode !== 'inverse');
  }

  function runInverseFromInputs() {
    const dR = [0, 1, 2, 3].map((i) => parseFloat($('dR' + i).value) || 0);
    state.invDR = dR;
    const inv = P.inverse(dR);
    state.lastInverse = inv;
    state.bendAmt = inv.state.bendAmt;
    state.theta = inv.state.theta;
    state.twist = inv.state.twist;
    state.bendPos = inv.state.bendPos;
    syncSlidersToState();
    updateSceneFromState();
  }

  function boot() {
    initThree();
    bindUI();
    syncSlidersToState();
    setMode('forward');
    updateSceneFromState();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot);
  } else {
    boot();
  }
})();

import { createRequire } from 'module';
import { writeFileSync } from 'fs';
import { dirname, join } from 'path';
import { fileURLToPath } from 'url';

const require = createRequire(import.meta.url);
const __dirname = dirname(fileURLToPath(import.meta.url));
const P = require('./js/physics.js');

function fmt(s) {
  return {
    meanAbsBend: +s.meanAbsBend.toFixed(4),
    rmseBend: +s.rmseBend.toFixed(4),
    meanAbsThetaDeg: +((s.meanAbsThetaRad * 180) / Math.PI).toFixed(2),
    rmseThetaDeg: +((s.rmseTheta * 180) / Math.PI).toFixed(2),
    meanAbsTwistDeg: +((s.meanAbsTwistRad * 180) / Math.PI).toFixed(2),
    rmseTwistDeg: +((s.rmseTwist * 180) / Math.PI).toFixed(2),
    meanAbsPos: +s.meanAbsPos.toFixed(4),
    meanResidual: +s.meanResidual.toFixed(6),
  };
}

// Sanity signatures
P.setParams({ twistAsym: 0.02, twistCommon: 0.004, coupleBeta: 0.3, GF: 2.4, bendHalfWidth: 0.22 });
console.log('bend th=0', P.forward({ bendAmt: 0.5, theta: 0, twist: 0, bendPos: 0.5 }).dR.map((x) => +x.toFixed(5)));
console.log('bend th=PI', P.forward({ bendAmt: 0.5, theta: Math.PI, twist: 0, bendPos: 0.5 }).dR.map((x) => +x.toFixed(5)));
console.log('twist +0.8', P.forward({ bendAmt: 0, theta: 0, twist: 0.8, bendPos: 0.5 }).dR.map((x) => +x.toFixed(5)));
console.log('twist -0.8', P.forward({ bendAmt: 0, theta: 0, twist: -0.8, bendPos: 0.5 }).dR.map((x) => +x.toFixed(5)));

console.log('\n=== Round 1 baseline ===');
const r1 = P.runValidation('round1');
console.log(fmt(r1));
const worst = [...r1.rows].sort((a, b) => b.eBend + b.eTw + b.eTh * 0.3 - (a.eBend + a.eTw + a.eTh * 0.3)).slice(0, 8);
for (const w of worst) {
  console.log(
    w.name,
    'eB', w.eBend.toFixed(3),
    'eTh°', ((w.eTh * 180) / Math.PI).toFixed(1),
    'eTw°', ((w.eTw * 180) / Math.PI).toFixed(1),
    'ePos', w.ePos.toFixed(3),
    'est', {
      b: +w.est.bendAmt.toFixed(3),
      th: +w.est.theta.toFixed(3),
      tw: +w.est.twist.toFixed(3),
      p: +w.est.bendPos.toFixed(3),
    }
  );
}

console.log('\n=== Tuning grid ===');
let bestScore = Infinity;
let bestP = null;
for (const GF of [2.2, 2.4, 2.6]) {
  for (const twistAsym of [0.015, 0.02, 0.028]) {
    for (const coupleBeta of [0.2, 0.3, 0.4]) {
      for (const twistCommon of [0.002, 0.004, 0.008]) {
        for (const bendHalfWidth of [0.2, 0.22, 0.26]) {
          P.setParams({ GF, twistAsym, coupleBeta, twistCommon, bendHalfWidth });
          const s = P.runValidation('tune');
          const score =
            s.rmseBend * 4 +
            s.rmseTwist * 2.5 +
            s.rmseTheta * 0.8 +
            s.meanAbsPos * 0.5 +
            s.meanResidual * 20;
          if (score < bestScore) {
            bestScore = score;
            bestP = { GF, twistAsym, coupleBeta, twistCommon, bendHalfWidth, metrics: fmt(s), score };
          }
        }
      }
    }
  }
}
console.log('Best', bestP);

P.setParams({
  GF: bestP.GF,
  twistAsym: bestP.twistAsym,
  coupleBeta: bestP.coupleBeta,
  twistCommon: bestP.twistCommon,
  bendHalfWidth: bestP.bendHalfWidth,
});
console.log('\n=== Round 2 tuned ===');
const r2 = P.runValidation('round2');
console.log(fmt(r2));

const out = {
  round1: { params: r1.params, metrics: fmt(r1) },
  tunedParams: {
    GF: bestP.GF,
    twistAsym: bestP.twistAsym,
    coupleBeta: bestP.coupleBeta,
    twistCommon: bestP.twistCommon,
    bendHalfWidth: bestP.bendHalfWidth,
  },
  round2: { params: Object.assign({}, P.getParams()), metrics: fmt(r2) },
  failCountRound2: r2.rows.filter((r) => r.eBend > 0.08 || r.eTw > 0.25 || r.eTh > 0.35).length,
  samples: r2.rows.filter((_, i) => i % 5 === 0).map((r) => ({
    name: r.name,
    eBend: +r.eBend.toFixed(4),
    eThDeg: +((r.eTh * 180) / Math.PI).toFixed(2),
    eTwDeg: +((r.eTw * 180) / Math.PI).toFixed(2),
    ePos: +r.ePos.toFixed(4),
  })),
};
writeFileSync(join(__dirname, 'validation-result.json'), JSON.stringify(out, null, 2));
console.log('Wrote validation-result.json, fails', out.failCountRound2);

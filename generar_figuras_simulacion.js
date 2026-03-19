const fs = require('fs');
const path = require('path');

const outDir = path.join(process.cwd(), 'output', 'simulacion');
fs.mkdirSync(outDir, { recursive: true });

const dt = 0.5;
const tEnd = 1800;
const n = Math.floor(tEnd / dt) + 1;
const t = Array.from({ length: n }, (_, i) => i * dt);

const Kp = 0.02;
const tauP = 200;
const tauA = 5;
const tauS = 1;
const Kc = 150;
const Ti = 200;
const qNom = 20 / Kp;

const r = t.map(tt => (tt >= 10 ? 20 : 0));
const dNom = t.map(() => 0);
const dDist = t.map(tt => (tt >= 600 ? -6 : 0));

function zeros() {
  return Array.from({ length: n }, () => 0);
}

function calcMetrics(y) {
  const idx = t.findIndex(v => v >= 10);
  const rFinal = r[r.length - 1];
  const tail = y.slice(-40);
  const yss = tail.reduce((a, b) => a + b, 0) / tail.length;
  const ess = rFinal - yss;
  const peak = Math.max(...y.slice(idx));
  const Mp = Math.max(0, ((peak - rFinal) / Math.abs(rFinal)) * 100);
  const band = 0.02 * Math.abs(rFinal);
  let ts = null;
  for (let k = idx; k < n; k++) {
    let ok = true;
    for (let j = k; j < n; j++) {
      if (Math.abs(y[j] - rFinal) > band) {
        ok = false;
        break;
      }
    }
    if (ok) {
      ts = t[k] - 10;
      break;
    }
  }
  return { ess, Mp, ts };
}

function simulateOpen(d) {
  const out = { T: zeros(), Q: zeros() };
  for (let k = 0; k < n - 1; k++) {
    if (t[k] >= 10) out.Q[k] = qNom;
    const dT = (-out.T[k] + Kp * out.Q[k] + d[k]) / tauP;
    out.T[k + 1] = out.T[k] + dt * dT;
  }
  out.Q[n - 1] = out.Q[n - 2];
  return out;
}

function simulateLinearPI(d) {
  const out = { T: zeros(), Q: zeros() };
  let iLin = 0;
  for (let k = 0; k < n - 1; k++) {
    const e = r[k] - out.T[k];
    iLin += e * dt;
    out.Q[k] = Kc * (e + iLin / Ti);
    const dT = (-out.T[k] + Kp * out.Q[k] + d[k]) / tauP;
    out.T[k + 1] = out.T[k] + dt * dT;
  }
  out.Q[n - 1] = out.Q[n - 2];
  return out;
}

function simulateNonlinearPI(d, loadChange) {
  const out = { T: zeros(), Tm: zeros(), Qcmd: zeros(), Qact: zeros() };
  let iNl = 0;
  for (let k = 0; k < n - 1; k++) {
    const KpEff = loadChange && t[k] >= 600 ? 0.019 : 0.02;
    const tauEff = loadChange && t[k] >= 600 ? 230 : 200;
    const ySensor = out.Tm[k] + 0.0003 * out.Tm[k] * out.Tm[k];
    const e = r[k] - ySensor;
    const uUnsat = Kc * (e + iNl / Ti);
    const uSat = Math.max(0, Math.min(3000, uUnsat));
    if (Math.abs(uUnsat - uSat) < 1e-9) iNl += e * dt;
    out.Qcmd[k] = uSat;
    const dQ = (-out.Qact[k] + out.Qcmd[k]) / tauA;
    out.Qact[k + 1] = out.Qact[k] + dt * dQ;
    const dT = (-out.T[k] + KpEff * out.Qact[k] + d[k]) / tauEff;
    out.T[k + 1] = out.T[k] + dt * dT;
    const dTm = (-out.Tm[k] + out.T[k]) / tauS;
    out.Tm[k + 1] = out.Tm[k] + dt * dTm;
  }
  out.Qcmd[n - 1] = out.Qcmd[n - 2];
  return out;
}

const nominal = {
  open: simulateOpen(dNom),
  lin: simulateLinearPI(dNom),
  nl: simulateNonlinearPI(dNom, false),
};

const disturbed = {
  open: simulateOpen(dDist),
  lin: simulateLinearPI(dDist),
  nl: simulateNonlinearPI(dDist, true),
};

const metrics = {
  nominal: {
    open: calcMetrics(nominal.open.T),
    lin: calcMetrics(nominal.lin.T),
    nl: calcMetrics(nominal.nl.T),
  },
  disturbed_final_error: {
    open: 20 - disturbed.open.T.slice(-40).reduce((a, b) => a + b, 0) / 40,
    lin: 20 - disturbed.lin.T.slice(-40).reduce((a, b) => a + b, 0) / 40,
    nl: 20 - disturbed.nl.T.slice(-40).reduce((a, b) => a + b, 0) / 40,
  }
};

fs.writeFileSync(
  path.join(outDir, 'metricas_simulacion.json'),
  JSON.stringify(metrics, null, 2)
);

function svgPlot({
  filename,
  title,
  yLabel,
  series,
  yMin,
  yMax,
  markers = []
}) {
  const width = 1000;
  const height = 520;
  const margin = { left: 80, right: 20, top: 50, bottom: 55 };
  const plotW = width - margin.left - margin.right;
  const plotH = height - margin.top - margin.bottom;
  const xMin = 0;
  const xMax = tEnd;

  const sx = x => margin.left + ((x - xMin) / (xMax - xMin)) * plotW;
  const sy = y => margin.top + (1 - (y - yMin) / (yMax - yMin)) * plotH;

  function polyline(dataX, dataY) {
    return dataX.map((x, i) => `${sx(x).toFixed(2)},${sy(dataY[i]).toFixed(2)}`).join(' ');
  }

  const grid = [];
  for (let i = 0; i <= 6; i++) {
    const yy = yMin + (i * (yMax - yMin)) / 6;
    grid.push(`<line x1="${margin.left}" y1="${sy(yy)}" x2="${width - margin.right}" y2="${sy(yy)}" stroke="#d9d9d9" stroke-width="1"/>`);
    grid.push(`<text x="${margin.left - 10}" y="${sy(yy) + 4}" font-size="12" text-anchor="end" fill="#333">${yy.toFixed(1)}</text>`);
  }
  for (let i = 0; i <= 6; i++) {
    const xx = xMin + (i * (xMax - xMin)) / 6;
    grid.push(`<line x1="${sx(xx)}" y1="${margin.top}" x2="${sx(xx)}" y2="${height - margin.bottom}" stroke="#ececec" stroke-width="1"/>`);
    grid.push(`<text x="${sx(xx)}" y="${height - margin.bottom + 20}" font-size="12" text-anchor="middle" fill="#333">${xx.toFixed(0)}</text>`);
  }

  const legend = series.map((s, i) =>
    `<g transform="translate(${margin.left + i * 220},${22})">
      <line x1="0" y1="0" x2="28" y2="0" stroke="${s.color}" stroke-width="3" ${s.dash ? `stroke-dasharray="${s.dash}"` : ''}/>
      <text x="36" y="4" font-size="13" fill="#222">${s.name}</text>
    </g>`
  ).join('\n');

  const markerSvg = markers.map(m =>
    `<g>
      <line x1="${sx(m.x)}" y1="${margin.top}" x2="${sx(m.x)}" y2="${height - margin.bottom}" stroke="${m.color || '#777'}" stroke-width="1.5" stroke-dasharray="6 4"/>
      <text x="${sx(m.x) + 6}" y="${margin.top + 16}" font-size="12" fill="${m.color || '#555'}">${m.label}</text>
    </g>`
  ).join('\n');

  const svg = `<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="${width}" height="${height}" viewBox="0 0 ${width} ${height}">
  <rect width="100%" height="100%" fill="#ffffff"/>
  <text x="${width / 2}" y="28" text-anchor="middle" font-size="20" fill="#111" font-family="Arial">${title}</text>
  ${legend}
  ${grid.join('\n')}
  ${markerSvg}
  <rect x="${margin.left}" y="${margin.top}" width="${plotW}" height="${plotH}" fill="none" stroke="#222" stroke-width="1.2"/>
  ${series.map(s => `<polyline fill="none" stroke="${s.color}" stroke-width="3" ${s.dash ? `stroke-dasharray="${s.dash}"` : ''} points="${polyline(t, s.data)}"/>`).join('\n')}
  <text x="${width / 2}" y="${height - 10}" text-anchor="middle" font-size="14" fill="#111">Tiempo [s]</text>
  <text x="20" y="${height / 2}" text-anchor="middle" font-size="14" fill="#111" transform="rotate(-90,20,${height / 2})">${yLabel}</text>
</svg>`;

  fs.writeFileSync(path.join(outDir, filename), svg);
}

svgPlot({
  filename: 'respuesta_nominal_escalon.svg',
  title: 'Respuesta nominal al escalon',
  yLabel: 'Temperatura [°C]',
  yMin: -1,
  yMax: 24,
  markers: [{ x: 10, label: 'Escalon de referencia' }],
  series: [
    { name: 'Referencia', data: r, color: '#111111', dash: '8 6' },
    { name: 'Sin mejoras', data: nominal.open.T, color: '#c43c39' },
    { name: 'PI lineal', data: nominal.lin.T, color: '#2b6cb0' },
    { name: 'PI con no linealidades', data: nominal.nl.T, color: '#2f855a' },
  ]
});

svgPlot({
  filename: 'respuesta_con_perturbacion.svg',
  title: 'Respuesta con perturbacion y no linealidades',
  yLabel: 'Temperatura [°C]',
  yMin: -1,
  yMax: 24,
  markers: [{ x: 10, label: 'Escalon de referencia' }, { x: 600, label: 'Perturbacion de carga', color: '#aa4444' }],
  series: [
    { name: 'Referencia', data: r, color: '#111111', dash: '8 6' },
    { name: 'Sin mejoras', data: disturbed.open.T, color: '#c43c39' },
    { name: 'PI lineal', data: disturbed.lin.T, color: '#2b6cb0' },
    { name: 'PI con no linealidades', data: disturbed.nl.T, color: '#2f855a' },
  ]
});

svgPlot({
  filename: 'accion_de_control.svg',
  title: 'Accion de control comparativa',
  yLabel: 'Potencia / accion [W]',
  yMin: -50,
  yMax: 3200,
  markers: [{ x: 10, label: 'Escalon de referencia' }, { x: 600, label: 'Perturbacion de carga', color: '#aa4444' }],
  series: [
    { name: 'Sin mejoras', data: disturbed.open.Q, color: '#c43c39' },
    { name: 'PI lineal', data: disturbed.lin.Q, color: '#2b6cb0' },
    { name: 'PI no lineal - mando', data: disturbed.nl.Qcmd, color: '#805ad5' },
    { name: 'PI no lineal - actuador', data: disturbed.nl.Qact, color: '#2f855a' },
  ]
});

console.log(JSON.stringify(metrics, null, 2));

/* Cálculos del ensayo, sin interfaz. Torque en kg·cm, corriente en A, masa en g. */
'use strict';
const Calc = (() => {
  const CPR = 2800, KGCM = 0.0980665, G = 9.80665;
  const T = [0, 12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228, 2.201, 2.179, 2.160, 2.145, 2.131,
    2.120, 2.110, 2.101, 2.093, 2.086, 2.080, 2.074, 2.069, 2.064, 2.060, 2.056, 2.052, 2.048, 2.045, 2.042];
  const tcrit = df => df < 1 ? NaN : df <= 30 ? T[df] : 1.96 + 2.4 / df;
  const mean = a => a.length ? a.reduce((s, v) => s + v, 0) / a.length : NaN;
  const sd = a => a.length < 2 ? NaN : Math.sqrt(a.reduce((s, v) => s + (v - mean(a)) ** 2, 0) / (a.length - 1));
  const median = a => { const b = [...a].sort((x, y) => x - y), n = b.length; return n ? (n % 2 ? b[(n - 1) / 2] : (b[n / 2 - 1] + b[n / 2]) / 2) : NaN; };
  const fin = v => typeof v === 'number' && Number.isFinite(v);

  /* y = m x + c por mínimos cuadrados; at(x) da valor e intervalo del 95 % de la recta. */
  function fit(x, y) {
    const n = x.length, mx = mean(x), my = mean(y);
    const sxx = x.reduce((s, v) => s + (v - mx) ** 2, 0);
    if (n < 2 || sxx < 1e-12) return null;
    const m = x.reduce((s, v, i) => s + (v - mx) * (y[i] - my), 0) / sxx, c = my - m * mx;
    const sse = y.reduce((s, v, i) => s + (v - m * x[i] - c) ** 2, 0);
    const sst = y.reduce((s, v) => s + (v - my) ** 2, 0);
    const s = n > 2 ? Math.sqrt(sse / (n - 2)) : NaN;
    return { n, m, c, s, r2: sst > 0 ? 1 - sse / sst : NaN,
      at: x0 => ({ y: m * x0 + c, u: tcrit(n - 2) * s * Math.sqrt(1 / n + (x0 - mx) ** 2 / sxx) }) };
  }
  const torque = (gramos, brazoCm) => gramos / 1000 * brazoCm;
  const usable = r => r.ok !== false && fin(r.V) && fin(r.m) && r.m > 0;

  /* Ensayo A, un sentido. rows: {V, I, m, ok}; devuelve null si faltan datos. */
  function ensayoA(rows, rCm) {
    const u = rows.filter(usable);
    if (new Set(u.map(r => r.Vn)).size < 3 || u.length < 4 || !(rCm > 0)) return null;
    const f = fit(u.map(r => r.V), u.map(r => torque(r.m, rCm)));
    const wi = u.filter(r => fin(r.I) && r.I > 0);
    const fi = wi.length >= 4 ? fit(wi.map(r => r.I), wi.map(r => torque(r.m, rCm))) : null;
    const e = f.at(12);
    return { f, fi, tau12: e.y, u12: e.u, rows: u };
  }
  /* Pulso: promedio de la meseta. log: {t (s desde el inicio), mode, mA, g} */
  function meseta(log, dur) {
    const w = log.filter(p => p.t >= 0.5 && p.t <= dur - 0.05 && p.mode === 'PULSE');
    const ma = w.map(p => p.mA).filter(v => fin(v) && v >= 0), g = w.map(p => p.g).filter(fin);
    return { I: ma.length ? mean(ma) / 1000 : NaN, g: g.length ? mean(g) : NaN, n: w.length };
  }
  const cuentas = (metros, rCm) => metros / (2 * Math.PI * rCm / 100) * CPR;
  /* Subida entre 0,20 m y 1,00 m: velocidad por regresión y control de mitades. */
  function subida(log, rCm) {
    const c0 = log[0].counts, a = cuentas(0.2, rCm), b = cuentas(1.0, rCm);
    const w = log.filter(p => p.mode === 'LIFT').map(p => ({ t: p.t, c: Math.abs(p.counts - c0), mA: p.mA })).filter(p => p.c >= a && p.c <= b);
    if (w.length < 10) return null;
    const v = q => Math.abs(fit(q.map(p => p.t), q.map(p => p.c)).m), h = w.length >> 1;
    const cs = v(w), v1 = v(w.slice(0, h)), v2 = v(w.slice(h));
    const ma = w.map(p => p.mA).filter(x => x >= 0);
    return { rpm: cs / CPR * 60, dif: Math.abs(v2 - v1) / cs, I: ma.length ? mean(ma) / 1000 : NaN, recorrido: w[w.length - 1].c / b };
  }
  function pesa(r, rCm) {
    const tau = torque(r.m, rCm), w = r.rpm * 2 * Math.PI / 60, po = tau * KGCM * w, pe = r.V * r.I;
    return { tau, po, pe, eta: pe > 0 ? po / pe : NaN };
  }
  function ensayoB(rows, rCm, rpm0, I0) {
    const u = rows.filter(r => r.ok !== false && fin(r.m) && fin(r.rpm) && r.rpm > 0);
    const x = u.map(r => torque(r.m, rCm)), y = u.map(r => r.rpm);
    if (fin(rpm0)) { x.push(0); y.push(rpm0); }
    if (new Set(x.map(v => v.toFixed(4))).size < 3) return null;
    const f = fit(x, y), wi = u.filter(r => fin(r.I) && r.I > 0);
    const xi = wi.map(r => torque(r.m, rCm)), yi = wi.map(r => r.I);
    if (fin(I0)) { xi.push(0); yi.push(I0); }
    return { f, fi: new Set(xi.map(v => v.toFixed(4))).size >= 3 ? fit(xi, yi) : null, stall: f.m < 0 ? -f.c / f.m : NaN };
  }
  /* Techo físico sin pérdidas a 12 V. */
  function techo(rpm0, R, I0 = 0) {
    const ke = (12 - I0 * R) / (rpm0 * 100 * 2 * Math.PI / 60);
    return ke * (12 / R - I0) * 100 / KGCM;
  }
  /* Ensayo C: lista {t s, T °C, m g} de un punto. */
  function sostener(list, rCm, razon) {
    const t = list.filter(p => fin(p.T)), mm = list.filter(p => fin(p.m));
    const ult = t.length ? t[t.length - 1] : null;
    const antes = ult ? [...t].reverse().find(p => ult.t - p.t >= 110) : null;
    const Tmax = t.length ? Math.max(...t.map(p => p.T)) : NaN;
    const sube = antes ? ult.T - antes.T : NaN;
    const cae = mm.length > 1 ? 1 - mm[mm.length - 1].m / mm[0].m : NaN;
    const completo = razon === 'OK' && ult && ult.t >= 560;
    return { Tmax, sube, cae, tau: mm.length ? torque(mm[mm.length - 1].m, rCm) : NaN, completo,
      ok: !!completo && Tmax < 60 && sube < 1 && cae < 0.10 };
  }
  const requerido = (M, f, d) => M * f * d;
  return { CPR, KGCM, G, tcrit, mean, sd, median, fin, fit, torque, ensayoA, meseta, cuentas, subida, pesa, ensayoB, techo, sostener, requerido };
})();

'use strict';
const $ = id => document.getElementById(id), { fin, mean } = Calc;
const esc = s => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const f = (v, n = 2) => fin(v) ? v.toFixed(n).replace('.', ',') : '—';
const num = s => { const v = parseFloat(String(s).replace(',', '.')); return Number.isFinite(v) ? v : null; };
const sleep = ms => new Promise(r => setTimeout(r, ms));
const AZ = '#1f5fa8', NA = '#d9730d', RO = '#b3261e', VE = '#2e7d4f';
let S, dev = null, demo = false, rec = null, hist = [], last = null, waiter = null, busy = false, dir = null, cool = 0, hold = null, hxZero = null, idOk = false;

/* ------------------------------------------------------------- sesión y registro */
const CAMPOS = [['motor', 'Motor n.º', 'text', '01'], ['canal', 'Canal MOD_DRV (1 = J3A, 2 = J3B, 3 = J3C)', 'number', 1], ['vbus', 'Vbus medido (V)', 'number', 12], ['tamb', 'T ambiente (°C)', 'number', 25],
  ['r', 'Brazo de palanca (mm)', 'number', 100], ['rp', 'Radio de polea (mm)', 'number', 15], ['dh', 'Diámetro del hilo (mm)', 'number', 1], ['rpt', 'R de las puntas (Ω)', 'number', 0.2], ['nota', 'Notas', 'text', '']];
function nueva(prep) {
  const d = new Date(), p = n => String(n).padStart(2, '0');
  S = { version: 1, inicio: d.toISOString(), id: '', simulado: demo, prep: {}, R: [null, null, null, null, null, null], vacio: { mas: null, menos: null, I0: null }, cal: 1, hx: null, A: [], B: [], C: [], n: 0, M: 1 };
  for (const [k, , , v] of CAMPOS) S.prep[k] = prep && prep[k] != null ? prep[k] : v;
  S.id = `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
}
const carpetaSesion = () => `${S.id}_motor${String(S.prep.motor).replace(/[^\w-]/g, '') || 'X'}${S.simulado ? '_SIMULADO' : ''}`;
async function escribir(nombre, texto) {
  if (dir) {
    try {
      const sub = await dir.getDirectoryHandle(S.carpeta || (S.carpeta = carpetaSesion()), { create: true });
      const w = await (await sub.getFileHandle(nombre, { create: true })).createWritable();
      await w.write(texto); await w.close(); return true;
    } catch (e) { aviso('No se pudo escribir en la carpeta: ' + e.message + '. Vuelve a elegirla.', 'w'); dir = null; pintaReg(); }
  }
  return false;
}
function descargar(nombre, texto) {
  const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([texto])); a.download = nombre; a.click(); setTimeout(() => URL.revokeObjectURL(a.href), 2000);
}
function resumenCSV() {
  const L = ['ensayo,n,sentido,nominal,V,V_medido,I_A,I_medida,masa_g,torque_kgcm,rpm,eta,valido,archivo'], q = v => v == null || Number.isNaN(v) ? '' : v;
  for (const r of S.A) L.push(['A', r.n, r.dir, r.Vn, q(r.V), !!r.Vm, q(r.I), !!r.Im, q(r.m), q(fin(r.m) ? Calc.torque(r.m, rCm()) : null), '', '', r.ok, r.archivo].join());
  for (const r of S.B) { const p = Calc.pesa(r, refCm()); L.push(['B', r.n, r.dir, r.m, q(r.V), !!r.Vm, q(r.I), !!r.Im, r.m, q(p.tau), q(r.rpm), q(p.eta), r.ok, r.archivo].join()); }
  for (const r of S.C) for (const p of r.puntos) L.push(['C', r.n, r.dir, r.tau, '', '', r.I, '', q(p.m), q(fin(p.m) ? Calc.torque(p.m, rCm()) : null), '', '', '', `t=${p.t}s T=${q(p.T)}C ${r.razon || ''}`].join());
  return '﻿' + L.join('\n') + '\n';
}
let guardando = Promise.resolve();
function guardar() {
  try { localStorage.setItem('ensayo_torque', JSON.stringify(S)); } catch (e) { /* respaldo opcional */ }
  if (dir) guardando = guardando.then(async () => { await escribir('sesion.json', JSON.stringify(S, null, 1)); await escribir('resumen.csv', resumenCSV()); });
  return guardando;
}
const pintaReg = () => { $('reg').textContent = dir ? 'registro: ' + dir.name + '/' : 'registro: sin carpeta'; };
async function telemetria(nombre, log) {
  const t = 't_s,modo,duty_milesimas,cuentas,corriente_mA,masa_g\n' + log.map(p => [p.t.toFixed(3), p.mode, p.duty, p.counts, p.mA, fin(p.g) ? p.g.toFixed(2) : ''].join()).join('\n') + '\n';
  if (!await escribir(nombre, t) && !demo) descargar(`${S.id}_${nombre}`, t);
}

/* ------------------------------------------------------------- comunicación */
const aviso = (t, c = '') => { $('msg').innerHTML = t ? `<div class="n ${c}">${esc(t)}</div>` : ''; if (t) traza(t); };
function traza(t) { const l = $('log'); l.textContent += `${new Date().toLocaleTimeString()}  ${t}\n`; if (l.textContent.length > 20000) l.textContent = l.textContent.slice(-12000); l.scrollTop = 1e9; }
function linea(s) {
  if (s.startsWith('T,')) {
    const a = s.split(','); if (a.length !== 10) return;
    const p = { ms: +a[1], motor: +a[2], mode: a[3], duty: +a[4], counts: +a[5], mA: +a[6], hx: a[7] === 'x' ? NaN : +a[7], seq: +a[8], nf: a[9] === '1' };
    if (![p.ms, p.counts, p.mA].every(Number.isFinite)) return;
    p.g = S.hx && fin(p.hx) && hxZero != null ? (p.hx - hxZero) / S.hx : NaN;
    last = p; last.rx = performance.now(); hist.push(p); if (hist.length > 1000) hist.shift();
    if (rec) rec.push(p);
    return;
  }
  traza(s);
  if (s.startsWith('ID,')) { idOk = s.startsWith('ID,ensayo_torque_N20,'); if (!idOk) aviso('La placa no tiene este firmware: ' + s, 'w'); }
  if (s.startsWith('BOOT,ensayo_torque_N20')) configurar();
  if (waiter && (s.startsWith('E,') || s.startsWith('ERR,'))) { const w = waiter; waiter = null; w(s); }
}
const enviar = s => dev ? dev.write(s + '\n') : Promise.resolve();
async function configurar() { if (!dev) return; await enviar('ID'); await enviar('SEL,' + (S.prep.canal || 1)); await enviar('CAL,' + Math.round(S.cal * 1000)); await enviar('HX,' + ($('hx').checked ? 1 : 0)); }
async function conectar() {
  if (dev) { await dev.close(); dev = null; if (demo) { demo = false; $('hx').checked = false; hxZero = null; } return estado(); }
  if (!navigator.serial) return aviso('Este navegador no tiene Web Serial. Usa Chrome o Edge.', 'w');
  try {
    const port = await navigator.serial.requestPort(); await port.open({ baudRate: 460800, bufferSize: 16384 });
    const wr = port.writable.getWriter(), rd = port.readable.getReader(), enc = new TextEncoder(), dec = new TextDecoder();
    let viva = true, buf = '';
    dev = { write: s => wr.write(enc.encode(s)).catch(() => 0), close: async () => { try { await wr.write(enc.encode('STOP\n')); } catch (e) { /* ya desconectado */ } viva = false; try { await rd.cancel(); rd.releaseLock(); wr.releaseLock(); await port.close(); } catch (e) { /* ya cerrado */ } } };
    (async () => {
      try { while (viva) { const { value, done } = await rd.read(); if (done) break; buf += dec.decode(value, { stream: true }); let k; while ((k = buf.indexOf('\n')) >= 0) { linea(buf.slice(0, k).trim()); buf = buf.slice(k + 1); } } } catch (e) { traza('Puerto: ' + e.message); }
      if (viva) { dev = null; if (waiter) { const w = waiter; waiter = null; w('E,DESCONECTADO,0,0'); } aviso('Se perdió la conexión.', 'w'); estado(); }
    })();
    if (S.simulado) { nueva(S.prep); pintaTodo(); }
    idOk = false; aviso(''); estado(); await sleep(1200); await configurar();
    setTimeout(() => { if (dev && !demo && !idOk) aviso('La placa no responde como ensayo_torque_N20. ¿Está grabado este firmware?', 'w'); }, 1500);
  } catch (e) { if (e.name !== 'NotFoundError') aviso('No se pudo abrir el puerto: ' + e.message + '. Cierra otros paneles o el monitor serie.', 'w'); dev = null; estado(); }
}
function estado() {
  const ok = dev && (demo || idOk);
  $('con').textContent = dev ? 'Desconectar' : 'Conectar'; $('demo').disabled = !!dev; $('sim').hidden = !S.simulado;
  const vivo = last && performance.now() - last.rx < 500;
  $('est').textContent = !dev ? 'sin conexión' : !ok ? 'esperando firmware' : !vivo ? 'sin datos' : last.nf ? 'nFAULT' : last.mode;
  document.querySelectorAll('.act').forEach(b => { b.disabled = !ok || !vivo || busy || !!hold || (b.id === 'aGo' || b.id === 'bGo' ? Date.now() < cool : false); });
  const w = Math.max(0, Math.ceil((cool - Date.now()) / 1000)); $('aW').textContent = $('bW').textContent = w ? `enfriando ${w} s` : '';
}
/* Ejecuta un ensayo y devuelve {razon, log, dur}. log: t en s desde el arranque. */
async function ensayo(cmd, previo) {
  if (busy || !dev) return null;
  busy = true; rec = []; estado();
  try {
    await sleep(500); if (previo) previo(rec);
    const antes = rec.length;
    const fin_ = new Promise(r => { waiter = r; }); await enviar(cmd);
    const s = await Promise.race([fin_, sleep(910000).then(() => 'E,SIN_RESPUESTA,0,0')]);
    waiter = null; await sleep(400);
    const a = s.split(','), datos = rec; rec = null;
    if (a[0] === 'ERR') { aviso('La placa rechazó la orden: ' + s, 'w'); return null; }
    const i0 = datos.findIndex((p, i) => i >= antes - 1 && p.mode !== 'IDLE' && p.mode !== 'BRAKE');
    const t0 = i0 >= 0 ? datos[i0].ms : datos.length ? datos[0].ms : 0;
    const razones = { SOBRECORRIENTE: 'corriente por encima del límite', BLOQUEADO: 'el eje no giró', NFAULT: 'fallo del driver', SIN_PANEL: 'se perdió el panel', SIN_CORRIENTE: 'sensado de corriente inválido', TIEMPO: 'no llegó a la marca a tiempo', STOP: 'detenido', LIMITE_VOLTAJE: 'no alcanzó la corriente pedida' };
    aviso(a[1] === 'OK' ? '' : 'Ensayo interrumpido: ' + (razones[a[1]] || a[1]) + '.', a[1] === 'OK' ? '' : 'w');
    return { razon: a[1], dur: +a[2] / 1000, cuentas: +a[3], log: datos.map(p => ({ ...p, t: (p.ms - t0) / 1000 })) };
  } finally { busy = false; rec = null; estado(); }
}
function parar() { enviar('STOP'); }

/* ------------------------------------------------------------- ESP32 simulado */
function simulador() {
  const m = { R: 16, a: 0.1167, b: 0.10, ts: 1.3, n0: 300 }; let ms = 0, mode = 'IDLE', d = 1, duty = 0, target = 0, counts = 0, start = 0, c0 = 0, lim = 0, tc = 0, ma = 0, hcur = 0, hx = false, seq = 0, hb = 0, w = 0;
  const masa = () => num($('bM').value) || 0, r = () => (Math.random() - .5);
  const fin_ = (z, b) => { if (mode !== 'IDLE' && mode !== 'BRAKE') linea(`E,${z},${ms - start},${Math.round(counts - c0)}`); mode = b ? 'BRAKE' : 'IDLE'; duty = 0; w = 0; };
  const go = (M, dd, l) => { mode = M; d = dd; start = ms; c0 = counts; lim = l; hcur = 0; linea('ACK,' + M); };
  const id = setInterval(() => {
    for (let k = 0; k < 5; k++) {
      ms += 10; let i = 0, g = 0; const V = duty * 12 / 1000, t = ms - start;
      if (mode === 'PULSE') { duty = target; i = duty * .012 / m.R; g = Math.max(0, m.a * V - m.b) * 100 + r() * 1.2; }
      if (mode === 'RUN' || mode === 'LIFT') { duty = Math.min(target, target * t / 300); const tau = mode === 'LIFT' ? masa() / 1000 * refCm() : 0; const wf = Math.max(0, m.n0 * (V / 12 - tau / m.ts)); w += (wf - w) * .08; i = .05 + .7 * tau / m.ts; counts += d * w / 60 * 2800 * .01 * (1 + r() * .01); }
      if (mode === 'HOLDI') { hcur += (target - hcur) * .1; i = hcur / 1000; duty = Math.round(i * m.R / 12 * 1000); g = Math.max(0, m.a * i * m.R - m.b) * 100 + r(); }
      ma = i * 1000 + r() * 2; if (hx) seq++;
      linea(`T,${ms},1,${mode},${d * Math.round(duty)},${Math.round(counts)},${Math.max(0, ma).toFixed(1)},${hx ? Math.round(84000 + g * 420) : 'x'},${seq},0`);
      if (mode !== 'IDLE' && mode !== 'BRAKE') {
        if (mode === 'LIFT' && Math.abs(counts - c0) >= tc) fin_('OK', true); else if (t >= lim) fin_(mode === 'LIFT' ? 'TIEMPO' : 'OK', mode === 'LIFT');
        else if (mode === 'LIFT' && t > 900 && Math.abs(counts - c0) < 28) fin_('BLOQUEADO'); else if (ms - hb > 60000) fin_('SIN_PANEL');
      }
    }
  }, 50);
  return { close: async () => clearInterval(id), write: async s => {
    hb = ms; const a = s.trim().split(','), n = a.slice(1).map(Number);
    if (a[0] === 'STOP') fin_('STOP'); else if (a[0] === 'ID') linea('ID,ensayo_torque_N20,sim'); else if (a[0] === 'HX') hx = n[0] === 1;
    else if (a[0] === 'RUN') { target = n[1]; go('RUN', n[0], n[2]); } else if (a[0] === 'PULSE') { target = n[1]; go('PULSE', n[0], n[2]); }
    else if (a[0] === 'LIFT') { target = n[1]; tc = n[2]; go('LIFT', n[0], 15000); } else if (a[0] === 'HOLDI') { target = n[1]; go('HOLDI', n[0], n[2] * 1000); }
  } };
}
function iniciarDemo() { if ((S.A.length || S.B.length || S.C.length) && !S.simulado && !confirm('La demostración abre una sesión nueva. ¿Continuar?')) return; demo = true; nueva(S.prep); hist = []; idOk = true; $('hx').checked = true; S.hx = 420; hxZero = 84000; dev = simulador(); configurar(); pintaTodo(); estado(); }

/* ------------------------------------------------------------- gráficos */
function plot(el, o) {
  const W = 520, H = 270, L = 58, B = 44, Tp = 14, Rr = 14, pts = o.series.flatMap(s => s.pts.map(p => [p[0], p[1]])).filter(p => fin(p[0]) && fin(p[1]));
  if (!pts.length) { el.innerHTML = '<p>Sin datos todavía.</p>'; return; }
  const ext = (k, a) => { let lo = a[0] ?? Math.min(...pts.map(p => p[k])), hi = a[1] ?? Math.max(...pts.map(p => p[k])); if (hi - lo < 1e-9) { hi = lo + 1; } if (a[1] == null) hi += (hi - lo) * .06; return [lo, hi]; };
  const [x0, x1] = ext(0, o.x || []), [y0, y1] = ext(1, o.y || []), X = v => L + (v - x0) / (x1 - x0) * (W - L - Rr), Y = v => H - B - (v - y0) / (y1 - y0) * (H - B - Tp);
  const ticks = (a, b) => { const r = (b - a) / 5, p = 10 ** Math.floor(Math.log10(r)), s = [1, 2, 2.5, 5, 10].map(k => k * p).find(k => k >= r), q = []; for (let v = Math.ceil(a / s - 1e-9) * s; v <= b + 1e-9; v += s) q.push(Math.abs(v) < 1e-12 ? 0 : v); return q; };
  const lab = v => String(+v.toPrecision(4)).replace('.', ',');
  let h = `<svg viewBox="0 0 ${W} ${H}" font-size="12" fill="#5b6470" role="img" aria-label="${esc(o.yl)} contra ${esc(o.xl)}">`;
  for (const v of ticks(x0, x1)) h += `<line x1="${X(v)}" x2="${X(v)}" y1="${Tp}" y2="${H - B}" stroke="#e6e9ed"/><text x="${X(v)}" y="${H - B + 16}" text-anchor="middle">${lab(v)}</text>`;
  for (const v of ticks(y0, y1)) h += `<line x1="${L}" x2="${W - Rr}" y1="${Y(v)}" y2="${Y(v)}" stroke="#e6e9ed"/><text x="${L - 6}" y="${Y(v) + 4}" text-anchor="end">${lab(v)}</text>`;
  h += `<text x="${(L + W) / 2}" y="${H - 6}" text-anchor="middle">${esc(o.xl)}</text><text transform="translate(13 ${(H - B) / 2 + 8}) rotate(-90)" text-anchor="middle">${esc(o.yl)}</text>`;
  let ly = Tp + 12;
  for (const s of o.series) {
    const q = s.pts.filter(p => fin(p[0]) && fin(p[1])); if (!q.length) continue;
    if (s.line) h += `<polyline fill="none" stroke="${s.color}" stroke-width="2" ${s.dash ? 'stroke-dasharray="6 5"' : ''} points="${q.map(p => X(p[0]).toFixed(1) + ',' + Y(p[1]).toFixed(1)).join(' ')}"/>`;
    else for (const p of q) h += `<circle cx="${X(p[0])}" cy="${Y(p[1])}" r="${s.r || 4}" fill="${s.color}"><title>${f(p[0], 3)} ; ${f(p[1], 3)}</title></circle>`;
    if (s.name) { h += `<rect x="${L + 10}" y="${ly - 9}" width="10" height="10" fill="${s.color}"/><text x="${L + 25}" y="${ly}">${esc(s.name)}</text>`; ly += 16; }
  }
  el.innerHTML = h + '</svg>';
}
const rectas = (fi, a, b) => [[a, fi.at(a).y], [b, fi.at(b).y]];
function vivo() {
  estado(); if (!last) return;
  const h = hist.filter(p => p.ms > last.ms - 10000), t = p => (p.ms - last.ms) / 1000, rpm = [];
  for (let i = 10; i < h.length; i += 4) rpm.push([t(h[i]), (h[i].counts - h[i - 10].counts) / Calc.CPR / ((h[i].ms - h[i - 10].ms) / 60000)]);
  const n = rpm.length ? rpm[rpm.length - 1][1] : NaN;
  $('vivo').textContent = `${last.mode} · PWM ${f(last.duty / 10, 1)} % · ${last.mA >= 0 ? f(last.mA, 0) : '—'} mA (CS) · ${f(n, 0)} RPM · ${fin(last.g) ? f(last.g, 1) + ' g' : 'sin celda'}`;
  plot($('gI'), { xl: 'Tiempo (s)', yl: 'Corriente CS (mA)', x: [-10, 0], y: [0], series: [{ line: 1, color: AZ, pts: h.filter((p, i) => i % 3 === 0 && p.mA >= 0).map(p => [t(p), p.mA]) }] });
  plot($('gN'), { xl: 'Tiempo (s)', yl: 'Velocidad (RPM)', x: [-10, 0], series: [{ line: 1, color: NA, pts: rpm }] });
}

/* ------------------------------------------------------------- vistas */
const rCm = () => (num(S.prep.r) || NaN) / 10, refCm = () => ((num(S.prep.rp) || NaN) + (num(S.prep.dh) || 0) / 2) / 10;
const Rmot = () => { const a = S.R.filter(fin); return a.length >= 3 ? Calc.median(a) - (num(S.prep.rpt) || 0) : NaN; };
const rpm0 = () => { const a = [S.vacio.mas, S.vacio.menos].filter(fin); return a.length ? mean(a) : NaN; };
const resA = d => Calc.ensayoA(S.A.filter(r => r.dir === d), rCm());
const tau12 = () => { const a = [resA(1), resA(-1)].filter(Boolean); return a.length ? Math.min(...a.map(x => x.tau12)) : NaN; };
const TABS = { prep: '0 · Preparar', A: 'A · Bloqueo', B: 'B · Pesas', C: 'C · Sostener', res: 'Resultado', log: 'Mensajes' };
function tab(k) { for (const t in TABS) { $('s-' + t).classList.toggle('on', t === k); $('n-' + t).classList.toggle('on', t === k); } pintaTodo(); }
function celda(v, est, cb) { const i = document.createElement('input'); i.type = 'number'; i.step = 'any'; i.value = fin(v) ? +v.toFixed(3) : ''; if (est) i.style.color = '#8b96a3'; i.onchange = () => cb(num(i.value)); return i; }
function tabla(el, cab, filas) {
  el.innerHTML = '<tr>' + cab.map(c => `<th>${c}</th>`).join('') + '</tr>';
  for (const fi of filas) { const tr = el.insertRow(); for (const c of fi) { const td = tr.insertCell(); if (c instanceof Node) td.append(c); else td.textContent = c; } }
}
const cambio = fn => v => { fn(v); guardar(); pintaTodo(); };
const chk = r => { const c = document.createElement('input'); c.type = 'checkbox'; c.checked = r.ok !== false; c.onchange = () => cambio(() => { r.ok = c.checked; })(); return c; };

function pintaPrep() {
  for (const [k] of CAMPOS) if (document.activeElement !== $('p-' + k)) $('p-' + k).value = S.prep[k] ?? '';
  S.R.forEach((v, i) => { if (document.activeElement !== $('R' + i)) $('R' + i).value = v ?? ''; });
  if (document.activeElement !== $('I0')) $('I0').value = S.vacio.I0 ?? '';
  const R = Rmot();
  $('r-R').innerHTML = fin(R) ? `Mediana − puntas = <b>${f(R, 1)} Ω</b> → bloqueo a 12 V ≈ <b>${f(12 / R)} A</b>. ${R > 25 ? 'Motor de baja corriente: espera menos torque.' : R < 12 ? 'Resistencia baja: revisa las puntas.' : 'Dentro de lo esperado (~14–17 Ω para 0,70–0,85 A).'}` : 'Faltan al menos 3 lecturas.';
  const v = S.vacio, n = rpm0();
  $('r-V').innerHTML = `Sentido +: <b>${f(v.mas, 0)} RPM</b> · Sentido −: <b>${f(v.menos, 0)} RPM</b>. ${fin(n) && Math.abs(n / 300 - 1) > .2 ? 'Difiere más de 20 % de 300: revisa Vbus y que el eje esté libre.' : ''}`;
  $('r-hx').textContent = S.hx ? `escala ${f(S.hx, 1)} cuentas/g` : 'sin calibrar';
}
function pintaA() {
  const filas = S.A.map(r => [r.n, r.dir > 0 ? '+' : '−', f(r.Vn, 1), celda(r.V, !r.Vm, cambio(x => { r.V = x ?? r.Vn; r.Vm = x != null; })), celda(r.I, !r.Im, cambio(x => { r.I = x ?? r.Ics; r.Im = x != null; })), celda(r.m, false, cambio(x => { r.m = x; })), fin(r.m) ? f(Calc.torque(r.m, rCm()), 3) : 'falta masa', r.razon, chk(r)]);
  tabla($('tA'), ['n', 'Sent.', 'V nom.', 'V bornes', 'I (A)', 'Masa (g)', 'τ (kg·cm)', 'Fin', 'Válido'], filas);
  $('aPlan').innerHTML = 'Repeticiones válidas (meta 5): ' + [3, 4.5, 6, 7.5].map(v => `${f(v, 1)} V: <b>${S.A.filter(r => r.Vn === v && r.dir === +$('aD').value && r.ok !== false && fin(r.m)).length}/5</b>`).join(' · ');
  const ser = []; let h = '';
  for (const [d, col] of [[1, AZ], [-1, NA]]) {
    const a = S.A.filter(r => r.dir === d && r.ok !== false && fin(r.m)), z = resA(d); if (!a.length) continue;
    ser.push({ pts: a.map(r => [r.V, Calc.torque(r.m, rCm())]), color: col, name: 'sentido ' + (d > 0 ? '+' : '−') });
    if (z) { ser.push({ line: 1, dash: 1, color: col, pts: rectas(z.f, 0, 12.5) });
      h += `<p>Sentido ${d > 0 ? '+' : '−'} (${z.f.n} pulsos)</p><p class="big">${f(z.tau12)} ± ${f(z.u12)} kg·cm</p><p>τ = ${f(z.f.m, 4)}·V ${z.f.c < 0 ? '−' : '+'} ${f(Math.abs(z.f.c), 3)} · R² ${f(z.f.r2, 3)} · ${f(z.tau12 * Calc.KGCM, 3)} N·m a 12 V (extrapolado)</p>`;
      const te = Calc.techo(rpm0(), Rmot(), S.vacio.I0 ?? 0); if (fin(te)) h += z.tau12 > te ? `<div class="n w">Supera el techo físico sin pérdidas (${f(te)}): revisa brazo, tara y R.</div>` : `<p>Techo físico sin pérdidas: ${f(te)} kg·cm → rendimiento global ${f(100 * z.tau12 / te, 0)} %.</p>`;
    }
  }
  plot($('pA'), { xl: 'Voltaje en bornes (V)', yl: 'Torque (kg·cm)', x: [0, 13], series: ser });
  const im = S.A.filter(r => r.Im && fin(r.Ics) && r.Ics > .06 && r.I > 0 && r.ok !== false);
  if (im.length >= 3) h += `<div class="n">Amperímetro / CS = <b>${f(S.cal * mean(im.map(r => r.I)) / mean(im.map(r => r.Ics)), 3)}</b> (factor actual ${f(S.cal, 3)}). <button id="apCal">Aplicar factor</button></div>`; else h += '<p>Con 3 pulsos con corriente de amperímetro puedes calibrar el sensado de la placa.</p>';
  $('cA').innerHTML = h || 'Faltan datos: ≥ 4 pulsos con masa en ≥ 3 voltajes.';
  if ($('apCal')) $('apCal').onclick = async () => { const k = S.cal * mean(im.map(r => r.I)) / mean(im.map(r => r.Ics)); if (k < .5 || k > 2) return aviso('Factor fuera de 0,5–2: revisa el amperímetro y el montaje.', 'w'); if (busy || hold) return; for (const r of S.A) { if (fin(r.Ics)) { r.Ics *= k / S.cal; if (!r.Im) r.I = r.Ics; } } for (const r of S.B) if (fin(r.I) && !r.Im) r.I *= k / S.cal; S.cal = k; await enviar('CAL,' + Math.round(k * 1000)); guardar(); pintaTodo(); };
}
function pintaB() {
  const t = tau12(), mx = fin(t) ? 500 * t / refCm() : NaN;
  $('bMax').innerHTML = fin(mx) ? `Masa máxima (50 % del bloqueo de A): <b>${f(mx, 0)} g</b>. Sugeridas: ${[.25, .5, .75, 1].map(k => f(mx * k, 0)).join(' · ')} g.` : '<b>Haz primero el ensayo A</b>: de él sale la masa máxima.';
  tabla($('tB'), ['n', 'Sent.', 'm (g)', 'V bornes', 'I (A)', 'RPM', 'Dif. mitades', 'τ (kg·cm)', 'P sal (W)', 'P ent (W)', 'η', 'Fin', 'Válido'], S.B.map(r => { const p = Calc.pesa(r, refCm()); return [r.n, r.dir > 0 ? '+' : '−', f(r.m, 0), celda(r.V, !r.Vm, cambio(x => { r.V = x ?? r.Vn; r.Vm = x != null; })), celda(r.I, !r.Im, cambio(x => { r.I = x; r.Im = x != null; })), f(r.rpm, 1), fin(r.dif) ? f(r.dif * 100, 1) + ' %' : '—', f(p.tau, 3), f(p.po), f(p.pe), fin(p.eta) ? f(p.eta * 100, 0) + ' %' : '—', r.razon, chk(r)]; }));
  const u = S.B.filter(r => r.ok !== false && fin(r.rpm)), z = Calc.ensayoB(S.B, refCm(), rpm0(), S.vacio.I0), xs = u.map(r => Calc.torque(r.m, refCm()));
  const a = [{ color: NA, pts: u.map((r, i) => [xs[i], r.rpm]) }], b = [{ color: NA, pts: u.map((r, i) => [xs[i], r.I]) }];
  if (fin(rpm0())) a.push({ color: VE, name: 'vacío', pts: [[0, rpm0()]] });
  if (z) { a.push({ line: 1, color: AZ, pts: rectas(z.f, 0, Math.min(z.stall, 3)) }); if (z.fi) b.push({ line: 1, color: AZ, pts: rectas(z.fi, 0, Math.max(...xs)) }); }
  plot($('pB'), { xl: 'Torque (kg·cm)', yl: 'Velocidad (RPM)', x: [0], y: [0], series: a }); plot($('pBi'), { xl: 'Torque (kg·cm)', yl: 'Corriente (A)', x: [0], y: [0], series: b });
  let h = 'Faltan datos: vacío y al menos 2 masas (o 3 masas).';
  if (z) { h = `<p>n = ${f(z.f.c, 0)} − ${f(-z.f.m, 1)}·τ → bloqueo estimado</p><p class="big">${f(z.stall)} kg·cm</p>`; if (z.fi && z.fi.m > 0) h += `<p>k<sub>τ</sub> = ${f(1 / z.fi.m)} kg·cm/A</p>`; if (fin(t)) { const dd = Math.abs(z.stall - t) / t; h += `<div class="n ${dd < .15 ? 'ok' : 'w'}">Difiere ${f(dd * 100, 0)} % del ensayo A (${f(t)}). ${dd < .15 ? 'Coherentes.' : 'Revisa V en bornes, radio efectivo y calentamiento.'}</div>`; } }
  if (u.some(r => r.dif > .05)) h += '<div class="n w">Hay subidas con más de 5 % de diferencia entre mitades: no es velocidad constante. Márcalas como no válidas.</div>';
  $('cB').innerHTML = h;
}
function corrienteC() { const z = resA(+$('cD').value); return z && z.fi && z.fi.m > 0 ? (+$('cTau').value - z.fi.c) / z.fi.m : NaN; }
function pintaC() {
  const i = corrienteC(); if (fin(i) && document.activeElement !== $('cI') && !hold) $('cI').value = i.toFixed(3);
  $('cNota').innerHTML = `<div class="n">${fin(i) ? `De tu recta τ–I del ensayo A: <b>${f(i, 3)} A</b>.` : 'Sin recta τ–I: escribe la corriente a mano.'} ${Math.abs(S.cal - 1) < 1e-9 ? '<b>El sensado de corriente no está calibrado</b>; la corriente real puede diferir.' : ''}</div>`;
  const col = [VE, AZ, NA, RO];
  plot($('pC'), { xl: 'Tiempo (min)', yl: 'Carcasa (°C)', x: [0, 10], series: [...S.C.map((c, k) => ({ line: 1, color: col[k % 4], name: `${f(c.tau)} kg·cm (n ${c.n})`, pts: c.puntos.map(p => [p.t / 60, p.T]) })), ...(S.C.some(c => c.puntos.some(p => fin(p.T))) ? [{ line: 1, dash: 1, color: RO, pts: [[0, 60], [10, 60]] }] : [])] });
  $('cC').innerHTML = S.C.map(c => { const z = Calc.sostener(c.puntos, rCm(), c.razon); return `<div class="n ${z.ok ? 'ok' : ''}"><b>${f(c.tau)} kg·cm objetivo, ${f(c.I, 3)} A</b> · ${esc(c.razon || 'en curso')}<br>Torque medido ${f(z.tau)} · T máx ${f(z.Tmax, 1)} °C · últimos 2 min ${f(z.sube, 1)} °C · caída ${fin(z.cae) ? f(z.cae * 100, 0) + ' %' : '—'} · <b>${!c.razon ? '…' : z.ok ? 'SOSTENIBLE' : z.completo ? 'NO SOSTENIBLE' : 'INCOMPLETO / CORTADO'}</b></div>`; }).join('') || 'Sin puntos todavía.';
}
function pintaRes() {
  const a = resA(1), b = resA(-1), t = tau12(), z = Calc.ensayoB(S.B, refCm(), rpm0(), S.vacio.I0);
  const sos = S.C.map(c => Calc.sostener(c.puntos, rCm(), c.razon)).filter(x => x.ok).map(x => x.tau), cont = sos.length ? Math.max(...sos) : NaN;
  $('rRes').innerHTML = (S.simulado ? '<div class="n w"><b>DATOS SIMULADOS: no son una medición.</b></div>' : '') + `<table><tr><th>Motor</th><td>${esc(S.prep.motor)} · ${esc(S.id)}</td></tr><tr><th>Resistencia</th><td>${f(Rmot(), 1)} Ω</td></tr><tr><th>Vacío</th><td>${f(rpm0(), 0)} RPM · ${f(S.vacio.I0, 3)} A</td></tr><tr><th>Bloqueo A, sentido +</th><td>${a ? `${f(a.tau12)} ± ${f(a.u12)} kg·cm (extrapolado a 12 V)` : '—'}</td></tr><tr><th>Bloqueo A, sentido −</th><td>${b ? `${f(b.tau12)} ± ${f(b.u12)} kg·cm` : '—'}</td></tr><tr><th>Bloqueo estimado de B</th><td>${z ? f(z.stall) + ' kg·cm' : '—'}</td></tr><tr><th>Mayor torque sostenible (C)</th><td>${f(cont)} kg·cm</td></tr></table><p>El ± solo recoge la dispersión: no incluye brazo, calentamiento ni la extrapolación.</p>`;
  const M = S.M, casos = [['6 patas', 1 / 6], ['Trípode ideal', 1 / 3], ['Trípode, pata media', .414]];
  const ver = q => !fin(t) ? '—' : q > t ? 'NO: supera el bloqueo' : q > t / 2 ? 'forzado: margen < 2' : 'margen ≥ 2';
  const vc = q => fin(cont) ? (q <= cont ? 'sí' : 'no') : 'sin medir';
  $('rHex').innerHTML = `<table><tr><th>Caso</th><th>Postura</th><th>Requiere (kg·cm)</th><th>Frente al bloqueo</th><th>¿Sostenible?</th></tr>${casos.flatMap(([n, k]) => [['pie bajo la rodilla', 5.56], ['30°/70°', 8.77]].map(([p, d]) => { const q = Calc.requerido(M, k, d); return `<tr><td>${n}</td><td>${p}</td><td>${f(q)}</td><td>${ver(q)}</td><td>${vc(q)}</td></tr>`; })).join('')}</table><p>Torque estático del fémur, robot de ${f(M)} kg. No incluye fricción, impactos ni el límite mecánico de la caja.</p>`;
}
function pintaTodo() { pintaPrep(); pintaA(); pintaB(); pintaC(); pintaRes(); estado(); }

/* ------------------------------------------------------------- acciones */
async function vacio(d) {
  const z = await ensayo(`RUN,${d},1000,5000`); if (!z) return; await telemetria(`0_vacio_${d > 0 ? 'mas' : 'menos'}.csv`, z.log);
  const w = z.log.filter(p => p.mode === 'RUN' && p.t > 2.5); if (z.razon === 'OK' && w.length > 20) S.vacio[d > 0 ? 'mas' : 'menos'] = Math.abs(Calc.fit(w.map(p => p.t), w.map(p => p.counts)).m) / Calc.CPR * 60;
  guardar(); pintaTodo();
}
function taraAuto(datos) { const g = datos.map(p => p.hx).filter(fin); if (S.hx && g.length > 3) hxZero = mean(g); }
async function pulso() {
  const Vn = +$('aV').value, d = +$('aD').value, vb = num(S.prep.vbus), duty = Math.round(Vn / vb * 1000);
  if (!(vb > 0)) return aviso('Escribe el Vbus medido.', 'w'); if (duty > 650) return aviso(`Con Vbus = ${f(vb, 1)} V, ${f(Vn, 1)} V pide ${f(duty / 10, 0)} % y el máximo bloqueado es 65 %. Sube Vbus a 12 V o usa menos voltaje.`, 'w');
  const z = await ensayo(`PULSE,${d},${duty},${Math.round(+$('aT').value * 1000)},600`, taraAuto); if (!z) return;
  for (const p of z.log) if (S.hx && fin(p.hx) && hxZero != null) p.g = (p.hx - hxZero) / S.hx;
  const q = Calc.meseta(z.log, z.dur), n = ++S.n, archivo = `A_${String(n).padStart(3, '0')}_${Vn}V.csv`;
  S.A.push({ n, dir: d, Vn, V: duty / 1000 * vb, Vm: false, I: fin(q.I) ? q.I : null, Ics: fin(q.I) ? q.I : null, Im: false, m: fin(q.g) ? Math.abs(q.g) : null, razon: z.razon, ok: z.razon === 'OK', archivo, hora: new Date().toISOString() });
  cool = Date.now() + (demo ? 300 : Vn > 7 ? 120000 : 60000); await telemetria(archivo, z.log); guardar(); pintaTodo();
}
async function pesa() {
  const m = num($('bM').value), d = +$('bD').value, vb = num(S.prep.vbus), t = tau12(); if (!(m > 0) || !(refCm() > 0)) return aviso('Escribe la masa y el radio de polea.', 'w');
  if (!fin(t)) return aviso('Haz primero el ensayo A: de él sale la masa máxima.', 'w'); if (m / 1000 * refCm() > t / 2 * 1.02) return aviso(`Masa por encima del 50 % del bloqueo (${f(500 * t / refCm(), 0)} g).`, 'w');
  const tc = Math.round(Calc.cuentas(1.05, refCm())); if (tc > 40000) return aviso('Radio de polea demasiado pequeño para 1,05 m.', 'w');
  const z = await ensayo(`LIFT,${d},1000,${tc},600`); if (!z) return;
  const ini = z.log.findIndex(p => p.mode === 'LIFT'), q = ini >= 0 ? Calc.subida(z.log.slice(ini), refCm()) : null, n = ++S.n, archivo = `B_${String(n).padStart(3, '0')}_${m}g.csv`;
  S.B.push({ n, dir: d, m, V: vb, Vn: vb, Vm: false, I: q && fin(q.I) ? q.I : null, Im: false, rpm: q ? q.rpm : null, dif: q ? q.dif : null, razon: z.razon, ok: z.razon === 'OK' && !!q && q.dif < .05, archivo, hora: new Date().toISOString() });
  cool = Date.now() + (demo ? 300 : 60000); await telemetria(archivo, z.log); guardar(); pintaTodo();
}
async function sostener() {
  const I = num($('cI').value), d = +$('cD').value; if (!(I >= .05 && I <= .45)) return aviso('La corriente debe estar entre 0,05 y 0,45 A.', 'w');
  const n = ++S.n, c = { n, tau: +$('cTau').value, I, dir: d, puntos: [], razon: null, t0: Date.now(), archivo: `C_${String(n).padStart(3, '0')}.csv` }; S.C.push(c); hold = c;
  try { const z = await ensayo(`HOLDI,${d},${Math.round(I * 1000)},600`, taraAuto); c.razon = c.corte || (z ? z.razon : 'RECHAZADO'); if (z) { for (const p of z.log) if (S.hx && fin(p.hx) && hxZero != null) p.g = (p.hx - hxZero) / S.hx; await telemetria(c.archivo, z.log.filter((p, i) => i % 10 === 0)); } } finally { hold = null; delete c.t0; guardar(); pintaTodo(); }
}
function anotarC(T, m, t) {
  if (!hold) return aviso('No hay un punto en curso.', 'w'); if (!fin(T)) return aviso('Escribe la temperatura.', 'w');
  if (!fin(m) && last && fin(last.g)) m = Math.abs(last.g);
  hold.puntos.push({ t: t ?? Math.round((Date.now() - hold.t0 - 500) / 1000), T, m: fin(m) ? m : null });
  if (T >= 60) { hold.corte = 'CORTE_60C'; parar(); aviso('60 °C: ensayo cortado.', 'w'); }
  $('cT').value = $('cM').value = ''; guardar(); pintaC();
}

/* ------------------------------------------------------------- arranque */
nueva(); $('hx').checked = false;
for (const k in TABS) { const b = document.createElement('button'); b.id = 'n-' + k; b.textContent = TABS[k]; b.onclick = () => tab(k); $('nav').append(b); }
for (const [k, l, t] of CAMPOS) { const e = document.createElement('label'); e.textContent = l; const i = document.createElement('input'); i.id = 'p-' + k; i.type = t; i.step = 'any'; i.onchange = () => { S.prep[k] = t === 'number' ? num(i.value) : i.value; if (k === 'canal') { if (![1, 2, 3].includes(S.prep.canal)) S.prep.canal = 1; enviar('SEL,' + S.prep.canal); } guardar(); pintaTodo(); }; e.append(i); $('f-prep').append(e); }
S.R.forEach((_, j) => { const e = document.createElement('label'); e.textContent = `Lectura ${j + 1} (Ω)`; const i = document.createElement('input'); i.id = 'R' + j; i.type = 'number'; i.step = 'any'; i.onchange = () => { S.R[j] = num(i.value); guardar(); pintaTodo(); }; e.append(i); $('f-R').append(e); });
$('con').onclick = conectar; $('demo').onclick = iniciarDemo; $('stop').onclick = parar; $('bSoltar').onclick = parar;
document.addEventListener('keydown', e => { if (e.key === 'Escape') parar(); });
$('v1').onclick = () => vacio(1); $('v2').onclick = () => vacio(-1); $('aGo').onclick = pulso; $('bGo').onclick = pesa; $('cGo').onclick = sostener; $('cAdd').onclick = () => anotarC(num($('cT').value), num($('cM').value));
$('I0').onchange = () => { S.vacio.I0 = num($('I0').value); guardar(); pintaTodo(); }; $('M').onchange = () => { S.M = num($('M').value) || 1; guardar(); pintaTodo(); };
$('aD').onchange = pintaA; $('cTau').onchange = $('cD').onchange = pintaC;
$('hx').onchange = () => { if ($('hx').checked && !confirm('¿Retiraste el módulo CAN del zócalo J4? IO21 pasará a ser el reloj del HX711.')) { $('hx').checked = false; return; } enviar('HX,' + ($('hx').checked ? 1 : 0)); };
$('tara').onclick = () => { const g = hist.slice(-80).map(p => p.hx).filter(fin); if (g.length < 4) return aviso('No llegan datos del HX711.', 'w'); hxZero = mean(g); aviso(''); pintaPrep(); };
$('calhx').onclick = () => { const g = hist.slice(-80).map(p => p.hx).filter(fin), m = num($('patron').value); if (hxZero == null || g.length < 4 || !(m > 0)) return aviso('Primero tara, luego coloca la masa patrón.', 'w'); const k = (mean(g) - hxZero) / m; if (Math.abs(k) < 1) return aviso('La celda no cambió: revisa el montaje.', 'w'); S.hx = k; aviso(''); guardar(); pintaPrep(); };
$('carpeta').onclick = async () => { if (!window.showDirectoryPicker) return aviso('Este navegador no puede escribir en carpetas; usa Chrome o Edge.', 'w'); try { dir = await showDirectoryPicker({ mode: 'readwrite' }); pintaReg(); await guardar(); } catch (e) { if (e.name !== 'AbortError') aviso(e.message, 'w'); } };
$('nueva').onclick = () => { if (busy || hold) return aviso('Detén el ensayo antes de empezar otra sesión.', 'w'); if ((S.A.length || S.B.length || S.C.length) && !confirm('¿Empezar una sesión nueva? La actual queda en el registro si elegiste carpeta.')) return; const hx = S.hx, cal = S.cal; nueva(S.prep); S.hx = hx; S.cal = cal; pintaTodo(); };
$('abrir').onchange = async e => { try { if (busy || hold) throw new Error('hay un ensayo en curso'); const o = JSON.parse(await e.target.files[0].text()); if (o.version !== 1 || !Array.isArray(o.A) || !Array.isArray(o.B) || !Array.isArray(o.C)) throw new Error('no es una sesión válida'); if (o.simulado !== demo && dev) throw new Error('desconecta primero: no se mezclan datos simulados y reales'); S = o; configurar(); pintaTodo(); } catch (x) { aviso('No se pudo abrir: ' + x.message, 'w'); } e.target.value = ''; };
$('imp').onclick = () => print(); $('gua').onclick = async () => { await guardar(); if (!dir) { descargar(S.id + '_sesion.json', JSON.stringify(S, null, 1)); descargar(S.id + '_resumen.csv', resumenCSV()); } };
setInterval(() => { if (dev) enviar('HB'); }, 150); setInterval(vivo, 250);
setInterval(() => { if (hold) { const s = Math.max(0, Math.round((Date.now() - hold.t0) / 1000)), u = hold.puntos.length ? hold.puntos[hold.puntos.length - 1].t : -30; $('cW').textContent = `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}${s - u >= 30 ? ' · ANOTA T y masa' : ''}`; if (!demo && s - u > 90 && !hold.corte) { hold.corte = 'SIN_LECTURAS'; parar(); aviso('Cortado: 90 s sin anotar temperatura.', 'w'); } } else $('cW').textContent = ''; }, 500);
window.addEventListener('beforeunload', e => { if (busy || hold) { parar(); e.preventDefault(); } });
pintaReg(); tab('prep');

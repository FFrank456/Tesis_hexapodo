/* Gráficas de tiempo en canvas, rápidas y sincronizadas (cursor común, pausa, ventana móvil).
   Guardan hasta CAP muestras (100 Hz · 180 s). */
"use strict";
(function (global) {
const CAP = 18000;

class Store {
  constructor() { this.t = new Float64Array(CAP); this.s = {}; this.n = 0; this.head = 0; }
  push(t, vals) {
    const i = this.head; this.t[i] = t;
    for (const k in vals) { if (!this.s[k]) this.s[k] = new Float32Array(CAP).fill(NaN); this.s[k][i] = vals[k]; }
    for (const k in this.s) if (!(k in vals)) this.s[k][i] = NaN;
    this.head = (i + 1) % CAP; this.n = Math.min(CAP, this.n + 1);
  }
  clear() { this.n = 0; this.head = 0; for (const k in this.s) this.s[k].fill(NaN); }
  idx(j) { return (this.head - this.n + j + CAP) % CAP; }       // j = 0 … n-1 (antigua → nueva)
  last() { return this.n ? this.t[this.idx(this.n - 1)] : 0; }
  /* índice lógico de la primera muestra con t >= t0 */
  lower(t0) { let lo = 0, hi = this.n; while (lo < hi) { const m = (lo + hi) >> 1; if (this.t[this.idx(m)] < t0) lo = m + 1; else hi = m; } return lo; }
  valueAt(key, t) {
    if (!this.n || !this.s[key]) return NaN; let j = this.lower(t); if (j >= this.n) j = this.n - 1;
    return this.s[key][this.idx(j)];
  }
}

function niceTicks(lo, hi, n = 4) {
  const span = hi - lo || 1, raw = span / n, mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const step = [1, 2, 2.5, 5, 10].map(k => k * mag).find(s => s >= raw) || raw;
  const out = []; for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(+v.toFixed(10));
  return out;
}

class Charts {
  /* specs: [{canvas, title, unit, series:[{key,label,color,dash,width}], lines:[{y,color,label}], range:[lo,hi]}] */
  constructor(store, specs) {
    this.store = store; this.specs = specs; this.window = 20; this.paused = false; this.tEnd = 0; this.hoverT = null;
    for (const sp of specs) {
      sp.canvas.addEventListener("mousemove", e => {
        const r = sp.canvas.getBoundingClientRect(), x = (e.clientX - r.left) / r.width;
        const L = sp._plot; if (!L) return; const px = x * sp.canvas.width;
        if (px < L.x0 || px > L.x1) { this.hoverT = null; return; }
        this.hoverT = this.tEnd - this.window + (px - L.x0) / (L.x1 - L.x0) * this.window; this.hoverSpec = sp;
      });
      sp.canvas.addEventListener("mouseleave", () => { this.hoverT = null; });
    }
  }
  draw() {
    if (!this.paused) this.tEnd = this.store.last();
    for (const sp of this.specs) this._drawOne(sp);
  }
  _drawOne(sp) {
    const cv = sp.canvas, r = cv.getBoundingClientRect(), d = window.devicePixelRatio || 1;
    if (r.width < 2) return;
    cv.width = Math.round(r.width * d); cv.height = Math.round(r.height * d);
    const c = cv.getContext("2d"), W = cv.width, H = cv.height, st = this.store;
    c.clearRect(0, 0, W, H);
    const x0 = 46 * d, x1 = W - 8 * d, y0 = 20 * d, y1 = H - 18 * d;
    sp._plot = { x0, x1 };
    const t1 = this.tEnd, t0 = t1 - this.window, j0 = st.lower(t0);
    // rango
    let lo = Infinity, hi = -Infinity;
    for (const s of sp.series) { const a = st.s[s.key]; if (!a) continue;
      for (let j = j0; j < st.n; j++) { const v = a[st.idx(j)]; if (v === v) { if (v < lo) lo = v; if (v > hi) hi = v; } } }
    for (const l of sp.lines || []) { if (l.inRange) { lo = Math.min(lo, l.y); hi = Math.max(hi, l.y); } }
    if (sp.range) { lo = Math.min(lo, sp.range[0]); hi = Math.max(hi, sp.range[1]); }
    if (!isFinite(lo)) { lo = -1; hi = 1; }
    if (hi - lo < (sp.minSpan || 1e-3)) { const m = (hi + lo) / 2, h = (sp.minSpan || 1e-3) / 2; lo = m - h; hi = m + h; }
    const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;
    const X = t => x0 + (t - t0) / this.window * (x1 - x0), Y = v => y1 - (v - lo) / (hi - lo) * (y1 - y0);
    // ejes y rejilla
    c.font = `${10 * d}px system-ui, Segoe UI, sans-serif`; c.fillStyle = "#898781"; c.strokeStyle = "#ecebe6"; c.lineWidth = 1;
    for (const v of niceTicks(lo, hi)) { const y = Y(v); c.beginPath(); c.moveTo(x0, y); c.lineTo(x1, y); c.stroke();
      c.textAlign = "right"; c.fillText(Math.abs(v) < 1e-9 ? "0" : +v.toPrecision(3), x0 - 4 * d, y + 3 * d); }
    c.textAlign = "center";
    for (const v of niceTicks(t0, t1, 5)) { const x = X(v); if (x < x0 || x > x1) continue; c.fillText(v.toFixed(0) + " s", x, H - 4 * d); }
    c.strokeStyle = "#c3c2b7"; c.beginPath(); c.moveTo(x0, y0); c.lineTo(x0, y1); c.lineTo(x1, y1); c.stroke();
    // líneas de referencia (límites, saturación)
    for (const l of sp.lines || []) { if (l.y < lo || l.y > hi) continue; const y = Y(l.y);
      c.strokeStyle = l.color; c.setLineDash([5 * d, 4 * d]); c.beginPath(); c.moveTo(x0, y); c.lineTo(x1, y); c.stroke(); c.setLineDash([]);
      if (l.label) { c.fillStyle = l.color; c.textAlign = "left"; c.fillText(l.label, x0 + 4 * d, y - 3 * d); } }
    // series (decimación min/max por columna de píxel)
    c.save(); c.beginPath(); c.rect(x0, y0 - 2, x1 - x0, y1 - y0 + 4); c.clip();
    for (const s of sp.series) {
      const a = st.s[s.key]; if (!a) continue;
      c.strokeStyle = s.color; c.lineWidth = (s.width || 1.4) * d; c.setLineDash((s.dash || []).map(v => v * d));
      c.beginPath(); let pen = false, col = -1, mn = 0, mx = 0, lastY = 0;
      for (let j = Math.max(0, j0 - 1); j < st.n; j++) {
        const k = st.idx(j), v = a[k]; if (v !== v) { pen = false; continue; }
        const x = Math.round(X(st.t[k])), y = Y(v);
        if (pen && x === col) { if (y < mn) mn = y; if (y > mx) mx = y; lastY = y; continue; }
        if (pen && col >= 0 && mx - mn > 1) { c.lineTo(col, mn); c.lineTo(col, mx); c.lineTo(col, lastY); }
        pen ? c.lineTo(x, y) : c.moveTo(x, y); pen = true; col = x; mn = mx = lastY = y;
      }
      c.stroke(); c.setLineDash([]);
    }
    c.restore();
    // título y leyenda
    c.textAlign = "left"; c.fillStyle = "#0b0b0b"; c.font = `600 ${11 * d}px system-ui, Segoe UI, sans-serif`;
    c.fillText(sp.title, x0, 13 * d);
    let lx = x1; c.font = `${10 * d}px system-ui, Segoe UI, sans-serif`; c.textAlign = "right";
    for (const s of [...sp.series].reverse()) { const w = c.measureText(s.label).width;
      c.fillStyle = "#52514e"; c.fillText(s.label, lx, 13 * d); lx -= w + 4 * d;
      c.strokeStyle = s.color; c.lineWidth = 2 * d; c.setLineDash((s.dash || []).map(v => v * d));
      c.beginPath(); c.moveTo(lx - 14 * d, 9 * d); c.lineTo(lx, 9 * d); c.stroke(); c.setLineDash([]); lx -= 22 * d; }
    // cursor común
    if (this.hoverT !== null && this.hoverT >= t0 && this.hoverT <= t1) {
      const x = X(this.hoverT); c.strokeStyle = "rgba(11,11,11,0.45)"; c.lineWidth = 1; c.beginPath(); c.moveTo(x, y0); c.lineTo(x, y1); c.stroke();
      if (this.hoverSpec === sp) {
        const rows = sp.series.map(s => [s, st.valueAt(s.key, this.hoverT)]).filter(([, v]) => v === v);
        const bw = 150 * d, bh = (16 + rows.length * 14) * d, bx = Math.min(x + 8 * d, x1 - bw), by = y0 + 4 * d;
        c.fillStyle = "rgba(255,255,255,0.95)"; c.strokeStyle = "#e1e0d9"; c.fillRect(bx, by, bw, bh); c.strokeRect(bx, by, bw, bh);
        c.textAlign = "left"; c.fillStyle = "#52514e"; c.fillText(`t = ${this.hoverT.toFixed(2)} s`, bx + 6 * d, by + 12 * d);
        rows.forEach(([s, v], k) => { c.fillStyle = s.color; c.fillRect(bx + 6 * d, by + (19 + k * 14) * d, 10 * d, 3 * d);
          c.fillStyle = "#0b0b0b"; c.fillText(`${s.label}: ${v.toFixed(sp.dec ?? 3)} ${sp.unit || ""}`, bx + 20 * d, by + (24 + k * 14) * d); });
      }
    }
  }
}
global.Graficas = { Store, Charts };
})(window);

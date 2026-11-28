/* Visor 3D mínimo en WebGL (sin librerías): mallas con sombreado plano, cámara orbital
   y una capa 2D encima (overlay) para líneas, puntos y textos con la misma proyección.
   Unidades: mm. Z hacia arriba. */
"use strict";
(function (global) {
const M4 = {
  mul(a, b) { const o = new Float32Array(16);
    for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
      let s = 0; for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k]; o[c * 4 + r] = s; }
    return o; },
  persp(fov, asp, n, f) { const t = 1 / Math.tan(fov / 2), o = new Float32Array(16);
    o[0] = t / asp; o[5] = t; o[10] = (f + n) / (n - f); o[11] = -1; o[14] = 2 * f * n / (n - f); return o; },
  lookAt(e, c, up) {
    const z = norm(sub(e, c)), x = norm(cross(up, z)), y = cross(z, x);
    return new Float32Array([x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0,
      -dot(x, e), -dot(y, e), -dot(z, e), 1]); },
  /* matriz de modelo desde R (3x3, filas) y traslación t: p' = R p + t */
  fromRT(R, t) { return new Float32Array([R[0][0], R[1][0], R[2][0], 0, R[0][1], R[1][1], R[2][1], 0,
    R[0][2], R[1][2], R[2][2], 0, t[0], t[1], t[2], 1]); },
  ident() { const o = new Float32Array(16); o[0] = o[5] = o[10] = o[15] = 1; return o; },
};
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => { const n = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / n, a[1] / n, a[2] / n]; };

const VS = `attribute vec3 p; attribute vec3 n; uniform mat4 M, VP; varying vec3 vn; varying vec3 vw;
void main(){ vec4 w = M*vec4(p,1.0); vw = w.xyz; vn = mat3(M)*n; gl_Position = VP*w; }`;
const FS = `precision mediump float; varying vec3 vn; varying vec3 vw; uniform vec3 col; uniform float alpha;
uniform vec3 L1, L2; uniform float unlit; void main(){ vec3 nn = normalize(vn); if(!gl_FrontFacing) nn = -nn;
float d = 0.45 + 0.55*max(dot(nn,L1),0.0) + 0.25*max(dot(nn,L2),0.0);
gl_FragColor = vec4(unlit > 0.5 ? col : min(col*d, vec3(1.0)), alpha); }`;

class Visor {
  constructor(canvas, overlay) {
    this.cv = canvas; this.ov = overlay; this.ctx = overlay.getContext("2d");
    const gl = this.gl = canvas.getContext("webgl", { antialias: true, preserveDrawingBuffer: true, alpha: false });
    if (!gl) throw new Error("WebGL no disponible");
    const sh = (t, s) => { const o = gl.createShader(t); gl.shaderSource(o, s); gl.compileShader(o);
      if (!gl.getShaderParameter(o, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(o)); return o; };
    const pr = this.pr = gl.createProgram();
    gl.attachShader(pr, sh(gl.VERTEX_SHADER, VS)); gl.attachShader(pr, sh(gl.FRAGMENT_SHADER, FS)); gl.linkProgram(pr);
    this.loc = { p: gl.getAttribLocation(pr, "p"), n: gl.getAttribLocation(pr, "n"),
      M: gl.getUniformLocation(pr, "M"), VP: gl.getUniformLocation(pr, "VP"), col: gl.getUniformLocation(pr, "col"),
      alpha: gl.getUniformLocation(pr, "alpha"), unlit: gl.getUniformLocation(pr, "unlit"), L1: gl.getUniformLocation(pr, "L1"), L2: gl.getUniformLocation(pr, "L2") };
    this.meshes = {};
    this.cam = { target: [100, 0, 0], yaw: -2.3, pitch: 0.42, dist: 520, fov: 0.6 };
    this.home = JSON.parse(JSON.stringify(this.cam));
    this.bg = [1, 1, 1];
    this._mouse();
  }
  setMesh(name, v, f) {              // v: [x,y,z,...] mm, f: [i,j,k,...]
    const gl = this.gl, n = f.length, P = new Float32Array(n * 3), N = new Float32Array(n * 3);
    for (let t = 0; t < n; t += 3) {
      const a = f[t] * 3, b = f[t + 1] * 3, c = f[t + 2] * 3;
      const A = [v[a], v[a + 1], v[a + 2]], B = [v[b], v[b + 1], v[b + 2]], Cc = [v[c], v[c + 1], v[c + 2]];
      const nn = norm(cross(sub(B, A), sub(Cc, A)));
      [A, B, Cc].forEach((q, k) => { P.set(q, (t + k) * 3); N.set(nn, (t + k) * 3); });
    }
    const bp = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, bp); gl.bufferData(gl.ARRAY_BUFFER, P, gl.STATIC_DRAW);
    const bn = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, bn); gl.bufferData(gl.ARRAY_BUFFER, N, gl.STATIC_DRAW);
    this.meshes[name] = { bp, bn, count: n };
  }
  /* segmentos de línea (pares de puntos [x,y,z,...] en mm), dibujados con prueba de profundidad */
  setLines(name, pts) {
    const gl = this.gl, P = new Float32Array(pts), N = new Float32Array(pts.length);
    const bp = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, bp); gl.bufferData(gl.ARRAY_BUFFER, P, gl.STATIC_DRAW);
    const bn = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, bn); gl.bufferData(gl.ARRAY_BUFFER, N, gl.STATIC_DRAW);
    this.meshes[name] = { bp, bn, count: pts.length / 3, lines: true };
  }
  resize() {
    const r = this.cv.getBoundingClientRect(), d = window.devicePixelRatio || 1;
    const w = Math.max(1, Math.round(r.width * d)), h = Math.max(1, Math.round(r.height * d));
    if (this.cv.width !== w || this.cv.height !== h) { this.cv.width = w; this.cv.height = h; this.ov.width = w; this.ov.height = h; }
    this.dpr = d;
  }
  matrices() {
    const c = this.cam, cp = Math.cos(c.pitch);
    const eye = [c.target[0] + c.dist * cp * Math.cos(c.yaw), c.target[1] + c.dist * cp * Math.sin(c.yaw), c.target[2] + c.dist * Math.sin(c.pitch)];
    const V = M4.lookAt(eye, c.target, [0, 0, 1]), Pm = M4.persp(c.fov, this.cv.width / this.cv.height, 5, 5000);
    this.VP = M4.mul(Pm, V); this.eye = eye;
  }
  project(p) {                         // mm -> píxeles del overlay
    const m = this.VP, x = p[0], y = p[1], z = p[2];
    const X = m[0] * x + m[4] * y + m[8] * z + m[12], Y = m[1] * x + m[5] * y + m[9] * z + m[13];
    const W = m[3] * x + m[7] * y + m[11] * z + m[15];
    return [(X / W * 0.5 + 0.5) * this.cv.width, (1 - (Y / W * 0.5 + 0.5)) * this.cv.height, W];
  }
  /* list: [{mesh, M, col:[r,g,b], alpha}] — opacos primero, luego transparentes */
  draw(list) {
    this.resize(); this.matrices();
    const gl = this.gl, L = this.loc;
    gl.viewport(0, 0, this.cv.width, this.cv.height);
    gl.clearColor(this.bg[0], this.bg[1], this.bg[2], 1); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.useProgram(this.pr); gl.enable(gl.DEPTH_TEST);
    gl.uniformMatrix4fv(L.VP, false, this.VP);
    gl.uniform3fv(L.L1, norm([0.4, -0.5, 0.8])); gl.uniform3fv(L.L2, norm([-0.6, 0.7, 0.2]));
    const ops = list.filter(d => d.alpha >= 1), tr = list.filter(d => d.alpha < 1);
    gl.disable(gl.BLEND); gl.depthMask(true);
    for (const d of ops) this._one(d);
    gl.enable(gl.BLEND); gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA); gl.depthMask(false);
    for (const d of tr) this._one(d);
    gl.depthMask(true);
    const ctx = this.ctx; ctx.setTransform(1, 0, 0, 1, 0, 0); ctx.clearRect(0, 0, this.ov.width, this.ov.height);
  }
  _one(d) {
    const gl = this.gl, L = this.loc, m = this.meshes[d.mesh]; if (!m) return;
    gl.uniformMatrix4fv(L.M, false, d.M || M4.ident()); gl.uniform3fv(L.col, d.col); gl.uniform1f(L.alpha, d.alpha);
    gl.uniform1f(L.unlit, m.lines ? 1 : 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, m.bp); gl.enableVertexAttribArray(L.p); gl.vertexAttribPointer(L.p, 3, gl.FLOAT, false, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, m.bn); gl.enableVertexAttribArray(L.n); gl.vertexAttribPointer(L.n, 3, gl.FLOAT, false, 0, 0);
    gl.drawArrays(m.lines ? gl.LINES : gl.TRIANGLES, 0, m.count);
  }
  // ----- overlay
  line(pts, color, w = 1.5, dash = []) {
    const c = this.ctx, d = this.dpr; c.strokeStyle = color; c.lineWidth = w * d; c.setLineDash(dash.map(x => x * d)); c.beginPath();
    let pen = false;
    for (const p of pts) { const q = this.project(p); if (q[2] <= 0) { pen = false; continue; } pen ? c.lineTo(q[0], q[1]) : c.moveTo(q[0], q[1]); pen = true; }
    c.stroke(); c.setLineDash([]);
  }
  dot(p, color, r = 3) { const q = this.project(p); if (q[2] <= 0) return; const c = this.ctx; c.fillStyle = color; c.beginPath(); c.arc(q[0], q[1], r * this.dpr, 0, 2 * Math.PI); c.fill(); }
  text(p, t, color = "#0b0b0b", dx = 6, dy = -6, bold = false) {
    const q = this.project(p); if (q[2] <= 0) return; const c = this.ctx, d = this.dpr;
    c.font = `${bold ? "600 " : ""}${12 * d}px system-ui, Segoe UI, sans-serif`;
    c.lineWidth = 3 * d; c.strokeStyle = "rgba(255,255,255,0.85)"; c.strokeText(t, q[0] + dx * d, q[1] + dy * d);
    c.fillStyle = color; c.fillText(t, q[0] + dx * d, q[1] + dy * d);
  }
  arrow(o, v, color, label) {
    const e = [o[0] + v[0], o[1] + v[1], o[2] + v[2]];
    this.line([o, e], color, 2.5); this.dot(e, color, 3.5); if (label) this.text(e, label, color, 6, -6, true);
  }
  setView(yaw, pitch) { this.cam.yaw = yaw; this.cam.pitch = pitch; this.onchange && this.onchange(); }
  resetView() { this.cam = JSON.parse(JSON.stringify(this.home)); this.onchange && this.onchange(); }
  _mouse() {
    let drag = null; const el = this.ov;
    el.addEventListener("contextmenu", e => e.preventDefault());
    el.addEventListener("mousedown", e => { drag = { x: e.clientX, y: e.clientY, b: e.button, shift: e.shiftKey, cam: JSON.parse(JSON.stringify(this.cam)) }; });
    window.addEventListener("mouseup", () => drag = null);
    window.addEventListener("mousemove", e => {
      if (!drag) return; const dx = e.clientX - drag.x, dy = e.clientY - drag.y, c = this.cam;
      if (drag.b === 2 || drag.shift) {          // desplazar
        const k = c.dist * 0.0018, cy = Math.cos(c.yaw), sy = Math.sin(c.yaw);
        c.target = [drag.cam.target[0] + k * (dx * sy) - k * dy * (-cy) * 0,
                    drag.cam.target[1] - k * (dx * cy), drag.cam.target[2] + k * dy];
      } else { c.yaw = drag.cam.yaw - dx * 0.008; c.pitch = Math.max(-1.5, Math.min(1.5, drag.cam.pitch + dy * 0.008)); }
      this.onchange && this.onchange();
    });
    el.addEventListener("wheel", e => { e.preventDefault(); this.cam.dist = Math.max(80, Math.min(3000, this.cam.dist * Math.exp(e.deltaY * 0.001))); this.onchange && this.onchange(); }, { passive: false });
    el.addEventListener("dblclick", () => this.resetView());
  }
}
global.Visor3D = { Visor, M4 };
})(window);

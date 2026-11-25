// Prueba comun/robot.c en la PC: node test_robot.js
// En este PC no hay compilador de C para Windows, asi que se traduce robot.c (C sencillo,
// sin punteros raros) a JavaScript y se ejecuta la misma logica en doble precision.
const fs = require("fs"), path = require("path");

function cargar() {
  let s = fs.readFileSync(path.join(__dirname, "../comun/robot.c"), "utf8");
  s = s.replace(/\/\*[\s\S]*?\*\//g, "").replace(/#include.*/g, "");
  s = s.replace(/#define (\w+) (.+)/g, "const $1 = $2;");
  s = s.replace(/(\d\.\d*|\be[-+]?\d+)f\b/g, "$1").replace(/\b(\d+e[-+]?\d+)f\b/g, "$1");
  s = s.replace(/\b(sin|cos|atan2|acos|sqrt|floor)f\(/g, "Math.$1(").replace(/\bfabsf\(/g, "Math.abs(");
  // funciones
  s = s.replace(/^(?:static )?(?:void|int|float) (robot_\w+)\(([^)]*)\)\s*\{/gm, (_, n, a) =>
    `function ${n}(${a.split(",").map((x) => x.trim().replace(/\[[^\]]*\]/g, "").split(/[\s*]+/).pop()).join(", ")}) {`);
  // tablas
  s = s.replace(/static const float (\w+)(?:\[\d\])+ = (\{[\s\S]*?\});/g, (_, n, v) => `const ${n} = ${v.replace(/\{/g, "[").replace(/\}/g, "]")};`);
  s = s.replace(/static robot_pata_t (\w+)\[6\];/g, "const $1 = patas();");
  s = s.replace(/\b(?:static )?(?:float|int) (\w+)\[\d+\];/g, "const $1 = [];");
  s = s.replace(/\(float\)/g, "").replace(/\b(?:float|int) (\w+)( =|;)/g, "let $1$2");
  s = s.replace(/->/g, ".").replace(/&m\b/g, "R").replace(/robot_marcha\(c, f, p, R\)/g, "robot_marcha(c, f, p, R)");
  s = s.replace(/\*(margen_min|margen|pos|fase)\b/g, "$1.v").replace(/let m;\s*let f/g, "const R = {v:0}; let m; let f");
  s = s.replace(/(robot_marcha\(c, f, p, R\);)/g, "$1 m = R.v;");
  // enteros y constantes de robot.h
  const h = "const ROBOT_PATAS=6, MARCHA_ONDULADA=0, MARCHA_RIPPLE=1, MARCHA_TRIPODE=2;" +
    "const ROBOT_OK=0, ROBOT_INALCANZABLE=1, ROBOT_LIM_COXA=2, ROBOT_LIM_FEMUR=3, ROBOT_LIM_TIBIA=4, ROBOT_INESTABLE=5, ROBOT_CONFIG=6, ROBOT_CHOQUE=7, ROBOT_SEP_MIN=25;" +
    "function patas(){return Array.from({length:6},()=>({pie:[0,0,0],q:[0,0,0],apoyo:false,avance:0,motivo:0}));}";
  return new Function(h + s + ";return {robot_cfg_defecto,robot_directa,robot_inversa,robot_j0_a_cuerpo,robot_cuerpo_a_j0,robot_marcha,robot_validar,robot_separacion,robot_factor_apoyo,patas};")();
}
module.exports = { cargar };
if (require.main !== module) return;

const R = cargar();
let fallas = 0;
const ok = (c, t) => { console.log((c ? "PASA  " : "FALLA ") + t); if (!c) fallas++; };
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
const FK = (q) => { const p = [0, 0, 0]; R.robot_directa(q, p); return p; };

// 1. Directa contra la tabla de 03-control/cinematica/README.md (R en planta, dH)
const p0 = FK([0, 0, 0]);
let peor = 0;
for (const [c, f, t, Rr, dH] of [[0,0,0,207.9,0],[0,0,.8,152.1,-59.2],[0,-.5,0,206.0,80.1],[0,-.5,1.2,154.1,-25.7],[0,.5,.5,124.9,-94.4],[0,-1,1.5,150.9,8.0],[.5,0,0,207.9,0]]) {
  const p = FK([c, f, t]); peor = Math.max(peor, Math.abs(Math.hypot(p[0], p[1]) - Rr), Math.abs(p[2] - p0[2] - dH));
}
ok(peor < 0.06, `directa = tabla del README (peor ${peor.toFixed(3)} mm)`);

// 2. Ejemplo de la inversa del README: (150,0,-110) -> (0, 0.363, 0.418)
const q = [0, 0, 0]; const r = R.robot_inversa([150, 0, -110], q);
ok(r === 0 && Math.abs(q[1] - .363) < 1e-3 && Math.abs(q[2] - .418) < 1e-3, `inversa = ejemplo del README (${q.map((v) => v.toFixed(3))})`);

// 3. Ida y vuelta
let e = 0, mal = 0;
for (let k = 0; k < 20000; k++) {
  const a = [-.5 + 1.1 * Math.random(), -1 + 2 * Math.random(), -.34 + 1.04 * Math.random()], b = [0, 0, 0];
  if (R.robot_inversa(FK(a), b) === 1) { mal++; continue; }
  e = Math.max(e, dist(FK(a), FK(b)));
}
ok(!mal && e < 1e-3, `ida y vuelta, 20000 poses: ${mal} fallas, error ${e.toExponential(1)} mm`);
ok(R.robot_inversa([400, 0, 0], q) === 1, "punto fuera de alcance: rechazado");
ok(R.robot_inversa(FK([0, 0, 1.0]), q) === 4, "tibia a 1.0 rad: rechazada por limite");

// 4. Marco del cuerpo
let em = 0;
for (let i = 0; i < 6; i++) { const a = [1, 2, 3], b = [0, 0, 0], c = [0, 0, 0]; R.robot_j0_a_cuerpo(i, a, b); R.robot_cuerpo_a_j0(i, b, c); em = Math.max(em, dist(a, c)); }
ok(em < 1e-9, "J0 -> cuerpo -> J0");

// 5. Marchas
const cfg = (x) => { const c = {}; R.robot_cfg_defecto(c); return Object.assign(c, x); };
const val = (c) => { const v = {}; const r = R.robot_validar(c, v); return { r, m: v.margen, ...v }; };
["ondulada", "ripple", "tripode"].forEach((n, k) => {
  const c = cfg({ marcha: k }), v = val(c);
  ok(v.r === 0, `${n}: ciclo valido, margen ${v.m.toFixed(1)} mm, separacion ${v.sep.toFixed(0)} mm, velocidad ${(v.dq / 12).toFixed(2)} rad/s con ciclo de 12 s`);
  const P = R.patas(), m = { v: 0 }; let maxV = 0, salto = 0, ant = null, des = 0;
  const N = 2400, ap = R.robot_factor_apoyo(k);
  for (let j = 0; j <= N; j++) {
    R.robot_marcha(c, (j / N) % 1, P, m);
    maxV = Math.max(maxV, P.filter((p) => !p.apoyo).length);
    if (ant) P.forEach((p, i) => {
      salto = Math.max(salto, dist(p.pie, ant[i].pie));
      if (p.apoyo && ant[i].apoyo) des = Math.max(des, Math.abs((p.pie[0] - ant[i].pie[0]) * N + c.paso_x / ap));
      });
    ant = P.map((p) => ({ pie: p.pie.slice(), apoyo: p.apoyo }));
  }
  ok(maxV === [1, 2, 3][k], `${n}: maximo ${maxV} patas en vuelo`);
  ok(salto < 0.5, `${n}: trayectoria continua (salto max ${salto.toFixed(3)} mm)`);
  ok(des < 1e-6, `${n}: los pies de apoyo retroceden todos a paso/apoyo`);
});
{ let g = 0, l = 0; for (let x = 1; x < 30; x++) { if (val(cfg({ paso_x: 0, paso_giro: x * Math.PI / 180 })).r === 0) g = x; if (val(cfg({ paso_x: 0, paso_y: x })).r === 0) l = x; }
  console.log(`INFO  elevacion 15 mm: giro maximo ${g} grados, lateral maximo ${l} mm`); ok(g >= 3 && l >= 5, "girar y andar de lado es posible"); }
ok(val(cfg({ paso_x: 120 })).r !== 0, "paso de 120 mm: rechazado");
ok(val(cfg({ altura: 160 })).r !== 0, "altura de 160 mm: rechazada");
for (const [n, x] of [["NaN", { radio: NaN }], ["infinito", { paso_x: Infinity }], ["altura negativa", { altura: -50 }], ["elevacion > altura", { elevacion: 90 }], ["marcha 7", { marcha: 7 }], ["radio 0", { radio: 0 }]])
  ok(val(cfg(x)).r === 6, `parametro invalido (${n}): rechazado`);
{ const P = R.patas(), m = { v: 0 }; R.robot_marcha(cfg({}), 0, P, m);
  const s0 = R.robot_separacion(P, 0, 1); P[0].pie[0] = -40; P[0].pie[1] = 200;
  ok(s0 > 25 && R.robot_separacion(P, 0, 1) === 0, `choque: patas 1 y 2 normales ${s0.toFixed(0)} mm, cruzadas 0`); }
{ /* trayectoria: giro puro no traslada; la pata 1 (delantera derecha) y la 6 (delantera izquierda) */
  const P = R.patas(), m = { v: 0 }; R.robot_marcha(cfg({}), 0, P, m);
  ok([[1,1],[0,1],[-1,1],[-1,-1],[0,-1],[1,-1]].every(([x, y], i) => Math.sign(P[i].pie[1]) === y && (!x || Math.sign(P[i].pie[0]) === x)), "posiciones: 1-2-3 a un lado, 4-5-6 al otro, 1 y 6 al frente");
  const o = (i) => { const r = [0, 0, 0]; R.robot_j0_a_cuerpo(i, [0, 0, 0], r); return r; };
  // Centros de J0 en la captura del usuario (px): 1 (527,467) 6 (393,467) 3 (527,292) 2 (550,387) 5 (370,387)
  const s = 180 / (o(1)[1] - o(4)[1]), e = [(o(0)[1] - o(5)[1]) * s - 134, (o(0)[0] - o(2)[0]) * s - 175, (o(0)[0] - o(1)[0]) * s - 80].map((v) => Math.abs(v / s));
  ok(Math.max(...e) < 2, `montaje de J0 = captura de Fusion (diferencias ${e.map((v) => v.toFixed(1))} mm)`); }
for (const el of [15, 20, 25, 30]) { let mx = 0; for (let s = 2; s <= 80; s += 2) if (val(cfg({ paso_x: s, elevacion: el })).r === 0) mx = s; console.log(`INFO  elevacion ${el} mm: paso maximo adelante ${mx} mm (radio 182, altura 75)`); }
let best = [0]; for (let rad = 140; rad <= 200; rad += 2) for (let alt = 40; alt <= 115; alt += 5) for (let s = 2; s <= 80; s += 2) if (val(cfg({ radio: rad, altura: alt, paso_x: s, elevacion: 20 })).r === 0 && s > best[0]) best = [s, rad, alt];
console.log(`INFO  mejor postura para elevacion 20: paso ${best[0]} mm con radio ${best[1]} y altura ${best[2]}`);
console.log(fallas ? `\n${fallas} FALLAS` : "\nTODO PASA");
process.exit(fallas ? 1 : 0);

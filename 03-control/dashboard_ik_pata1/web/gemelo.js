/* Gemelo digital de la Pata 1: réplica en JavaScript de cascade_control.py / motor_model.py,
   que a su vez replican pi_update_sample, speed_pi_update, position_pi_update,
   friction_feedforward, cascade_current y dither_term del firmware pata_MOD_DRV.
   Trabaja en el marco del encoder (rad), igual que el firmware. Paso de control 1 ms,
   planta identificada a 0.2 ms. Corrientes en A equivalentes del sensado. */
"use strict";
(function (global) {
const P_PERIOD = 20000, P_QUIET = 100000, P_REV_QUIET = 20000, P_STOP = 0.3;
const P_STILL = 0.02, P_DEADBAND = 0.05, S_PERIOD = 10000, OVERSPEED = 0.5;
const F_MOVING = 0.15, F_LOOSE = 0.01, F_FAST = 0.2, F_MAXF = 2.0, G_MIN = 0.01;
const TRACK = 0, REVERSING = 1, HOLD = 2, ARRIVING = 3;
const STOP = 0, BRAKE = 2;
const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));

function gravHold(terms, pos, positions) {
  let tot = 0;
  for (const t of terms || []) {
    let arg = pos + t.theta0;
    if (t.motor !== null && t.motor !== undefined) arg += t.c * positions[t.motor];
    tot += t.g * Math.cos(arg);
  }
  return tot;
}

class Joint {
  constructor(j, C, P, cpr) {
    this.j = j; this.cpr = cpr; this.setGains(C); this.setPlant(P);
    // La gravedad de la planta es la calibrada en config.yaml; la de la compensación (this.grav)
    // puede cambiar con las líneas CFG de la placa.
    this.plantGrav = JSON.parse(JSON.stringify(C.gravedad[j] || []));
    this.reset(0);
  }
  setGains(C) {
    const j = this.j;
    this.kpI = C.pid_kp[j]; this.kiI = C.pid_ki[j]; this.outMax = C.pwm_max_pct[j];
    this.kpS = C.spi_kp[j]; this.kiS = C.spi_ki[j]; this.ilim = C.ilimit_a[j];
    this.kpP = C.ppi_kp[j]; this.kiP = C.ppi_ki[j]; this.vmax = C.vmax_rad_s[j]; this.acc = C.acel_rad_s2[j];
    this.tol = C.tolerancia_rad[j]; this.ic = C.frici_ic_a[j]; this.is = C.frici_is_a[j];
    this.fricbStart = C.fricb_inicio[j]; this.fricbRampUs = C.fricb_rampa_s[j] * 1e6;
    this.ditherPct = C.dither_pct[j]; this.ditherHz = C.dither_hz[j];
    this.grav = C.gravedad[j]; this.holdBrake = C.freno_en_hold;
  }
  setPlant(P) {
    const j = this.j;
    this.a = P.a_a_por_pct[j]; this.tauE = P.tau_e_s; this.dz = P.zona_muerta_pct; this.J = P.jeq_a_s2_rad[j];
    this.pIc = P.ic_a[j]; this.pB = P.b_a_s_rad[j]; this.pIs = Math.max(P.is_a[j], P.ic_a[j]);
    this.bBrake = P.b_freno_a_s_rad; this.gFactor = P.factor_gravedad_planta;
  }
  reset(theta) {             // estado en reposo en la posición dada
    this.theta = theta; this.omega = 0; this.iSense = 0; this.torque = 0;
    const k = 2 * Math.PI / this.cpr; this.c0 = Math.floor(theta / k); this.hist = [];
    this.mode = 0;           // 0 libre (copia la real), 3 posición
    this.ref = theta;
    this.pid = { ref: 0, integ: 0, out: 0, err: 0, last: 0 };
    this.spd = { ref: 0, integ: 0, out: 0, err: 0, last: 0, limited: false };
    this.pos = { err: 0, integ: 0, out: 0, dir: 1, last: 0, quiet: 0, state: HOLD, limited: false };
    this.ctrlDir = 1; this.drive = 1; this.dirOut = STOP; this.speedCurrent = 0; this.fric = 0;
    this.breakaway = false; this.stuckSince = 0; this.stuckPos = 0; this.gTerm = 0;
    this.u = 0; this.applied = STOP; this.iBlock = 0; this.meas = [theta, 0, false];
  }
  // ---------- encoder (PCNT x4, ventana de 20 ms)
  sample() {
    const k = 2 * Math.PI / this.cpr, cnt = Math.floor(this.theta / k) - this.c0;
    this.hist.push(cnt); if (this.hist.length > 21) this.hist.shift();
    const valid = this.hist.length > 20;
    this.meas = [(cnt + this.c0) * k, valid ? (this.hist[20] - this.hist[0]) * k / 0.02 : 0, valid];
    return this.meas;
  }
  // ---------- controladores (mismas reglas que el firmware)
  pidReset() { const p = this.pid; p.integ = p.out = p.err = 0; p.last = 0; }
  pidUpdate(meas, t) {
    const p = this.pid, dt = p.last ? (t - p.last) * 1e-6 : 0, e = p.ref - meas;
    const cand = p.integ + this.kiI * e * dt, eff = this.kpI * e + cand;
    const hi = eff > this.outMax, lo = eff < 0;
    if (!((hi && e > 0) || (lo && e < 0))) p.integ = cand;
    p.out = Math.min(this.outMax, Math.max(0, eff)); p.err = e; p.last = t;
  }
  spdReset() { const s = this.spd; s.integ = s.out = s.err = 0; s.last = 0; s.limited = false; }
  spdUpdate(proj, t, innerHigh) {
    const s = this.spd, dt = s.last ? (t - s.last) * 1e-6 : 0, e = s.ref - proj;
    const cand = s.integ + this.kiS * e * dt; let req = this.kpS * e + cand;
    const hi = req > this.ilim, lo = req < 0;
    if (!((hi && e > 0) || (lo && e < 0) || (innerHigh && e > 0))) s.integ = cand;
    if (innerHigh && e > 0) req = this.kpS * e + s.integ;
    s.out = Math.min(this.ilim, Math.max(0, req)); s.err = e; s.last = t;
    s.limited = hi || lo || (innerHigh && e > 0);
  }
  posUpdate(meas, speed, now, downstream) {
    const p = this.pos, dt = p.last ? (now - p.last) * 1e-6 : 0, e = this.ref - meas;
    p.err = e; p.last = now;
    const band = (p.state === HOLD || p.state === ARRIVING) ? 2 * this.tol : this.tol;
    if (Math.abs(e) <= band) {
      p.out = p.integ = 0; p.limited = false;
      if (Math.abs(speed) <= P_STOP) { if (!p.quiet) p.quiet = now; p.state = now - p.quiet >= P_QUIET ? HOLD : ARRIVING; }
      else { p.quiet = 0; p.state = ARRIVING; }
      return;
    }
    if (p.state === HOLD || p.state === ARRIVING) { p.state = TRACK; p.quiet = 0; }
    let cand = p.integ + this.kiP * e * dt, req = this.kpP * e + cand;
    if (req * e < 0) { p.integ = 0; cand = this.kiP * e * dt; req = this.kpP * e + cand; }
    const wanted = req > 0 ? 1 : req < 0 ? -1 : p.dir;
    if (wanted !== p.dir || wanted * speed < -P_STOP || p.state === REVERSING) {
      p.state = REVERSING; p.out = p.integ = 0; p.limited = true;
      if (Math.abs(speed) <= P_STOP) {
        if (!p.quiet) p.quiet = now;
        if (now - p.quiet >= P_REV_QUIET) { p.dir = wanted; p.state = TRACK; p.quiet = 0; }
      } else p.quiet = 0;
      return;
    }
    p.quiet = 0;
    const blocked = downstream && e * p.dir > 0;
    if (blocked) req = this.kpP * e + p.integ;
    const dist = Math.max(0, Math.abs(e) - this.tol), lat = P_PERIOD * 1e-6 + 0.02, al = this.acc * lat;
    const approach = 2 * this.acc * dist / (Math.sqrt(al * al + 2 * this.acc * dist) + al);
    const lim = Math.min(this.vmax, approach);
    const bounded = Math.min(lim, Math.max(-lim, req)), delta = this.acc * dt;
    let ramped = Math.min(p.out + delta, Math.max(p.out - delta, bounded));
    ramped = Math.min(lim, Math.max(-lim, ramped));
    const sat = (req - ramped) * e > 0;
    if (!sat && !blocked) p.integ = cand;
    p.out = ramped; p.limited = sat || blocked; p.state = TRACK;
  }
  frictionFF(pos, speed, valid, now) {
    const ic = this.ic, is = Math.max(this.is, this.ic);
    if (this.spd.ref === 0 || is <= 0 || !valid) { this.stuckSince = 0; this.breakaway = false; return 0; }
    const d = this.ctrlDir < 0 ? -1 : 1, proj = d * speed;
    const moved = this.stuckSince ? d * (pos - this.stuckPos) : (proj > 0 ? F_LOOSE : -F_LOOSE);
    if (proj > F_MOVING && moved >= F_LOOSE) { this.stuckSince = 0; this.breakaway = false; return ic; }
    if (proj < -F_MOVING && moved <= -F_LOOSE) { this.stuckSince = 0; this.breakaway = false; return 0; }
    if (!this.stuckSince) { this.stuckSince = now; this.stuckPos = pos; }
    this.breakaway = true;
    const sf = Math.abs(this.spd.ref) >= F_FAST ? this.fricbStart : 0, start = ic + (is - ic) * sf;
    const cap = Math.max(Math.min(F_MAXF * is, this.ilim), ic);
    const ramp = (now - this.stuckSince) / (this.fricbRampUs > 0 ? this.fricbRampUs : 1);
    return Math.min(start + (cap - start) * clamp(ramp, 0, 1), cap);
  }
  dither(now) {
    if (this.ditherPct <= 0 || this.ditherHz <= 0 || this.spd.ref === 0) return 0;
    const half = Math.floor(500000 / this.ditherHz);
    return (Math.floor(now / half) & 1) ? -this.ditherPct : this.ditherPct;
  }
  // ---------- paso de control de 1 ms (motor_control_step, parte de cascada)
  control(now, positions) {
    const [pos, speed, valid] = this.meas;
    if (this.mode !== 3) { this.u = 0; this.applied = STOP; return; }
    if (now - this.pos.last >= P_PERIOD) {
      const downstream = this.spd.limited || (this.pid.out >= this.outMax - 0.01 && this.pid.err > 0);
      const prev = this.pos.dir;
      this.posUpdate(pos, speed, now, downstream);
      if (prev !== this.pos.dir || this.pos.out === 0) {
        this.spdReset();
        if (!(this.grav && this.grav.length)) { this.pidReset(); this.pid.ref = 0; }
        this.speedCurrent = 0; this.spd.last = now;
      }
      this.ctrlDir = this.pos.dir > 0 ? 1 : -1;
      this.spd.ref = Math.abs(this.pos.out);
    }
    if (this.spd.ref === 0) { this.spdReset(); this.spd.last = now; this.speedCurrent = 0; }
    else if (now - this.spd.last >= S_PERIOD) {
      const proj = this.ctrlDir * speed;
      const innerHigh = (this.drive === this.ctrlDir && this.pid.out >= this.outMax - 0.01 && this.pid.err > 0) || this.breakaway;
      if (proj > this.spd.ref + OVERSPEED) this.spd.integ = 0;
      this.spdUpdate(proj, now, innerHigh); this.speedCurrent = this.spd.out;
    }
    this.fric = this.frictionFF(pos, speed, valid, now);
    const part = this.speedCurrent + this.fric, hold = gravHold(this.grav, pos, positions);
    this.gTerm = hold;
    let drive, iref;
    if (hold === 0) { drive = this.ctrlDir; iref = clamp(part, 0, this.ilim); }
    else {
      const net = (this.ctrlDir < 0 ? -part : part) + hold;
      drive = net >= 0 ? 1 : -1; const mag = Math.abs(net); iref = mag < G_MIN ? 0 : Math.min(mag, this.ilim);
    }
    this.pid.ref = iref;
    if (drive !== this.drive) { this.pidReset(); this.drive = drive; }
    let dirBrake = false;
    if ((this.dirOut === 1 || this.dirOut === -1) && this.drive !== this.dirOut) { dirBrake = true; this.pidReset(); }
    let brakeHold = this.holdBrake && this.pos.state !== TRACK, over = false;
    if (this.holdBrake && valid && this.ctrlDir * speed > this.spd.ref + OVERSPEED) {
      brakeHold = true; over = this.pid.ref === 0 || this.drive === this.ctrlDir;
    }
    if (over) {
      this.pidReset(); this.pid.ref = 0; this.pid.last = now;
      this.spd.integ = this.spd.out = this.speedCurrent = 0; this.fric = 0; this.breakaway = false; this.stuckSince = 0;
      this.applied = BRAKE; this.u = 0;
    } else if (dirBrake) { this.pid.last = now; this.applied = BRAKE; this.u = 0; }
    else if (this.pid.ref === 0) { this.pidReset(); this.pid.last = now; this.applied = brakeHold ? BRAKE : STOP; this.u = 0; }
    else {
      this.pidUpdate(this.iBlock, now);
      this.u = clamp(this.pid.out + this.dither(now), 0, this.outMax); this.applied = this.drive;
    }
    this.dirOut = this.applied;
  }
  // ---------- planta identificada (Jeq·α = T − Ic·sign(ω) − b·ω − gravedad), dt = 0.2 ms
  plant(dt, positions) {
    const s = (this.applied === 1 || this.applied === -1) ? this.applied : 0;
    const target = s ? this.a * Math.max(this.u - this.dz, 0) : 0;
    this.iSense += (target - this.iSense) * (1 - Math.exp(-dt / this.tauE));
    let T = s * this.iSense;
    if (this.applied === BRAKE) T = -this.bBrake * this.omega;
    this.torque = T;
    const fext = T - this.gFactor * gravHold(this.plantGrav, this.theta, positions);
    let acc;
    if (Math.abs(this.omega) < 1e-3) {
      if (Math.abs(fext) <= this.pIs) { this.omega = 0; return; }
      acc = (fext - Math.sign(fext) * this.pIc) / this.J;
    } else acc = (fext - Math.sign(this.omega) * this.pIc - this.pB * this.omega) / this.J;
    let w = this.omega + acc * dt;
    if (this.omega !== 0 && w * this.omega < 0) w = 0;
    this.theta += 0.5 * (this.omega + w) * dt; this.omega = w;
  }
}

class Twin {
  /* G.gemelo = {control, planta, cpr}. q0: posiciones iniciales (encoder). */
  constructor(gem, q0) {
    this.gem = gem; this.t = 0;
    this.J = [0, 1, 2].map(j => new Joint(j, gem.control, gem.planta, gem.cpr));
    this.J.forEach((jt, i) => jt.reset(q0 ? q0[i] : 0));
  }
  setGains(j, C) { this.J[j].setGains(C); }
  /* Avanza dtS segundos (pasos de 1 ms) */
  advance(dtS) {
    const n = Math.min(200, Math.max(0, Math.round(dtS * 1000)));
    for (let k = 0; k < n; k++) this.step1ms();
  }
  step1ms() {
    this.t += 1000;
    const now = this.t, J = this.J;
    const meas = J.map(j => j.sample()), positions = meas.map(m => m[0]);
    J.forEach(j => j.control(now, positions));
    const acc = [0, 0, 0];
    for (let s = 0; s < 5; s++) {
      const th = J.map(j => j.theta);
      J.forEach((j, i) => { if (j.mode === 3) j.plant(2e-4, th); acc[i] += j.iSense; });
    }
    J.forEach((j, i) => { j.iBlock = acc[i] / 5; });
  }
  /* Activa el modo posición de una articulación con un objetivo (como PREF + MODE,POSITION) */
  command(i, ref, fromQ) {
    const j = this.J[i];
    if (fromQ !== undefined && fromQ !== null) j.reset(fromQ);
    j.ref = ref;
    j.pos.integ = j.pos.out = 0; j.pos.state = TRACK; j.pos.last = this.t; j.pos.dir = ref >= j.theta ? 1 : -1;
    j.spdReset(); j.spd.ref = 0; j.spd.last = this.t; j.speedCurrent = 0;
    if (!(j.grav && j.grav.length)) { j.pidReset(); j.pid.ref = 0; }
    j.mode = 3;
  }
  release(i, q) { this.J[i].reset(q); }
  snapshot() {       // mismos campos que la telemetría T
    return this.J.map(j => ({
      mode: j.mode, duty: (j.applied === 1 || j.applied === -1 ? j.applied : 0) * j.u, cur: j.iBlock,
      curRef: j.pid.ref * (j.applied === -1 ? -1 : 1), pos: j.meas[0], spd: j.meas[1], spdRef: j.ctrlDir * j.spd.ref,
      posRef: j.ref, state: j.pos.state, theta: j.theta, omega: j.omega, torque: j.torque, brake: j.applied === BRAKE,
    }));
  }
}
global.Gemelo = { Twin, gravHold, STATES: ["TRACK", "REVERSA", "HOLD", "LLEGANDO"] };
})(window);

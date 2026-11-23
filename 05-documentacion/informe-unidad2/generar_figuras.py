"""Figuras del informe de la Unidad 2. Todo sale de los CSV de matlab/datos o de
valores medidos anotados en 02-electronica/notas. Escribe figuras/*.png y metricas.json."""
import csv, json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

AQUI = os.path.dirname(os.path.abspath(__file__))
D = os.path.join(AQUI, "..", "..", "03-control", "pata_MOD_DRV", "matlab", "datos")
F = os.path.join(AQUI, "figuras"); os.makedirs(F, exist_ok=True)
AZ, VI, VE, NA, GR, RO = "#1f6fb2", "#7a4fb5", "#2b8a57", "#d9822b", "#6b7580", "#b3282d"
COL = {1: AZ, 2: VI, 3: VE}; NOM = {1: "Coxa", 2: "Fémur", 3: "Tibia"}
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.grid": True, "grid.color": "#dfe3e8",
    "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False, "axes.axisbelow": True,
    "legend.frameon": False, "figure.dpi": 100, "savefig.dpi": 220, "savefig.bbox": "tight"})
M = {}
from matplotlib.ticker import ScalarFormatter
_c = ScalarFormatter.__call__
def class_coma(self, x, pos=None): return _c(self, x, pos).replace('.', ',')
ScalarFormatter.__call__ = class_coma

def leer(n):
    with open(os.path.join(D, n), encoding="utf-8", errors="replace") as f:
        r = csv.reader(f); h = next(r); filas = [x for x in r if len(x) == len(h)]
    def num(v):
        try: return float(v)
        except ValueError: return np.nan
    a = np.array([[num(v) for v in x] for x in filas])
    return {k: a[:, i] for i, k in enumerate(h)}

def guardar(fig, n): fig.savefig(os.path.join(F, n + ".png")); plt.close(fig)

def escalones(d, m):
    t = d["time_ms"]/1000; q = d[f"m{m}_position_rad"]; r = d[f"m{m}_position_ref_rad"]
    mo = d[f"m{m}_mode"]; es = d[f"m{m}_position_state"]
    c = [0]+[i for i in range(1, len(t)) if r[i] != r[i-1] or mo[i] != mo[i-1]]+[len(t)]
    out = []
    for a, b in zip(c[:-1], c[1:]):
        if mo[a] != 3 or t[b-1]-t[a] < 0.3 or abs(r[a]-q[a]) < 0.1: continue
        s = r[a]-q[a]; h = np.where(es[a:b] == 2)[0]; e = q[a:b]-r[a]
        out.append(dict(ini=round(q[a], 3), ref=r[a], llego=bool(len(h)), t=float(t[a+h[0]]-t[a]) if len(h) else None,
                        sobrepaso=float(max(0, np.max(np.sign(s)*e))/abs(s)*100), efin=float(abs(e[-1]))))
    return out

PR = {"fem_a": ("femur_prueba_ganancias.csv", 2), "fem_b": ("femur_prueba_ganancias (1).csv", 2), "fem_c": ("femur_prueba_ganancias (2).csv", 2),
      "tib_a": ("tibia_prueba_ganancias.csv", 3), "tib_c": ("tibia_prueba_ganancias (2).csv", 3), "cox": ("coxa_prueba_ganancias.csv", 1)}
DAT = {k: leer(v[0]) for k, v in PR.items()}
for k, (n, m) in PR.items():
    E = escalones(DAT[k], m); ok = [e for e in E if e["llego"]]
    M[k] = dict(archivo=n, n=len(E), llegaron=len(ok), t_media=float(np.mean([e["t"] for e in ok])), t_min=min(e["t"] for e in ok),
                t_max=max(e["t"] for e in ok), efin_max=max(e["efin"] for e in E), efin_max_llegaron=max(e["efin"] for e in ok),
                sob_max=max(e["sobrepaso"] for e in E), pasos=E)

# 1-3 respuesta en posicion
def respuesta(k, nombre):
    d, m = DAT[k], PR[k][1]; t = (d["time_ms"]-d["time_ms"][0])/1000; p = f"m{m}_"
    fig, ax = plt.subplots(3, 1, figsize=(7, 5.6), sharex=True, gridspec_kw={"height_ratios": [2.2, 1.2, 1.2]})
    ax[0].plot(t, d[p+"position_ref_rad"], color=NA, ls="--", lw=1.3, label="Referencia"); ax[0].plot(t, d[p+"position_rad"], color=COL[m], lw=1.6, label="Medida")
    ax[0].set_ylabel("Posición (rad)"); ax[0].legend(loc="upper left", ncol=2, bbox_to_anchor=(0, 1.14))
    ax[1].plot(t, d[p+"speed_ref_rad_s"], color=NA, ls="--", lw=1); ax[1].plot(t, d[p+"speed_rad_s"], color=COL[m], lw=.9); ax[1].set_ylabel("Velocidad\n(rad/s)")
    ax[2].plot(t, d[p+"duty_percent"], color=GR, lw=.8); ax[2].set_ylabel("PWM (%)"); ax[2].set_xlabel("Tiempo (s)")
    fig.align_ylabels(); guardar(fig, nombre)
respuesta("fem_c", "fig_resp_femur"); respuesta("tib_c", "fig_resp_tibia"); respuesta("cox", "fig_resp_coxa")

# 4 antes/despues del empujon en el femur
fig, ax = plt.subplots(2, 1, figsize=(7, 4.4))
for a, k, ti in ((ax[0], "fem_a", "Is = 0,036 A: 4 de 7 llegan"), (ax[1], "fem_b", "Is = 0,070 A: 7 de 7 llegan")):
    d = DAT[k]; t = (d["time_ms"]-d["time_ms"][0])/1000
    a.plot(t, d["m2_position_ref_rad"], color=NA, ls="--", lw=1.3, label="Referencia"); a.plot(t, d["m2_position_rad"], color=VI, lw=1.5, label="Medida")
    a.set_title(ti, loc="left", fontsize=9.5); a.set_ylabel("Posición (rad)")
ax[0].legend(loc="upper right", ncol=2, bbox_to_anchor=(1, 1.2)); ax[1].set_xlabel("Tiempo (s)"); fig.tight_layout(); guardar(fig, "fig_femur_empujon")

# 5 tiempos y errores finales
fig, ax = plt.subplots(1, 2, figsize=(7, 3))
for i, k in enumerate(("cox", "fem_c", "tib_c")):
    m = PR[k][1]; T = [e["t"] for e in M[k]["pasos"]]; Er = [e["efin"]*1000 for e in M[k]["pasos"]]
    x = i+np.linspace(-.18, .18, len(T)); ax[0].scatter(x, T, color=COL[m], s=26); ax[0].hlines(np.mean(T), i-.3, i+.3, color=COL[m], lw=2)
    ax[0].text(i+.32, np.mean(T), f"{np.mean(T):.2f}".replace(".", ","), va="center", fontsize=8.5)
    ax[1].scatter(x, Er, color=COL[m], s=26)
ax[1].axhline(10, color=GR, ls=":", lw=1.2); ax[1].text(-.45, 20.4, "2 × tolerancia", fontsize=8, color=GR); ax[1].axhline(20, color=GR, ls=":", lw=1.2); ax[1].text(.55, 10.4, "tolerancia", fontsize=8, color=GR)
for a in ax: a.set_xticks(range(3)); a.set_xticklabels(["Coxa", "Fémur", "Tibia"]); a.set_xlim(-.5, 2.6)
ax[0].set_ylabel("Tiempo de llegada (s)"); ax[1].set_ylabel("Error final (mrad)"); ax[0].set_ylim(0); ax[1].set_ylim(0); fig.tight_layout(); guardar(fig, "fig_desempeno")

# 6-7 identificacion (mismo metodo de matlab/identificar_articulacion.m)
def suav(x, k=5):
    v = np.isfinite(x); n = np.arange(len(x)); xi = np.interp(n, n[v], x[v]); ker = np.ones(k)/k
    y = np.convolve(xi, ker, "same")/np.convolve(np.ones(len(x)), ker, "same"); y[~v] = np.nan; return y
def ident(arch, m, grav):
    A, Y, TH, W = [], [], [], []
    for n in arch:
        d = leer(n); p = f"m{m}_"; t = d["time_ms"]/1000; w0 = d[p+"speed_rad_s"]; ws = suav(w0)
        k = max(1, round(.01/np.median(np.diff(t)))); ws = np.r_[ws[k:], np.full(k, np.nan)]
        ac = suav(np.gradient(ws, t)); I = np.sign(d[p+"duty_percent"])*d[p+"current_a"]; th = d[p+"position_rad"]
        ok = np.isin(d[p+"current_status"], [1, 9, 11, 13, 14, 15]) & np.isfinite(w0) & np.isfinite(ac) & np.isfinite(I) & ((d[p+"flags"].astype(int) & 16) == 0) & (np.abs(ws) > .3)
        c = [ac, np.sign(ws), ws]+([np.cos(th), np.sin(th)] if grav else []); A.append(np.c_[tuple(c)][ok]); Y.append(I[ok]); TH.append(th[ok]); W.append(ws[ok])
    X, y, th, w = np.vstack(A), np.concatenate(Y), np.concatenate(TH), np.concatenate(W)
    c = np.linalg.lstsq(X, y, rcond=None)[0]; r = y-X@c; s = 1.4826*np.median(np.abs(r-np.median(r))); b = np.abs(r) < 3*s
    X, y, th, w = X[b], y[b], th[b], w[b]; c = np.linalg.lstsq(X, y, rcond=None)[0]; r = y-X@c
    G = float(np.hypot(c[3], c[4])) if grav else 0.; t0 = float(np.arctan2(-c[4], c[3])) if grav else 0.
    return dict(J=c[0], Ic=c[1], b=c[2], G=G, th0=t0, r2=1-np.sum(r**2)/np.sum((y-y.mean())**2), n=len(y)), X, y, th, w, c
IDS = {"coxa": (["coxa_rampa.csv", "coxa_vaiven (2).csv"], 1, False), "tibia": (["tibia_colgando_rampa.csv", "tibia_colgando_vaiven.csv"], 3, True),
       "femur_t0": (["femur_rampa.csv", "femur_tibia_estirada.csv"], 2, True), "femur_tm": (["femur_rampa (2).csv", "femur_tibia_90.csv"], 2, True),
       "femur_tp": (["femur_rampa (3).csv", "femur_tibia_recogida.csv"], 2, True)}
R = {k: ident(*v) for k, v in IDS.items()}
M["identificacion"] = {k: {a: float(b) for a, b in v[0].items()} for k, v in R.items()}
fig, ax = plt.subplots(1, 2, figsize=(7, 3.1)); P, X, y, th, w, c = R["femur_t0"]; s = slice(None, None, 4)
ax[0].plot(w[s], (y-c[0]*X[:, 0]-c[3]*np.cos(th)-c[4]*np.sin(th))[s], ".", ms=2, color=VI, alpha=.35)
for lado in (-1, 1):
    wl = np.linspace(.3, max(abs(w.min()), w.max()), 50)*lado; ax[0].plot(wl, c[1]*lado+c[2]*wl, color=NA, lw=2)
ax[0].set_xlabel("Velocidad (rad/s)"); ax[0].set_ylabel("I sin inercia ni gravedad (A)"); ax[0].set_title("Fricción", loc="left", fontsize=9.5)
ax[1].plot(th[s], (y-X[:, :3]@c[:3])[s], ".", ms=2, color=VI, alpha=.35); tl = np.linspace(th.min(), th.max(), 200)
ax[1].plot(tl, P["G"]*np.cos(tl+P["th0"]), color=NA, lw=2); ax[1].set_xlabel("Ángulo del fémur (rad)"); ax[1].set_ylabel("I sin inercia ni fricción (A)"); ax[1].set_title("Gravedad", loc="left", fontsize=9.5)
fig.tight_layout(); guardar(fig, "fig_ident_femur")
fig, ax = plt.subplots(1, 2, figsize=(7, 2.9)); x = np.arange(3); ant = [.046, .041, .059]; des = [.0604, .0123, .0130]
ax[0].bar(x-.19, ant, .36, color=GR, label="Pata anterior (11-nov)"); ax[0].bar(x+.19, des, .36, color=AZ, label="Pata modificada (19-nov)")
for i in range(3):
    for dx, v in ((-.19, ant[i]), (.19, des[i])): ax[0].text(i+dx, v+.001, f"{v:.3f}".replace(".", ","), ha="center", fontsize=7.5)
ax[0].set_xticks(x); ax[0].set_xticklabels(["Coxa", "Fémur", "Tibia"]); ax[0].set_ylabel("Fricción de Coulomb Ic (A)"); ax[0].legend(fontsize=7, loc="upper center", bbox_to_anchor=(.5, 1.3)); ax[0].grid(axis="x", visible=False)
g = [R[k][0]["G"] for k in ("femur_tm", "femur_t0", "femur_tp")]; ax[1].bar(range(3), g, .5, color=VI)
for i, v in enumerate(g): ax[1].text(i, v+.002, f"{v:.3f}".replace(".", ","), ha="center", fontsize=8)
ax[1].set_xticks(range(3)); ax[1].set_xticklabels(["−1,55", "0", "+1,55"]); ax[1].set_xlabel("Ángulo de la tibia (rad)"); ax[1].set_ylabel("Gravedad del fémur G (A)"); ax[1].grid(axis="x", visible=False)
fig.tight_layout(); guardar(fig, "fig_friccion_gravedad")

# 8 lazos con las ganancias guardadas en la placa el 21-nov
GAN = {1: dict(a=.00292, J=.00131, b=0., ki=80161.5, kpw=.01883, kiw=.08187, kpp=3.680), 2: dict(a=.00268, J=.00164, b=.0059, ki=87340.2, kpw=.01989, kiw=.11356, kpp=3.009),
       3: dict(a=.00318, J=.00105, b=0., ki=73607.4, kpw=.01509, kiw=.06562, kpp=3.680)}
f = np.logspace(-2, 2.6, 6000); jw = 2j*np.pi*f
def cruce(L):
    mag = np.abs(L); i = np.where((mag[:-1] >= 1) & (mag[1:] < 1))[0]
    if not len(i): return None
    i = i[-1]; return float(f[i]), float(180+np.degrees(np.unwrap(np.angle(L)))[i])
fig, ax = plt.subplots(2, 1, figsize=(7, 4.6), sharex=True); M["lazos"] = {}
for m, g in GAN.items():
    Li = g["a"]*g["ki"]*.001/(np.exp(jw*.001)-1); Li[f > 450] = np.nan
    Lw = (g["kpw"]+g["kiw"]/jw)/(g["J"]*jw+g["b"])*np.exp(-jw*.019); Tw = Lw/(1+Lw); Lp = g["kpp"]/jw*Tw*np.exp(-jw*.010)
    M["lazos"][NOM[m]] = {"corriente": cruce(Li), "velocidad": cruce(Lw), "posicion": cruce(Lp)}
m, g = 2, GAN[2]
Li = g["a"]*g["ki"]*.001/(np.exp(jw*.001)-1); Lw = (g["kpw"]+g["kiw"]/jw)/(g["J"]*jw+g["b"])*np.exp(-jw*.019); Lp = g["kpp"]/jw*(Lw/(1+Lw))*np.exp(-jw*.010)
for L, c, n, fmax in ((Li, AZ, "Corriente", 400), (Lw, VE, "Velocidad", 50), (Lp, VI, "Posición", 25)):
    k = f < fmax; fc, mf = cruce(L); ax[0].plot(f[k], 20*np.log10(np.abs(L[k])), color=c, lw=1.6, label=f"{n}: {fc:.2f} Hz, MF {mf:.0f}°".replace(".", ",")); ax[1].plot(f[k], np.degrees(np.unwrap(np.angle(L)))[k], color=c, lw=1.6)
    ax[0].plot(fc, 0, "o", color=c, ms=5); ax[1].plot(fc, mf-180, "o", color=c, ms=5)
ax[0].axhline(0, color=GR, lw=.9); ax[1].axhline(-180, color=GR, lw=.9, ls=":"); ax[0].set_xscale("log"); ax[0].set_ylim(-40, 60); ax[1].set_ylim(-300, 0); ax[0].set_ylabel("Magnitud (dB)"); ax[1].set_ylabel("Fase (°)"); ax[1].set_xlabel("Frecuencia (Hz)"); ax[0].legend()
guardar(fig, "fig_bode_femur")

# 9 CAN: latencia medida
fig, ax = plt.subplots(figsize=(7, 2.5)); et = ["Firmware de prueba\n(1000 pings)", "pata_MOD_DRV v1\n(6 pings sueltos)", "pata_MOD_DRV v1\n(1000 pings seguidos)"]
mn = [.255, 8.278, 4.217]; pr = [.263, np.mean([8.283, 8.300, 8.312, 8.278, 8.320, 8.288]), 9.989]; mx = [1.656, 8.320, 10.014]
for i in range(3):
    ax.plot([mn[i], mx[i]], [i, i], color=GR, lw=2); ax.plot(pr[i], i, "o", color=AZ, ms=7); ax.text(pr[i], i-.28, f"{pr[i]:.3g} ms".replace(".", ","), ha="center", fontsize=8)
ax.set_xscale("log"); ax.set_xticks([.25, .5, 1, 2, 5, 10]); ax.set_xticklabels(["0,25", "0,5", "1", "2", "5", "10"]); ax.minorticks_off(); ax.set_yticks(range(3)); ax.set_yticklabels(et); ax.invert_yaxis(); ax.set_ylim(2.5, -.6); ax.set_xlabel("Ida y vuelta (ms, escala logarítmica): mínimo–máximo y promedio"); guardar(fig, "fig_can_latencia")

# 10 CAN: ordenes absolutas del 21-nov (LLEGO de la maestra)
O = [("f 0,8", .8, .819, .7), ("f 0,4", .4, .417, 1.4), ("f 0", 0, .016, 1.4), ("ft 0,5 0,2\n(fémur)", .5, .496, 1.5), ("ft 0,9 0\n(fémur)", .9, .911, 1.1),
     ("t 0,3", .3, .294, .8), ("t 0", 0, .016, 1.1), ("ft 0,5 0,2\n(tibia)", .2, .197, .8), ("ft 0,9 0\n(tibia)", 0, -.002, .8)]
fig, ax = plt.subplots(figsize=(7, 2.9)); c = [VI]*5+[VE]*4; e = [(q-r)*1000 for _, r, q, _ in O]; ax.bar(range(9), e, .6, color=c)
for i, v in enumerate(e): ax.text(i, v+(1 if v >= 0 else -2.6), f"{v:+.0f}", ha="center", fontsize=8)
ax.axhspan(-20, 20, color="#eef1f4", zorder=0); ax.axhline(0, color=GR, lw=.8); ax.set_xticks(range(9)); ax.set_xticklabels([o[0] for o in O], fontsize=7.5); ax.set_ylim(-26, 26); ax.set_ylabel("Error al llegar (mrad)"); ax.grid(axis="x", visible=False); guardar(fig, "fig_can_ordenes")
M["can_ordenes"] = dict(error_max_mrad=max(abs(v) for v in e), t_min=min(o[3] for o in O), t_max=max(o[3] for o in O))

# 11 evolucion
fig, ax = plt.subplots(1, 2, figsize=(7, 2.9)); x = np.arange(3)
ta = [2.51, 1.44, 1.76]; tn = [M["cox"]["t_media"], M["fem_c"]["t_media"], M["tib_c"]["t_media"]]; sa = [14, 0, 14]; sn = [M["cox"]["sob_max"], M["fem_c"]["sob_max"], M["tib_c"]["sob_max"]]
for a, v1, v2, yl in ((ax[0], ta, tn, "Tiempo medio de llegada (s)"), (ax[1], sa, sn, "Sobrepaso máximo (%)")):
    a.bar(x-.19, v1, .36, color=GR, label="12-nov (pata anterior)"); a.bar(x+.19, v2, .36, color=AZ, label="19–20-nov (pata actual)"); a.set_xticks(x); a.set_xticklabels(["Coxa", "Fémur", "Tibia"]); a.set_ylabel(yl); a.grid(axis="x", visible=False)
    for i in range(3):
        for dx, v in ((-.19, v1[i]), (.19, v2[i])): a.text(i+dx, v+.02*max(v1), (f"{v:.2f}" if a is ax[0] else f"{v:.1f}").replace(".", ","), ha="center", fontsize=7.5)
fig.tight_layout(); fig.legend(*ax[0].get_legend_handles_labels(), loc="lower center", bbox_to_anchor=(.5, 1), ncol=2, fontsize=8); guardar(fig, "fig_evolucion")

# 12 gravedad calibrada
fig, ax = plt.subplots(figsize=(7, 2.9)); q = np.linspace(-1.2, 1.2, 200)
for qt, ls in ((-1, ":"), (0, "-"), (.7, "--")): ax.plot(q, .0677*np.cos(q-3.1)+.0102*np.cos(q+qt-2.968), color=VI, ls=ls, lw=1.7, label=f"Tibia en {qt:+.1f} rad".replace(".", ","))
ax.set_xlabel("Ángulo del fémur (rad); 0 = horizontal, positivo = baja"); ax.set_ylabel("Corriente de gravedad (A)"); ax.legend(); guardar(fig, "fig_gravedad_modelo")

# 13 carga del bus
def bits(n): return 47+8*n, 47+8*n+(34+8*n-1)//4
M["can_bits"] = {str(n): dict(bits=bits(n)[0], bits_max=bits(n)[1], us=bits(n)[0]*2, us_max=bits(n)[1]*2) for n in (0, 1, 2, 7, 8)}
car = [(p*(bits(1)[0]+bits(8)[0])*10/5000, p*(bits(1)[1]+bits(8)[1])*10/5000) for p in range(1, 7)]
fig, ax = plt.subplots(figsize=(7, 2.5)); ax.bar(range(1, 7), [x[1] for x in car], .55, color=AZ)
for i, x_ in enumerate(car): ax.text(i+1, x_[1]+.05, f"{x_[1]:.1f} %".replace(".", ","), ha="center", fontsize=8)
ax.set_xlabel("Patas con latido (ESTADO cada 100 ms)"); ax.set_ylabel("Ocupación del bus (%)"); ax.set_ylim(0, 3); ax.grid(axis="x", visible=False); guardar(fig, "fig_can_carga")
M["can_carga"] = car
with open(os.path.join(AQUI, "metricas.json"), "w", encoding="utf-8") as fo: json.dump(M, fo, indent=1, ensure_ascii=False)
for k in PR: print(k, {a: (round(b, 4) if isinstance(b, float) else b) for a, b in M[k].items() if a != "pasos"})
print(json.dumps({k: M[k] for k in ("identificacion", "lazos", "can_ordenes", "can_bits", "can_carga")}, indent=0))

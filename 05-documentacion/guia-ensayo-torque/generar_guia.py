"""Genera guia_ensayo_torque_N20.pdf: gráficos (matplotlib) + HTML + Chrome/Edge sin ventana.

Uso: python generar_guia.py
Los datos de los gráficos son SINTÉTICOS (motor de referencia), no mediciones.
"""
import io, math, subprocess, sys, tempfile
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

AQUI = Path(__file__).resolve().parent
AZ, NA, GR, VE, RO = "#1f5fa8", "#d9730d", "#5b6470", "#2e7d4f", "#b3261e"
plt.rcParams.update({"font.family": "Segoe UI", "font.size": 10, "svg.fonttype": "path",
                     "axes.spines.top": False, "axes.spines.right": False,
                     "axes.grid": True, "grid.color": "#e3e6ea", "axes.axisbelow": True})


def coma(x, n=2):
    return f"{x:.{n}f}".replace(".", ",")


def svg(fig):
    from matplotlib.ticker import FuncFormatter
    for ax in fig.axes:
        for e in (ax.xaxis, ax.yaxis):
            if not ax.get_xticklabels()[0].get_text().startswith("6 patas") or e is ax.yaxis:
                e.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}".replace(".", ",")))
    b = io.StringIO()
    fig.savefig(b, format="svg", bbox_inches="tight")
    plt.close(fig)
    t = b.getvalue()
    return t[t.index("<svg"):]


# ------------------------------------------------ motor de referencia (supuesto)
TS, IS, I0, N0, R = 1.30, 0.75, 0.05, 300.0, 16.0   # kg·cm, A, A, rpm, ohm
KGCM = 0.0980665                                     # N·m por kg·cm

# Ensayo A sintético: tau = a V - b, con dispersión fija
a0, b0 = 0.1167, 0.10
Vn = np.array([3, 4.5, 6, 7.5])
V = np.repeat(Vn, 5)
ruido = np.array([.012, -.010, .004, -.015, .008, -.011, .014, -.004, .009, -.007,
                  .015, -.012, .006, .002, -.013, -.006, .013, -.015, .010, .001])
masa = np.round((a0 * V - b0 + ruido) / 10 * 1000)   # g con r = 10 cm, balanza de 1 g
tau = masa / 1000 * 10                               # kg·cm
n = len(V)
a, c = np.polyfit(V, tau, 1)
s = math.sqrt(np.sum((tau - a * V - c) ** 2) / (n - 2))
Sxx = np.sum((V - V.mean()) ** 2)
tau12 = a * 12 + c
u12 = 2.101 * s * math.sqrt(1 / n + (12 - V.mean()) ** 2 / Sxx)   # t(0,975; 18)
medias = [masa[V == v].mean() for v in Vn]

f, ax = plt.subplots(figsize=(6.6, 3.0))
x = np.linspace(0, 12.6, 50)
banda = 2.101 * s * np.sqrt(1 / n + (x - V.mean()) ** 2 / Sxx)
m = x <= 7.5
ax.plot(x[m], a * x[m] + c, color=AZ)
ax.plot(x[x >= 7.4], a * x[x >= 7.4] + c, "--", color=AZ)
ax.fill_between(x, a * x + c - banda, a * x + c + banda, color=AZ, alpha=.15, lw=0)
ax.plot(V, tau, "o", ms=4, color=NA)
ax.plot([12], [tau12], "D", color=RO)
ax.annotate(f"Bloqueo a 12 V:\n{coma(tau12)} ± {coma(u12)} kg·cm", (12, tau12), (8.1, 1.25), color=RO)
ax.text(5.5, .22, "puntos medidos", color=NA)
ax.text(9.4, .72, "extrapolación", color=AZ)
ax.set(xlabel="Voltaje en bornes (V)", ylabel="Torque (kg·cm)", xlim=(0, 13), ylim=(-.15, 1.6))
G_A = svg(f)

# Ensayo B
tl = np.linspace(0, TS, 50)
f, (p, q) = plt.subplots(1, 2, figsize=(7.2, 2.8))
REFF = 1.55
mb = np.array([0, 100, 200, 300, 400])
tb = mb / 1000 * REFF
p.plot(tl, N0 * (1 - tl / TS), color=AZ); p.plot(tb, N0 * (1 - tb / TS), "o", color=NA)
p.axvspan(TS / 2, TS, color=RO, alpha=.07, lw=0); p.text(.72, 250, "no ensayar\n(>50 %)", color=RO)
p.set(xlabel="Torque (kg·cm)", ylabel="Velocidad (RPM)", ylim=(0, 330), xlim=(0, 1.35))
q.plot(tl, I0 + (IS - I0) * tl / TS, color=AZ); q.plot(tb, I0 + (IS - I0) * tb / TS, "o", color=NA)
q.axvspan(TS / 2, TS, color=RO, alpha=.07, lw=0)
q.set(xlabel="Torque (kg·cm)", ylabel="Corriente (A)", ylim=(0, .8), xlim=(0, 1.35))
f.tight_layout(); G_B = svg(f)
filasB = ""
for g in mb[1:]:
    t = g / 1000 * REFF; w = N0 * (1 - t / TS) * 2 * math.pi / 60; i = I0 + (IS - I0) * t / TS
    vv = w * REFF / 100; po = t * KGCM * w; pe = 12 * i
    filasB += (f"<tr><td>{g}</td><td>{coma(t,3)}</td><td>{coma(0.8/vv)}</td><td>{coma(w*60/2/math.pi,0)}</td>"
               f"<td>{coma(i,3)}</td><td>{coma(po)}</td><td>{coma(pe)}</td><td>{coma(100*po/pe,0)} %</td></tr>")

# Ensayo C (térmico ilustrativo)
f, ax = plt.subplots(figsize=(6.6, 2.7))
tt = np.linspace(0, 10, 100)
filasC = ""
for t, col in ((.30, VE), (.45, AZ), (.60, NA)):
    i = (t - c) / (a * R); P = i * i * R
    T = 25 + 30 * P * (1 - np.exp(-tt / 5))
    ax.plot(tt, T, color=col); ax.text(10.1, T[-1] - (5 if t == .45 else 0), f"{coma(t)} kg·cm", color=col, va="center")
    filasC += f"<tr><td>{coma(t)}</td><td>{coma(i,3)}</td><td>{coma(i*R,1)}</td><td>{coma(P,1)}</td></tr>"
ax.axhline(60, color=RO, ls="--"); ax.text(.1, 61.5, "cortar a 60 °C", color=RO)
ax.set(xlabel="Tiempo (min)", ylabel="Carcasa (°C)", xlim=(0, 11.9), ylim=(20, 90))
G_C = svg(f)

# Decisión
f, ax = plt.subplots(figsize=(6.8, 2.9))
lab = ["6 patas", "Trípode\nideal (1/3)", "Trípode, pata\nmedia (41 %)"]
v1 = [1 * k * 5.56 for k in (1 / 6, 1 / 3, .414)]
v2 = [1 * k * 8.77 for k in (1 / 6, 1 / 3, .414)]
xx = np.arange(3)
for d, v, col, l in ((-.19, v1, AZ, "pie bajo la rodilla (d = 5,56 cm)"), (.19, v2, NA, "postura 30°/70° (d = 8,77 cm)")):
    ax.bar(xx + d, v, .36, color=col, label=l)
    for k, y in enumerate(v): ax.text(k + d, y + .05, coma(y), ha="center")
ax.axhline(TS, color=RO, ls="--"); ax.text(2.42, TS + .06, "bloqueo 1,3\n(referencia)", color=RO, ha="left")
ax.set_xticks(xx, lab); ax.set(ylabel="Torque del fémur (kg·cm)", xlim=(-.5, 3.05), ylim=(0, 4.2))
ax.legend(frameon=False, loc="upper left"); ax.grid(axis="x", visible=False)
G_D = svg(f)

# ------------------------------------------------ esquemas
DEF = f'''<defs><marker id="fl" markerUnits="userSpaceOnUse" markerWidth="12" markerHeight="12" refX="7" refY="4" orient="auto-start-reverse"><path d="M0,0 L8,4 L0,8z" fill="{RO}"/></marker>
<marker id="fg" markerUnits="userSpaceOnUse" markerWidth="12" markerHeight="12" refX="7" refY="4" orient="auto-start-reverse"><path d="M0,0 L8,4 L0,8z" fill="{GR}"/></marker></defs>'''
S_A = f'''<svg viewBox="0 0 680 255" class="esq">{DEF}
<rect x="20" y="225" width="640" height="8" fill="#c8cdd3"/>
<rect x="95" y="112" width="70" height="113" fill="#e4e8ed" stroke="{GR}"/><text x="130" y="195" text-anchor="middle">soporte</text>
<rect x="95" y="62" width="70" height="50" rx="6" fill="#b9c6d6" stroke="{GR}"/><circle cx="130" cy="87" r="9" fill="#fff" stroke="{GR}" stroke-width="2"/>
<text x="88" y="50" text-anchor="end">N20 fijo</text>
<rect x="121" y="81" width="330" height="12" rx="3" fill="{NA}" opacity=".85"/><circle cx="130" cy="87" r="5" fill="#fff" stroke="{GR}"/>
<text x="300" y="74" text-anchor="middle">palanca rígida y horizontal</text>
<path d="M433 93 L447 93 L440 155z" fill="{GR}"/><text x="425" y="130" text-anchor="end">punta de contacto</text>
<rect x="375" y="155" width="130" height="12" fill="#b9c6d6" stroke="{GR}"/><rect x="385" y="167" width="110" height="58" fill="#e4e8ed" stroke="{GR}"/>
<rect x="405" y="182" width="70" height="24" fill="#fff" stroke="{GR}"/><text x="440" y="199" text-anchor="middle" font-weight="700">60 g</text>
<line x1="130" y1="20" x2="440" y2="20" stroke="{GR}" marker-start="url(#fg)" marker-end="url(#fg)"/><text x="285" y="14" text-anchor="middle">r = 100 mm (eje → punta)</text>
<line x1="130" y1="22" x2="130" y2="40" stroke="{GR}" stroke-dasharray="3"/><line x1="440" y1="22" x2="440" y2="80" stroke="{GR}" stroke-dasharray="3"/>
<path d="M105 100 A 32 32 0 0 1 152 64" fill="none" stroke="{RO}" stroke-width="2.5" marker-end="url(#fl)"/><text x="78" y="92" fill="{RO}" font-weight="700">τ</text>
<line x1="535" y1="90" x2="535" y2="150" stroke="{RO}" stroke-width="2.5" marker-end="url(#fl)"/><text x="546" y="150" fill="{RO}" font-weight="700">F = m·g</text></svg>'''
S_E = f'''<svg viewBox="0 0 680 150" class="esq">
<rect x="20" y="35" width="130" height="85" rx="6" fill="#e4e8ed" stroke="{GR}"/><text x="85" y="70" text-anchor="middle">Fuente DC</text><text x="85" y="90" text-anchor="middle">límite 0,6 A</text>
<line x1="150" y1="55" x2="265" y2="55" stroke="{RO}" stroke-width="2.5"/><text x="205" y="48" text-anchor="middle">+</text>
<rect x="265" y="33" width="130" height="44" rx="6" fill="#fff" stroke="{AZ}"/><text x="330" y="60" text-anchor="middle">Amperímetro (serie)</text>
<line x1="395" y1="55" x2="540" y2="55" stroke="{RO}" stroke-width="2.5"/>
<line x1="150" y1="105" x2="540" y2="105" stroke="#222" stroke-width="2.5"/><text x="205" y="99" text-anchor="middle">−</text>
<rect x="425" y="64" width="88" height="32" rx="6" fill="#fff" stroke="{VE}"/><text x="469" y="85" text-anchor="middle">Voltímetro</text>
<line x1="469" y1="55" x2="469" y2="64" stroke="{VE}" stroke-width="2"/><line x1="469" y1="96" x2="469" y2="105" stroke="{VE}" stroke-width="2"/>
<rect x="540" y="38" width="120" height="84" rx="6" fill="#b9c6d6" stroke="{GR}"/><text x="600" y="76" text-anchor="middle">Motor N20</text><text x="600" y="96" text-anchor="middle">2 cables M1–M2</text></svg>'''
S_B = f'''<svg viewBox="0 0 680 290" class="esq">{DEF}
<rect x="40" y="55" width="225" height="14" fill="#c8cdd3"/><rect x="40" y="69" width="16" height="206" fill="#c8cdd3"/><text x="66" y="50">mesa o soporte fijo (≥ 1,4 m)</text>
<rect x="40" y="55" width="275" height="14" fill="#c8cdd3"/><rect x="282" y="69" width="36" height="36" rx="5" fill="#b9c6d6" stroke="{GR}"/><text x="262" y="93" text-anchor="end">motor, detrás de la polea</text>
<circle cx="300" cy="87" r="30" fill="{NA}" opacity=".85"/><circle cx="300" cy="87" r="4" fill="#fff"/>
<line x1="300" y1="87" x2="321" y2="66" stroke="#fff" stroke-width="2"/><text x="338" y="70">r polea = 15 mm</text>
<line x1="330" y1="87" x2="330" y2="200" stroke="#222" stroke-width="2"/><rect x="308" y="200" width="44" height="38" fill="#8b96a3" stroke="{GR}"/><text x="330" y="224" text-anchor="middle" fill="#fff">m</text>
<line x1="385" y1="230" x2="385" y2="130" stroke="{RO}" stroke-width="2.5" marker-end="url(#fl)"/><text x="395" y="185" fill="{RO}" font-weight="700">v</text>
<line x1="450" y1="110" x2="450" y2="185" stroke="{GR}" marker-start="url(#fg)" marker-end="url(#fg)"/><text x="460" y="152">h = 0,80 m cronometrados</text>
<line x1="430" y1="110" x2="470" y2="110" stroke="{VE}" stroke-width="3"/><text x="476" y="114">marca 2</text><line x1="430" y1="185" x2="470" y2="185" stroke="{VE}" stroke-width="3"/><text x="476" y="189">marca 1 (0,20 m tras el arranque)</text>
<rect x="265" y="258" width="130" height="17" rx="5" fill="#f2d9b8" stroke="{NA}"/><text x="403" y="272">espuma</text>
<rect x="20" y="275" width="640" height="8" fill="#c8cdd3"/></svg>'''
S_F = f'''<svg viewBox="0 0 680 92" class="esq">{DEF}''' + "".join(
    f'<rect x="{x}" y="14" width="140" height="62" rx="8" fill="{c}"/><text x="{x+70}" y="40" text-anchor="middle" fill="#fff" font-weight="700">{t}</text><text x="{x+70}" y="60" text-anchor="middle" fill="#fff">{u}</text>'
    + (f'<line x1="{x+142}" y1="45" x2="{x+168}" y2="45" stroke="{GR}" stroke-width="2" marker-end="url(#fg)"/>' if x < 500 else "")
    for x, t, u, c in ((10, "0 · Preparar", "R, RPM, I en vacío", GR), (180, "A · Bloqueo", "palanca y balanza", AZ),
                       (350, "B · Pesas", "torque–velocidad", NA), (520, "C · Sostener", "límite térmico", VE))) + "</svg>"

# ------------------------------------------------ contenido
ej = medias[2]
tau_ej = ej / 1000 * 9.81 * .1
pag = []
pag.append(f'''<div class="cab">Unidad 2 · Tesis de Alfredo Erazo, UTEC</div>
<h1>¿Cuánto torque da de verdad el motor N20?</h1><p class="sub">Guía paso a paso del ensayo · N20 12 V, 1:100, 300 RPM (AliExpress)</p>
<p>No hay un datasheet verificable de este motor y las fichas de motores parecidos se contradicen. Esta guía mide el torque de salida directamente: <b>fuerza × brazo</b>. No usa el sensado de corriente de la MOD_DRV, que no está calibrado.</p>
{S_F}
<table><tr><th>Ensayo</th><th>Pregunta que responde</th><th>Motores</th><th>Tiempo por motor</th></tr>
<tr><td>0 Preparar</td><td>¿El motor está sano y es el esperado?</td><td>los 18</td><td>15 min</td></tr>
<tr><td>A Bloqueo</td><td>¿Cuál es el máximo?</td><td>los 18 (empieza con 1)</td><td>30–45 min por sentido</td></tr>
<tr><td>B Pesas</td><td>¿Cuánto da en movimiento y con qué eficiencia?</td><td>3</td><td>45 min</td></tr>
<tr><td>C Sostener</td><td>¿Cuánto aguanta sin calentarse demasiado?</td><td>1–2</td><td>1–2 h, con enfriado</td></tr></table>
<div class="rojo"><b>Seguridad: corta la fuente de inmediato si…</b><ul><li>la corriente supera 0,6 A o la fuente entra en límite en el ensayo A o B;</li><li>la carcasa pasa de 60 °C, hay olor o humo;</li><li>la caja salta, cruje o el eje patina.</li></ul>Usa gafas. No pongas las manos entre la palanca y la balanza ni debajo de la pesa. Nunca bloquees el motor a 12 V.</div>
<div class="nota"><b>Cómo leer los números.</b> Los gráficos y ejemplos son <b>sintéticos</b>: usan un motor de referencia supuesto (bloqueo ≈ {coma(TS,1)} kg·cm, 0,75 A, 16 Ω). Sirven para reconocer un buen resultado; <b>no son mediciones</b> y no deben citarse como tales. Los voltajes, tiempos y límites son una propuesta de ensayo, no una norma.</div>''')

pag.append(f'''<h2>1 · La idea en cuatro ecuaciones</h2>
<p><b>1. Torque = fuerza por brazo.</b> La balanza muestra masa; se convierte en fuerza con la gravedad.</p><div class="eq">τ = m · g · r &nbsp;&nbsp;→&nbsp;&nbsp; τ [kg·cm] = m [kg] · r [cm]</div>
<p class="ej">Ejemplo: 60 g a 10 cm → 0,060 × 10 = <b>0,60 kg·cm</b> = 0,060 × 9,81 × 0,10 = <b>0,0589 N·m</b>. (1 kg·cm = 0,0981 N·m)</p>
<p><b>2. Bloqueado, la corriente la fija la resistencia.</b> Sin giro no hay fuerza contraelectromotriz.</p><div class="eq">I<sub>bloqueo</sub> = V / R</div>
<p class="ej">Ejemplo: 6 V / 16 Ω = <b>0,375 A</b>; a 12 V serían 0,75 A.</p>
<p><b>3. El torque de bloqueo crece en línea recta con el voltaje</b>, menos lo que se come la fricción.</p><div class="eq">τ(V) = a · V − b</div>
<p>Por eso basta medir a voltajes bajos y extrapolar a 12 V, sin castigar la caja. La fricción <i>b</i> hace que multiplicar por 2 la lectura de 6 V dé de menos.</p>
<p><b>4. En movimiento, la velocidad cae en línea recta con el torque.</b></p><div class="eq">n(τ) = n<sub>0</sub> · (1 − τ / τ<sub>bloqueo</sub>)</div>
<p class="ej">Ejemplo: con 0,465 kg·cm y bloqueo de 1,3 → 300 × (1 − 0,358) = <b>193 RPM</b>.</p>
<h3>Control de sentido físico: ¿es creíble una ficha?</h3>
<p>Lo máximo que puede dar el motor, sin ninguna pérdida, sale de su constante y la reducción N = 100:</p>
<div class="eq">K<sub>e</sub> ≈ 12 V / (300 RPM × 100 × 2π/60) = 12 / 3142 = 3,8 mN·m/A</div>
<div class="eq">τ<sub>máx ideal</sub> = K<sub>e</sub> · I · N = 0,0038 × 0,3 × 100 = 0,115 N·m ≈ 1,2 kg·cm</div>
<p>Una ficha que promete 4 kg·cm con 0,3 A <b>es imposible</b>. Descontando la corriente de vacío, el techo baja a ~1,1. Aplica el mismo control a tu propio resultado: si lo supera, revisa el brazo, la tara o la corriente.</p>''')

pag.append(f'''<h2>2 · Materiales y preparación</h2>
<table><tr><th>Necesitas</th><th>Detalle</th></tr>
<tr><td>Fuente de laboratorio</td><td>0–12 V con límite de corriente</td></tr><tr><td>2 multímetros</td><td>uno en serie (A, entrada correcta) y otro en bornes (V)</td></tr>
<tr><td>Balanza</td><td>mejor 0,1 g; sirve 1 g. O celda de carga + HX711</td></tr><tr><td>Palanca</td><td>rígida, 100 mm del eje a la punta, fijada a la brida del eje D</td></tr>
<tr><td>Soporte</td><td>abrazadera que sujete la <b>caja metálica</b>, sin aplastar el motor</td></tr><tr><td>Polea e hilo (B)</td><td>radio 15 mm, ancho ≥ 15 mm; hilo de 1 mm; pesas 100–400 g; gancho</td></tr>
<tr><td>Otros</td><td>termopar, regla o calibre, celular (video a 60 fps), espuma, gafas</td></tr></table>
<h3>Conexión: directa a la fuente, sin la MOD_DRV</h3>{S_E}
<p>Usa solo los dos cables de potencia del motor (M1 y M2). <b>Identifícalos antes</b>: el orden cambia según el vendedor. El encoder queda sin conectar. No apliques el voltaje del motor a los cables del encoder.</p>
<h3>Pasos (15 min)</h3><ol><li><b>Numera el motor</b> y anota la temperatura ambiente.</li>
<li><b>Mide R</b> desenergizado, entre M1 y M2, en 6 posiciones del eje. Quédate con la mediana y resta la de las puntas. <span class="ok">Esperado: ~14–17 Ω para 0,70–0,85 A de bloqueo.</span> Si da ~40 Ω, el motor es de ~0,3 A. Si marca abierto, revisa que no sean los cables del encoder.</li>
<li><b>Gira en vacío a 12 V</b> 10 s en cada sentido. Mide V en bornes, I y RPM (marca en el eje y video en cámara lenta). <span class="ok">Esperado: ~300 RPM y ~0,05 A.</span> Si difiere más de 20 %, comprueba el voltaje en bornes y que el eje esté libre.</li>
<li><b>Mide el brazo r</b> tres veces, desde el centro del eje hasta la punta. Anota la media. <span class="ok">Esperado: 100 ± 0,5 mm.</span></li></ol>
<p class="ej">Cálculo: lecturas 16,4 / 15,9 / 16,3 / 16,1 / 18,0 / 16,2 Ω → mediana 16,25 Ω; puntas 0,25 Ω → <b>R = 16,0 Ω</b>.</p>''')

pag.append(f'''<h2>3 · Ensayo A: torque de bloqueo</h2>{S_A}
<h3>Pasos</h3><ol><li><b>Fija el motor</b> y coloca la palanca horizontal (±5°) con la punta sobre la balanza. Gira la fuente en el sentido que hace <b>bajar</b> la palanca. <i>Si sube, invierte M1 y M2 con la fuente apagada.</i></li>
<li><b>Levanta y apoya</b> la palanca con suavidad, espera 5 s y <b>tara</b>. <i>Hazlo antes de cada pulso:</i> la fricción de la caja puede dejar una fuerza atrapada.</li>
<li><b>Ajusta la fuente a 3 V</b> con límite 0,6 A y la salida apagada. Empieza a grabar balanza y multímetros.</li>
<li><b>Enciende 2 s y apaga.</b> Con balanza lenta, hasta 3 s. Anota del video V en bornes, I y la masa en la meseta.</li>
<li><b>Espera 60 s.</b> Repite 5 veces; si tu soporte lo permite, gira un poco el cuerpo del motor entre repeticiones para cargar otros dientes.</li>
<li><b>Repite a 4,5, 6 y 7,5 V.</b> Para 7,5 V espera 120 s. Si la carcasa llega a 45 °C, deja enfriar.</li>
<li><b>Repite en el otro sentido</b>, con la balanza al lado contrario. Analiza cada sentido por separado.</li></ol>
<table><tr><th>V en bornes</th><th>I = V/R (16 Ω)</th><th>Masa esperada</th><th>Torque</th></tr>
{"".join(f"<tr><td>{coma(v,1)} V</td><td>{coma(v/16,3)} A</td><td>~{coma(a0*v*100-b0*100,0)} g</td><td>{coma(a0*v-b0)} kg·cm</td></tr>" for v in Vn)}</table>
<div class="nota"><b>Si algo no sale bien.</b> <b>La balanza marca 0:</b> comprueba el sentido, la tara o que arranque la corriente. <b>Salta de valor:</b> la punta resbala; usa una base antideslizante. <b>La lectura cae durante el pulso:</b> se calienta; pulso más corto y usa el valor inicial estable. <b>I mucho menor que V/R:</b> revisa el contacto o R. <b>La palanca flexa:</b> hazla más rígida.</div>''')

pag.append(f'''<h2>4 · Ensayo A: cálculo con números de ejemplo</h2>
<p><b>Paso 1. Promedia cada voltaje.</b> En este ejemplo, las 5 lecturas a 6 V dan un promedio de {coma(ej,1)} g.</p>
<div class="eq">τ = {coma(ej/1000,4)} kg × 9,81 m/s² × 0,100 m = {coma(tau_ej,4)} N·m = {coma(ej/100,3)} kg·cm</div>
<table><tr><th>V</th><th>Masas leídas (g)</th><th>Media (g)</th><th>τ (kg·cm)</th></tr>
{"".join(f"<tr><td>{coma(v,1)}</td><td>{' · '.join(str(int(k)) for k in masa[V==v])}</td><td>{coma(md,1)}</td><td>{coma(md/100,3)}</td></tr>" for v, md in zip(Vn, medias))}</table>
<p><b>Paso 2. Ajusta la recta</b> con todas las repeticiones (Excel: <code>PENDIENTE</code> e <code>INTERSECCION.EJE</code>).</p>
<div class="eq">a = {coma(a,4)} kg·cm/V &nbsp;&nbsp; b = {coma(-c,3)} kg·cm &nbsp;&nbsp; s = {coma(s,4)} kg·cm</div>
<p><b>Paso 3. Extrapola a 12 V.</b></p><div class="eq">τ<sub>12</sub> = {coma(a,4)} × 12 − {coma(-c,3)} = {coma(tau12)} kg·cm = {coma(tau12*KGCM,3)} N·m</div>
{G_A}
<p class="pie">Datos sintéticos. Banda: intervalo de confianza del 95 % de la recta.</p>
<p><b>Paso 4. Incertidumbre.</b> Con n = {n} puntos y t<sub>0,975;18</sub> = 2,101:</p>
<div class="eq">U = t · s · √( 1/n + (12 − V̄)² / Σ(V<sub>i</sub> − V̄)² ) = {coma(u12,3)} kg·cm</div>
<p>Para una sola lectura de 60 g con balanza de 1 g, u = 1/√12 = 0,29 g si solo cuenta la resolución; con r = 100 ± 0,5 mm:</p>
<div class="eq">u(τ)/τ = √( (0,29/60)² + (0,5/100)² ) ≈ 0,7 %</div>
<p>La banda solo recoge la dispersión: <b>no</b> incluye el error del brazo, el calentamiento ni que la recta deje de serlo cerca de 12 V. Declara el resultado como <i>extrapolado</i>.</p>''')

pag.append(f'''<h2>5 · Ensayo B: levantar pesas</h2>{S_B}
<h3>Pasos</h3><ol><li><b>Calcula tu masa máxima</b>, con 50 % del bloqueo medido en A: <span class="eqi">m<sub>máx</sub> [g] = 1000 · 0,5 · τ<sub>12</sub> / r<sub>ef</sub></span>. Con 1,3 kg·cm: 0,65/1,55 → <b>419 g</b>, así que se usan hasta 400 g.</li>
<li><b>Mide el radio efectivo:</b> <span class="eqi">r<sub>ef</sub> = r<sub>polea</sub> + d<sub>hilo</sub>/2</span> = 15 + 0,5 = <b>15,5 mm</b>. El hilo debe enrollarse en una sola capa (10,3 vueltas por metro).</li>
<li><b>Coloca las marcas:</b> una a 0,20 m del punto de partida y otra 0,80 m más arriba. Pon la cámara a la altura media, lejos, y deja 0,15 m libres antes de la polea.</li>
<li><b>Fija la fuente</b> de modo que haya 12 V <i>en bornes mientras sube</i>, con límite 0,6 A. Cuelga 100 g (incluye el gancho).</li>
<li><b>Enciende, graba y apaga</b> al pasar la marca 2, antes de que la pesa llegue a la polea. La pesa puede caer al apagar: nadie debajo. Anota V e I en el tramo medio.</li>
<li><b>Espera 60 s. Repite 5 veces</b> y sigue con 200, 300 y 400 g (si tu m<sub>máx</sub> es menor, usa 25, 50, 75 y 100 % de ella). La carga 0 sale de la prueba en vacío.</li></ol>
<p><span class="ok">Qué deberías ver:</span> cada subida de 0,80 m dura entre 1,8 y 3,2 s y las repeticiones difieren menos de 5 %. Si la pesa no arranca, apaga de inmediato y baja la carga.</p>
<div class="nota"><b>Error típico:</b> cronometrar desde el reposo mezcla aceleración con velocidad constante. Comprueba en el video que las dos mitades (0,40 m cada una) duran casi lo mismo; si no, sube la marca 1.</div>''')

pag.append(f'''<h2>6 · Ensayo B: cálculo con números de ejemplo</h2>
<p>Ejemplo con <b>300 g</b>, que tarda 2,56 s en subir 0,80 m a 12 V y 0,300 A:</p>
<div class="eq">τ = 0,300 × 9,81 × 0,0155 = 0,0456 N·m = 0,465 kg·cm</div>
<div class="eq">v = 0,80 / 2,56 = 0,313 m/s &nbsp;&nbsp;→&nbsp;&nbsp; ω = v / r<sub>ef</sub> = 0,313 / 0,0155 = 20,2 rad/s = 193 RPM</div>
<div class="eq">P<sub>sal</sub> = τ·ω = 0,0456 × 20,2 = 0,92 W &nbsp;&nbsp; P<sub>ent</sub> = V·I = 12 × 0,300 = 3,60 W</div>
<div class="eq">η = P<sub>sal</sub> / P<sub>ent</sub> = 0,92 / 3,60 = 26 %</div>
<table><tr><th>m (g)</th><th>τ (kg·cm)</th><th>t para 0,8 m (s)</th><th>RPM</th><th>I (A)</th><th>P sal (W)</th><th>P ent (W)</th><th>η</th></tr>{filasB}</table>
{G_B}<p class="pie">Datos sintéticos del motor de referencia. Los puntos naranjas son las cargas de 0 a 400 g.</p>
<h3>Qué hacer con tus datos</h3><ol><li>Ajusta <span class="eqi">n = n<sub>0</sub> − k·τ</span>. El bloqueo estimado es <span class="eqi">n<sub>0</sub>/k</span>: aquí k = 300/1,3 = 231 RPM/(kg·cm) → 1,30 kg·cm.</li>
<li>Compáralo con el ensayo A. <b>Si difieren menos de 15 %, son coherentes.</b> Si difieren más, revisa voltaje en bornes, r<sub>ef</sub> y calentamiento. Ese 15 % es un criterio propuesto.</li>
<li>Ajusta <span class="eqi">I = I<sub>0</sub> + τ/k<sub>τ</sub></span>. Aquí k<sub>τ</sub> = 1,3/0,70 = 1,86 kg·cm/A. Con ella puedes estimar torque desde corriente <b>medida en serie</b>, no desde el CS de la placa.</li>
<li>Si η sale mayor que 60 %, sospecha de un error: es alto para este tipo de motor.</li></ol>''')

pag.append(f'''<h2>7 · Ensayo C: cuánto puede sostener</h2>
<p>Usa el montaje de A. El motor empuja contra la balanza con <b>corriente constante</b> y se vigila la temperatura.</p>
<ol><li><b>Pega el termopar</b> a la carcasa del motor, no a la caja. Espera a temperatura ambiente.</li>
<li><b>Calcula la corriente</b> desde tu recta τ–I del ensayo A: <span class="eqi">I = (τ + b) / (a·R)</span>.</li>
<li><b>Empieza con 0,30 kg·cm.</b> Pon el límite de corriente en ese valor y el de voltaje en 7,5 V, tara y enciende.</li>
<li><b>Anota cada 30 s</b> la masa, V, I y temperatura, durante 10 min. <b>No te alejes.</b> Corta si llega a 60 °C y anota el tiempo.</li>
<li><b>Enfría hasta estar a menos de 3 °C del ambiente</b> (20–30 min) y sigue con 0,45 y 0,60. Detente en el primer punto que corte.</li></ol>
<table><tr><th>τ (kg·cm)</th><th>I (A)</th><th>V inicial (V)</th><th>Calor I²R (W)</th></tr>{filasC}</table>
<p class="ej">Ejemplo para 0,45: I = (0,45 + {coma(-c,3)}) / ({coma(a,4)} × 16) = {coma((.45-c)/(a*R),3)} A; P = I²R = {coma(((.45-c)/(a*R))**2*R)} W.</p>
{G_C}<p class="pie">Curvas ilustrativas: suponen 25 °C, 30 °C/W y constante de tiempo de 5 min. Tu motor puede ser muy distinto.</p>
<h3>Criterio y cálculo</h3><p>Punto <b>sostenible</b> (criterio propuesto): carcasa &lt; 60 °C, subida &lt; 1 °C en los 2 últimos minutos y torque con caída &lt; 10 %. El bobinado está más caliente que la carcasa. La resistencia del cobre permite estimarlo: con corriente constante, el voltaje sube.</p>
<div class="eq">ΔT<sub>bobinado</sub> ≈ (V<sub>final</sub>/V<sub>inicial</sub> − 1) / 0,0039</div>
<p class="ej">Ejemplo: de 4,71 V a 5,25 V con 0,295 A → (1,115 − 1)/0,0039 ≈ <b>29 °C</b> de subida. La resistencia de escobillas lo hace aproximado.</p>''')

pag.append(f'''<h2>8 · ¿Levanta el hexápodo? Cómo decidir</h2>
<p>Torque estático del fémur, con <i>M</i> masa del robot (kg), <i>f</i> fracción del peso en esa pata y <i>d</i> distancia horizontal eje–pie (cm):</p>
<div class="eq">τ<sub>req</sub> [kg·cm] = M · f · d</div>
<p class="ej">1 kg, 6 patas, pie bajo la rodilla: 1 × 1/6 × 5,56 = <b>0,93</b>. Pata media en trípode: 1 × 0,414 × 5,56 = <b>2,30</b> kg·cm. Para 1,2 kg multiplica por 1,2.</p>
{G_D}<p class="pie">Robot de 1 kg. El 41 % sale de las posiciones de cadera del URDF de noviembre; la geometría (d), del modelo de Fusion.</p>
<table><tr><th>Tu resultado</th><th>Interpretación para 1 kg</th></tr>
<tr><td>Bloqueo &lt; 0,93</td><td>No se sostiene con 6 patas ni en la mejor postura evaluada</td></tr>
<tr><td>0,93 – 1,9</td><td>Con 6 patas, postura recogida, muy forzado. Sin factor 2 de margen</td></tr>
<tr><td>1,9 – 4,6</td><td>6 patas con margen ≥ 2 en la mejor postura. Trípode: sin margen suficiente</td></tr>
<tr><td>&gt; 4,6</td><td>Trípode con margen 2 en la mejor postura; falta verificar la resistencia de la caja</td></tr></table>
<p><b>Para uso continuo</b>, compara τ<sub>req</sub> con el resultado de <b>C</b>, no con el bloqueo. Usa el <b>motor más débil</b> de los 18, y reporta media, desviación estándar e intervalo del 95 %.</p>
<div class="nota"><b>Lo que este ensayo no cubre:</b> fricción de las juntas, impactos, reparto desigual del peso, fuerzas horizontales en el pie, desgaste de la caja y su límite mecánico. El peso propio de la pata tampoco está en la fórmula. Al sostener una carga, la fricción de la caja puede ayudar: C es conservador para ese caso.</div>''')

def vacias(k, col): return "".join("<tr>" + "<td>&nbsp;</td>" * col + "</tr>" for _ in range(k))
pag.append(f'''<h2>9 · Hoja de registro (imprimir una por motor)</h2>
<table class="reg"><tr><td>Motor n.º</td><td></td><td>Fecha</td><td></td><td>T ambiente</td><td></td></tr><tr><td>R (Ω)</td><td></td><td>RPM vacío</td><td></td><td>I vacío (A)</td><td></td></tr><tr><td>r palanca (mm)</td><td></td><td>r<sub>ef</sub> (mm)</td><td></td><td>Sentido</td><td></td></tr></table>
<p><b>A · Bloqueo</b></p><table class="reg"><tr><th>V nominal</th><th>V bornes</th><th>I (A)</th><th>m1</th><th>m2</th><th>m3</th><th>m4</th><th>m5</th><th>τ medio</th></tr>{"".join(f"<tr><td>{coma(v,1)}</td>"+"<td></td>"*8+"</tr>" for v in Vn)}</table>
<p><b>B · Pesas</b></p><table class="reg"><tr><th>m (g)</th><th>V</th><th>I (A)</th><th>t1</th><th>t2</th><th>t3</th><th>t4</th><th>t5</th><th>RPM</th><th>η</th></tr>{"".join(f"<tr><td>{g}</td>"+"<td></td>"*9+"</tr>" for g in mb[1:])}</table>
<p><b>C · Sostener</b></p><table class="reg"><tr><th>τ</th><th>I (A)</th><th>V ini</th><th>V fin</th><th>T ini</th><th>T 5 min</th><th>T 10 min</th><th>τ final</th><th>¿Sostenible?</th></tr>{vacias(3,9)}</table>
<p><b>Resultado:</b> τ<sub>12</sub> = ________ ± ________ kg·cm &nbsp;&nbsp; τ continuo = ________ kg·cm</p>
<h3>Fuentes consultadas</h3><p class="fu">Las guías universitarias y artículos se vieron en extractos de búsqueda; ábrelos antes de citarlos.</p><ul class="fu">
<li>Univ. of Utah, ME3200, <i>DC Motor Torque-Speed Curve</i>: my.mech.utah.edu/~me3200/labs/torquespeed.pdf (celda de carga, bloqueo por ajuste)</li>
<li>Univ. of Utah, caracterización de motores: my.mech.utah.edu/~me3200/labs/motorchar.pdf (R, Ke, fricción)</li>
<li>Stanford ME161, Lab 6: web.stanford.edu/class/me161/labs/Lab06MotorConstants.pdf (no prolongar el bloqueo)</li>
<li>Michigan Tech, <i>DC Motor Calculations</i>: pages.mtu.edu/~wjendres/ProductRealization1Course/DC_Motor_Calculations.pdf</li>
<li>Cal Poly, <i>Desktop Dynamometer</i> (proyecto estudiantil): digitalcommons.calpoly.edu/cgi/viewcontent.cgi?article=1995&amp;context=mesp</li>
<li>Pololu 5216 (página leída; otro motor): pololu.com/product/5216 (bloqueo extrapolado; regla general ≤ 25 % de la corriente de bloqueo)</li>
<li>Aslong JGA12-N20 (tabla leída; 4 kg·cm con 0,3 A): aslongdcmotor.com/quality-14376341-3v-6v-12v-n20-small-dc-gear-motors-jga12-n20</li>
<li>Geometría: 03-control/cinematica/README.md</li></ul>''')

CSS = f'''@page{{size:A4;margin:0}}*{{box-sizing:border-box}}body{{margin:0;font:11.3pt/1.42 "Segoe UI",sans-serif;color:#20262d;-webkit-print-color-adjust:exact}}
.page{{width:210mm;height:296.8mm;padding:14mm 17mm 15mm;position:relative;break-after:page;overflow:hidden}}.page:last-child{{break-after:auto}}
h1{{font-size:24pt;line-height:1.15;margin:5mm 0 1mm;color:{AZ}}}h2{{font-size:16pt;margin:0 0 3mm;color:{AZ};border-bottom:2px solid {AZ};padding-bottom:1mm}}h3{{font-size:11.5pt;margin:3mm 0 1mm;color:{AZ}}}
p{{margin:1.6mm 0}}.sub{{color:{GR};font-size:12pt;margin-bottom:5mm}}.cab{{color:{GR}}}ol,ul{{margin:1.5mm 0;padding-left:6mm}}li{{margin:1.1mm 0}}
table{{width:100%;border-collapse:collapse;margin:2mm 0;font-size:10.4pt}}th{{background:{AZ};color:#fff;text-align:left;font-weight:600}}td,th{{border:1px solid #cfd5dc;padding:1mm 2mm}}
.reg{{table-layout:fixed}}.reg td{{height:6.6mm}}.reg th{{font-size:8.6pt;padding:1mm}}.eq{{background:#eef3f9;border-left:4px solid {AZ};padding:1.4mm 4mm;margin:1.5mm 0;font:11pt Cambria,serif}}.eqi{{font-family:Cambria,serif;white-space:nowrap;background:#eef3f9;padding:0 1mm}}
.ej{{color:#374250;background:#fff7ee;border-left:4px solid {NA};padding:1.2mm 4mm}}.nota,.rojo{{padding:2mm 4mm;margin:3mm 0;background:#f1f4f7;border-left:4px solid {GR}}}.rojo{{background:#fdeeed;border-color:{RO}}}
.ok{{color:{VE};font-weight:600}}.pie{{font-size:8.6pt;color:{GR};text-align:center;margin:0 0 2mm}}.fu{{font-size:8.2pt;line-height:1.28}}.fu li{{margin:.3mm 0}}
svg{{display:block;margin:1mm auto;max-width:100%}}svg:not(.esq){{height:auto}}.esq{{width:100%;font:13px "Segoe UI";margin:2mm 0}}.num{{position:absolute;bottom:7mm;left:17mm;right:17mm;font-size:8.5pt;color:{GR};display:flex;justify-content:space-between}}'''
cuerpo = "".join(f'<section class="page">{p}<div class="num"><span>Ensayo de torque · motor N20 · Unidad 2</span><span>{i} / {len(pag)}</span></div></section>' for i, p in enumerate(pag, 1))
html = f'<!doctype html><html lang="es"><meta charset="utf-8"><title>Guía de ensayo de torque del motor N20</title><style>{CSS}</style><body>{cuerpo}'
html += '''<script>let o=[];document.querySelectorAll(".page").forEach((p,i)=>{let n=p.querySelector(".num").getBoundingClientRect().top,m=0;[...p.children].forEach(c=>{if(!c.classList.contains("num"))m=Math.max(m,c.getBoundingClientRect().bottom)});o.push((i+1)+":"+Math.round(n-m))});document.body.dataset.libre=o.join(" ")</script></body></html>'''

if __name__ == "__main__":
    nav = next(p for p in (Path(r"C:\Program Files\Google\Chrome\Application\chrome.exe"),
                           Path(r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe")) if p.exists())
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as d:
        h = Path(d) / "g.html"; h.write_text(html, encoding="utf-8")
        base = [str(nav), "--headless=new", "--disable-gpu", f"--user-data-dir={d}\\perfil"]
        r = subprocess.run(base + ["--dump-dom", h.as_uri()], capture_output=True, text=True, encoding="utf-8", timeout=90)
        k = r.stdout.find('data-libre="')
        print("px libres por página:", r.stdout[k + 12:].split('"')[0] if k >= 0 else "?")
        sal = AQUI / "guia_ensayo_torque_N20.pdf"
        subprocess.run(base + ["--no-pdf-header-footer", f"--print-to-pdf={sal}", h.as_uri()], check=True, capture_output=True, timeout=90)
        print(sal, sal.stat().st_size, "bytes; tau12", round(tau12, 3), "±", round(u12, 3), "a", a, "b", -c)

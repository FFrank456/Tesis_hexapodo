"""Dashboard interactivo (Dash + Plotly): cinemática, trayectorias en 3D, cascada y datos reales.

    python app.py        ->  abrir http://127.0.0.1:8050 en el navegador
"""
import base64
import functools

import numpy as np
import plotly.graph_objects as go
from dash import Dash, Input, Output, State, dcc, html
from plotly.subplots import make_subplots

import experiments as ex
import loop_analysis as la
import real_data as rd
import trajectories as tr
from common import load_config
from kinematics import LegKinematics

CFG = load_config()
KIN = LegKinematics(CFG)
JN = ["Coxa", "Fémur", "Tibia"]
JC = ["#2a78d6", "#eb6834", "#1baf7a"]
REF = "#52514e"
try:
    from mesh_utils import load_leg_meshes, pose_meshes
    MESHES = load_leg_meshes(CFG, KIN)
except Exception:  # noqa: BLE001
    MESHES = None
LAYOUT = dict(template="plotly_white", font=dict(family="Segoe UI, sans-serif", size=12),
              margin=dict(l=50, r=20, t=50, b=40))


def mm(x):
    return np.asarray(x) * 1000


def leg_traces(q, show_mesh=True, name="pata"):
    P = mm(KIN.points(q)[0])
    tr_ = [go.Scatter3d(x=P[:, 0], y=P[:, 1], z=P[:, 2], mode="lines+markers", name=name,
                        line=dict(color="#0b0b0b", width=6), marker=dict(size=4))]
    if show_mesh and MESHES:
        for (V, F), col, n in zip(pose_meshes(MESHES, KIN, q), JC, JN):
            V = mm(V)
            tr_.append(go.Mesh3d(x=V[:, 0], y=V[:, 1], z=V[:, 2], i=F[:, 0], j=F[:, 1], k=F[:, 2],
                                 color=col, opacity=0.45, name=n, hoverinfo="name", showlegend=False))
    return tr_


def scene(extra=None):
    return dict(aspectmode="data", xaxis_title="x [mm]", yaxis_title="y [mm]", zaxis_title="z [mm]",
                camera=dict(eye=dict(x=1.6, y=-1.6, z=1.0)), **(extra or {}))


@functools.lru_cache(maxsize=8)
def run_traj(name):
    out = tr.run(CFG, KIN, name)
    return out, tr.cartesian_metrics(out, CFG)


@functools.lru_cache(maxsize=1)
def steps():
    return ex.all_steps(CFG)


# ------------------------------------------------------------------ layout
lim = KIN.lim_cad
slider = lambda k: dcc.Slider(id=f"q{k}", min=float(np.degrees(lim[k, 0])), max=float(np.degrees(lim[k, 1])),
                              step=0.5, value=round(float(np.degrees(np.array(CFG["trayectorias"]["postura_central_cad_rad"])[k])), 1),
                              marks=None, tooltip={"placement": "bottom", "always_visible": True})
p_c = mm(KIN.fk(np.array(CFG["trayectorias"]["postura_central_cad_rad"])))

app = Dash(__name__, title="IK y cascada · Pata 1")
app.layout = html.Div(style={"fontFamily": "Segoe UI, sans-serif", "maxWidth": "1400px", "margin": "auto"}, children=[
    html.H2("Pata 1 (Unidad 2): cinemática inversa y control en cascada"),
    dcc.Tabs([
        dcc.Tab(label="Cinemática", children=[html.Div(style={"display": "flex", "gap": "20px"}, children=[
            html.Div(style={"width": "32%"}, children=[
                html.H4("Directa (q en el marco de Fusion, °)"),
                *[html.Div([html.Label(n), slider(k)]) for k, n in enumerate(JN)],
                html.H4("Inversa: objetivo del pie (mm)"),
                *[dcc.Input(id=f"p{a}", type="number", value=round(float(v), 1), step=1, style={"width": "30%"})
                  for a, v in zip("xyz", p_c)],
                html.Button("Resolver IK", id="ik_btn", style={"marginTop": "8px"}),
                html.Pre(id="ik_out", style={"background": "#f5f5f3", "padding": "8px"}),
                dcc.Checklist(id="mesh_on", options=[{"label": " mostrar mallas STL (versión 4-nov)", "value": "m"}],
                              value=[]),
            ]),
            dcc.Graph(id="leg3d", style={"width": "68%", "height": "650px"}),
        ])]),
        dcc.Tab(label="Trayectorias", children=[
            dcc.Dropdown(id="traj", options=[{"label": v, "value": k} for k, v in tr.NAMES.items()],
                         value="ciclo_paso", clearable=False, style={"width": "300px", "margin": "10px 0"}),
            html.Div(id="traj_metrics"),
            html.Div(style={"display": "flex"}, children=[
                dcc.Graph(id="traj3d", style={"width": "50%", "height": "600px"}),
                dcc.Graph(id="traj_err", style={"width": "50%", "height": "600px"})]),
            dcc.Graph(id="traj_loops", style={"height": "700px"}),
        ]),
        dcc.Tab(label="Cascada", children=[
            dcc.Dropdown(id="joint", options=[{"label": n, "value": k} for k, n in enumerate(JN)], value=1,
                         clearable=False, style={"width": "200px", "margin": "10px 0"}),
            dcc.Graph(id="steps", style={"height": "450px"}),
            dcc.Graph(id="bode", style={"height": "420px"}),
            html.Div(id="loop_table"),
        ]),
        dcc.Tab(label="Datos reales", children=[
            html.P("Sube un CSV guardado por pata_dashboard.html (formato en el README). Se simula la misma "
                   "secuencia de objetivos y se superpone."),
            dcc.Upload(id="upload", children=html.Button("Elegir CSV"), multiple=False),
            dcc.Dropdown(id="real_joint", options=[{"label": n, "value": k} for k, n in enumerate(JN)], value=1,
                         clearable=False, style={"width": "200px", "margin": "10px 0"}),
            html.Div(id="real_msg"),
            dcc.Graph(id="real_fig", style={"height": "750px"}),
        ]),
    ]),
])


# ------------------------------------------------------------------ callbacks
@app.callback(Output("leg3d", "figure"), Input("q0", "value"), Input("q1", "value"), Input("q2", "value"),
              Input("mesh_on", "value"))
def cb_leg(a, b, c, mesh_on):
    q = np.radians([a, b, c])
    fig = go.Figure(leg_traces(q, bool(mesh_on)))
    p = mm(KIN.fk(q))
    fig.add_trace(go.Scatter3d(x=[p[0]], y=[p[1]], z=[p[2]], mode="markers", marker=dict(size=7, color="#d03b3b"),
                               name=f"pie ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) mm"))
    rel = np.degrees(KIN.rel_knee_angle(q)[0])
    fig.update_layout(**LAYOUT, scene=scene(), title=f"Ángulo relativo fémur–tibia {rel:.1f}° "
                      f"({'¡cerca de singular!' if abs(np.sin(np.radians(rel))) < 0.05 else 'lejos de singularidad'})")
    return fig


@app.callback(Output("ik_out", "children"), Input("ik_btn", "n_clicks"),
              State("px", "value"), State("py", "value"), State("pz", "value"))
def cb_ik(_, x, y, z):
    p = np.array([x, y, z], float) / 1000
    lines = []
    for c in (1, -1):
        for b in (-1, 1):
            q, s = KIN.ik(p, b, c)
            e = np.linalg.norm(KIN.fk(q) - p) * 1000
            st = ["dentro de límites", "FUERA de límites", "fuera del alcance", "en el cilindro de la coxa"][s]
            lines.append(f"coxa {c:+d}, codo {b:+d}: q_cad = {np.degrees(q).round(2)}°  ({st})\n"
                         f"   q_enc = {KIN.cad_to_enc(q).round(4)} rad   |FK(IK)-p| = {e:.2e} mm")
    return "\n".join(lines)


@app.callback(Output("traj3d", "figure"), Output("traj_err", "figure"), Output("traj_loops", "figure"),
              Output("traj_metrics", "children"), Input("traj", "value"))
def cb_traj(name):
    out, m = run_traj(name)
    pd_, ps, t = mm(out["p_des"]), mm(out["p_sim"]), out["t"]
    k = np.arange(0, len(t), 100)                 # cuadros de la animación cada 0.1 s
    base = [go.Scatter3d(x=pd_[:, 0], y=pd_[:, 1], z=pd_[:, 2], mode="lines", name="deseada",
                         line=dict(color=REF, dash="dash", width=4)),
            go.Scatter3d(x=ps[:, 0], y=ps[:, 1], z=ps[:, 2], mode="lines", name="alcanzada",
                         line=dict(color=JC[0], width=5))]
    legs = lambda i: leg_traces(out["q_sim_cad"][i], show_mesh=False, name="pata (simulada)")
    fig = go.Figure(base + legs(0))
    fig.frames = [go.Frame(data=legs(i), traces=[2], name=f"{t[i]:.1f}") for i in k]
    fig.update_layout(**LAYOUT, scene=scene(), title=f"{tr.NAMES[name]}: trayectoria del pie",
                      updatemenus=[dict(type="buttons", x=0, y=0, buttons=[
                          dict(label="▶ reproducir", method="animate",
                               args=[None, dict(frame=dict(duration=100, redraw=True), fromcurrent=True)])])],
                      sliders=[dict(steps=[dict(method="animate", label=f.name,
                                                args=[[f.name], dict(mode="immediate", frame=dict(duration=0, redraw=True))])
                                           for f in fig.frames], currentvalue=dict(prefix="t = ", suffix=" s"))])
    e = out["err"] * 1000
    fe = make_subplots(rows=2, cols=1, shared_xaxes=True, subplot_titles=("Error cartesiano [mm]", "Error angular [°]"))
    for i, (lab, c) in enumerate(zip("xyz", JC)):
        fe.add_trace(go.Scatter(x=t, y=e[:, i], name=f"e{lab}", line=dict(color=c, width=1)), 1, 1)
    fe.add_trace(go.Scatter(x=t, y=np.linalg.norm(e, axis=1), name="‖e‖", line=dict(color="#0b0b0b")), 1, 1)
    ea = np.degrees(out["q_des_cad"] - out["q_sim_cad"])
    for j in range(3):
        fe.add_trace(go.Scatter(x=t, y=ea[:, j], name=JN[j], line=dict(color=JC[j], width=1.2)), 2, 1)
    fe.update_layout(**LAYOUT, xaxis2_title="tiempo [s]")
    r = out["res"]
    fl = make_subplots(rows=3, cols=3, shared_xaxes=True, subplot_titles=[f"{n} · {l}" for l in
                       ("posición [rad]", "velocidad [rad/s]", "corriente [A eq.]") for n in JN])
    for j in range(3):
        for row, (a, b) in enumerate([("q_ref", "q"), ("w_ref", "w"), ("i_ref", "i_meas")]):
            fl.add_trace(go.Scatter(x=t, y=r[a][:, j], line=dict(color=REF, dash="dash", width=1), showlegend=False),
                         row + 1, j + 1)
            fl.add_trace(go.Scatter(x=t, y=r[b][:, j], line=dict(color=JC[row], width=1), showlegend=False),
                         row + 1, j + 1)
    fl.update_layout(**LAYOUT, title="Referencia (discontinua) vs respuesta de cada lazo")
    txt = html.P(f"RMSE {m['RMSE_mm']:.2f} mm · error máximo {m['max_mm']:.2f} mm · "
                 f"error permanente {m['ss_mm']:.2f} mm ({m['nota']})")
    return fig, fe, fl, txt


@app.callback(Output("steps", "figure"), Output("bode", "figure"), Output("loop_table", "children"),
              Input("joint", "value"))
def cb_cascade(j):
    s = steps()[j]
    fs = make_subplots(rows=1, cols=3, subplot_titles=("Corriente [A eq.]", "Velocidad [rad/s]", "Posición [rad]"))
    for c, (loop, ref, y) in enumerate([("corriente", "i_ref", "i_meas"), ("velocidad", "w_ref", "w"),
                                        ("posicion", "q_ref", "q")]):
        r, m = s[loop]
        fs.add_trace(go.Scatter(x=r.t, y=r[ref][:, j], line=dict(color=REF, dash="dash"), showlegend=False), 1, c + 1)
        fs.add_trace(go.Scatter(x=r.t, y=r[y][:, j], line=dict(color=JC[c]), showlegend=False), 1, c + 1)
    fs.update_xaxes(range=[0.08, 0.17], row=1, col=1)
    fs.update_layout(**LAYOUT, title=f"{JN[j]}: respuesta al escalón")
    w, L, nyq = la.loops(CFG, j)
    f = w / (2 * np.pi)
    fb = go.Figure()
    summ = la.summary(CFG, j)
    for c, (loop, (Lk, Tk)) in enumerate(L.items()):
        mk = f <= nyq[loop]
        fb.add_trace(go.Scatter(x=f[mk], y=20 * np.log10(np.abs(Tk[mk])), name=f"{loop}: {summ[loop]['bw_hz']:.2f} Hz",
                                line=dict(color=JC[c])))
    fb.add_hline(-3, line_dash="dot", line_color="#898781")
    fb.update_layout(**LAYOUT, title="Bode de lazo cerrado", xaxis_type="log", xaxis_title="frecuencia [Hz]",
                     yaxis_title="|T| [dB]", yaxis_range=[-40, 10])
    rows = [html.Tr([html.Th(h) for h in ["Lazo", "tr [s]", "Mp [%]", "ts [s]", "ess", "BW [Hz]", "cruce [Hz]", "MF [°]"]])]
    for loop in ("corriente", "velocidad", "posicion"):
        m, a = s[loop][1], summ[loop]
        rows.append(html.Tr([html.Td(loop)] + [html.Td(f"{v:.4g}") for v in
                    (m["tr"], m["Mp"], m["ts"], m["ess"], a["bw_hz"], a["fc_hz"], a["pm_deg"])]))
    rows.append(html.Tr([html.Td(f"separación corriente/velocidad {summ['sep_corr_vel']:.1f}× · "
                                 f"velocidad/posición {summ['sep_vel_pos']:.1f}×", colSpan=8)]))
    return fs, fb, html.Table(rows, style={"borderSpacing": "12px 4px"})


@app.callback(Output("real_fig", "figure"), Output("real_msg", "children"),
              Input("upload", "contents"), Input("real_joint", "value"), State("upload", "filename"))
def cb_real(contents, j, fname):
    if contents is None:
        return go.Figure(layout=LAYOUT), ""
    try:
        text = base64.b64decode(contents.split(",", 1)[1]).decode("utf-8-sig")
        data = rd.active_window(rd.load_csv(text), j)
        sim, m = rd.compare(CFG, data, j)
    except Exception as exc:  # noqa: BLE001
        return go.Figure(layout=LAYOUT), html.P(f"No se pudo leer {fname}: {exc}", style={"color": "#d03b3b"})
    f = make_subplots(rows=3, cols=1, shared_xaxes=True,
                      subplot_titles=("Posición [rad]", "Velocidad [rad/s]", "Corriente [A eq.]"))
    t = data["t"]
    f.add_trace(go.Scatter(x=t, y=data["position_ref_rad"][:, j], name="referencia", line=dict(color=REF, dash="dash")), 1, 1)
    for row, (real, sim_y) in enumerate([(data["position_rad"][:, j], sim["q"][:, j]),
                                         (data["speed_rad_s"][:, j], sim["w_meas"][:, j]),
                                         (np.sign(data["duty_percent"][:, j]) * data["current_a"][:, j], sim["i_meas"][:, j])]):
        f.add_trace(go.Scatter(x=t, y=real, name="pata real", line=dict(color=JC[0], width=1.2), showlegend=row == 0), row + 1, 1)
        f.add_trace(go.Scatter(x=sim.t, y=sim_y, name="simulación", line=dict(color=JC[1], width=1.2), showlegend=row == 0),
                    row + 1, 1)
    f.update_layout(**LAYOUT, title=f"{JN[j]}: real vs simulación ({fname})")
    return f, html.P(f"RMSE de posición sim–real {m['rmse_sim_real_rad']:.4f} rad · máximo {m['max_sim_real_rad']:.4f} rad")


if __name__ == "__main__":
    app.run(debug=False, port=8050)

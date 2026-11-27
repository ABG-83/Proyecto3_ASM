"""
disenar_controlador.py - Diseno del PID digital del banco de frenado

Pasos:
  1. Modelo de la planta (de Integrante B; si no existe, el de la Tarea 2).
  2. Sintonia inicial por SIMC (Skogestad, 2003): PI y PID.
  3. Discretizacion: planta con ZOH (retardo fraccionario exacto) y
     controlador con Euler hacia atras (igual que Control.h).
  4. Polos y ceros del controlador y del lazo cerrado; margenes.
  5. Simulacion no lineal de la rampa con saturacion y anti-windup.

Instalar:  pip install control numpy scipy matplotlib
Uso:       python disenar_controlador.py
Salidas:   figuras PNG en ../../documentacion/paper/figuras/
"""
import json
import warnings
from pathlib import Path

import numpy as np
import control as ct
import matplotlib.pyplot as plt
from scipy.linalg import expm

warnings.filterwarnings("ignore")   # avisos numericos de scipy/control sin efecto en el resultado

AQUI = Path(__file__).resolve().parent
FIGURAS = AQUI.parent.parent / "documentacion" / "paper" / "figuras"
TS = 0.020          # periodo de muestreo [s] (Tarea 2, Pregunta 2)
TF = 0.10           # constante de tiempo del filtro derivativo [s] (Control.h)


# ---------------------------------------------------------------- 1. Modelo
def cargar_modelo():
    """G(s) = -K e^(-L s) / ((tau s + 1)(tau_a s + 1))."""
    mat = AQUI.parent / "matlab" / "modelo_planta.mat"
    js = AQUI / "modelo_planta.json"
    if mat.exists():
        from scipy.io import loadmat
        d = loadmat(mat)
        m = {k: float(np.squeeze(d[k])) for k in ("K", "tau", "tau_a", "L")}
        return m, "modelo identificado (modelo_planta.mat)"
    if js.exists():
        m = json.loads(js.read_text())
        return {k: float(m[k]) for k in ("K", "tau", "tau_a", "L")}, \
            "modelo identificado (modelo_planta.json)"
    return dict(K=29.17, tau=5.0, tau_a=0.12, L=0.15), "modelo supuesto de la Tarea 2"


# ------------------------------------------------------------- 2. Sintonia
def simc(m):
    K, tau, ta, L = m["K"], m["tau"], m["tau_a"], m["L"]
    # PI: regla de la mitad -> primer orden con retardo, tau_c = theta
    tau1, theta = tau + ta / 2, L + ta / 2
    kp = tau1 / (K * 2 * theta)
    ti = min(tau1, 8 * theta)
    pi = dict(kp=kp, ki=kp / ti, kd=0.0)
    # PID: segundo orden con retardo (forma serie) -> forma paralela
    kc = tau / (K * 2 * L)
    t_i = min(tau, 8 * L)
    t_d = ta
    kp2 = kc * (1 + t_d / t_i)
    ti2, td2 = t_i + t_d, t_i * t_d / (t_i + t_d)
    pid = dict(kp=kp2, ki=kp2 / ti2, kd=kp2 * td2)
    return pi, pid


# --------------------------------------------------------- 3. Discretizacion
def c2d_zoh_retardo(G, ts, L):
    """ZOH exacto de G(s) con retardo de entrada L (puede ser fraccionario).

    Se escribe L = l*Ts + tau', 0 <= tau' < Ts. Entonces
      x[k+1] = Phi x[k] + G0 u[k-l] + G1 u[k-l-1]
    con G0 = int_0^{Ts-tau'} e^{As} ds B  y  G1 = e^{A(Ts-tau')} int_0^{tau'} e^{As} ds B
    (Astrom y Wittenmark, Computer-Controlled Systems, cap. 2).
    """
    s = ct.ss(G)
    A, B, C, D = (np.atleast_2d(x) for x in (s.A, s.B, s.C, s.D))
    n = A.shape[0]
    l = int(np.floor(L / ts + 1e-9))
    tp = L - l * ts

    def integral(t):                      # (e^{At}, int_0^t e^{As} ds B)
        M = np.zeros((n + 1, n + 1))
        M[:n, :n], M[:n, n:] = A * t, B * t
        E = expm(M)
        return E[:n, :n], E[:n, n:]

    phi, _ = integral(ts)
    e_a, g0 = integral(ts - tp)
    _, i_tp = integral(tp)
    g1 = e_a @ i_tp

    # Estado aumentado: [x, u[k-1], ..., u[k-l-1]]  (l+1 entradas guardadas)
    q = l + 1
    Az = np.zeros((n + q, n + q))
    Bz = np.zeros((n + q, 1))
    Az[:n, :n] = phi
    if l == 0:
        Bz[:n] = g0                       # u[k] entra directo
        Az[:n, n] = g1[:, 0]              # u[k-1]
    else:
        Az[:n, n + l - 1] = g0[:, 0]      # u[k-l]
        Az[:n, n + l] = g1[:, 0]          # u[k-l-1]
    Bz[n] = 1.0                           # guarda u[k] como u[k-1] del siguiente paso
    for i in range(1, q):
        Az[n + i, n + i - 1] = 1.0        # desplaza la cola de entradas
    Cz = np.hstack([C, np.zeros((1, q))])
    return ct.ss(Az, Bz, Cz, D, ts)


def controlador_z(g, ts=TS, tf=TF):
    """Misma ley que Control.h, linealizada (derivada sobre el error):
    C(z) = Kp + Ki Ts z/(z-1) + Kd (1-alfa)(z-1) / (Ts (z-alfa))."""
    alfa = tf / (tf + ts)
    z = ct.tf([1, 0], [1], ts)
    return g["kp"] + g["ki"] * ts * z / (z - 1) + g["kd"] * (1 - alfa) * (z - 1) / (ts * (z - alfa))


# ------------------------------------------------- 5. Simulacion no lineal
def simular(g, w0, a, ts=TS, tf=TF, tfin=None):
    """Planta fisica de la Tarea 2 (Pregunta 5) + PID identico a Control.h."""
    J, b, tc, kb, ta, L = 0.006, 0.0012, 0.002, 0.035, 0.12, 0.15
    if tfin is None:
        tfin = w0 / a + 1.0               # la rampa llega a cero y un segundo mas
    dt = 1e-3
    paso, nd = round(ts / dt), round(L / dt)
    cola = [0.0] * nd
    w, v, integ, df, w_ant, u = w0, 0.0, 0.0, 0.0, w0, 0.0
    alfa = tf / (tf + ts)
    T, R, W, U = [], [], [], []
    for k in range(round(tfin / dt)):
        if k % paso == 0:                               # ---- PID (Control.h)
            t = k * dt
            r = max(w0 - a * t, 0.0)
            e = w - r
            df = alfa * df + (1 - alfa) * (w - w_ant) / ts
            w_ant = w
            u_sin = g["kp"] * e + integ + g["kd"] * df
            u = min(max(u_sin, 0.0), 1.0)
            if not ((u_sin > 1 and e > 0) or (u_sin < 0 and e < 0)):
                integ = min(max(integ + g["ki"] * ts * e, 0.0), 1.0)
            T.append(t); R.append(r); W.append(w); U.append(u)
        cola.append(u)                                  # ---- planta fisica
        ud = cola.pop(0)
        v += dt * (ud - v) / ta
        if w > 0:
            w = max(w + dt * (-b * w - tc - kb * v) / J, 0.0)
    return tuple(np.array(x) for x in (T, R, W, U))


# ------------------------------------------------------------------ main
def main():
    m, fuente = cargar_modelo()
    print(f"Planta: {fuente}")
    print("  K = {K:.2f} rad/s, tau = {tau:.2f} s, tau_a = {tau_a:.2f} s, L = {L:.2f} s".format(**m))

    # Accion inversa: e = w - r, u = C e  ->  equivale a un lazo normal con Gp = -G.
    Gp = ct.tf([m["K"]], [m["tau"] * m["tau_a"], m["tau"] + m["tau_a"], 1])
    Gz = c2d_zoh_retardo(Gp, TS, m["L"])

    pi, pid = simc(m)
    casos = {"PI": pi, "PID": pid}
    print("\nSintonia SIMC")
    for nombre, g in casos.items():
        print(f"  {nombre:3s}: Kp = {g['kp']:.3f}, Ki = {g['ki']:.3f} 1/s, Kd = {g['kd']:.4f} s")

    FIGURAS.mkdir(parents=True, exist_ok=True)
    fig_pz, ejes = plt.subplots(1, 2, figsize=(9, 4.4))
    for eje, (nombre, g) in zip(ejes, casos.items()):
        C = controlador_z(g)
        Lz = ct.series(ct.ss(C), Gz)       # en espacio de estados: mejor condicionado
        T = ct.feedback(Lz, 1)
        p_c, z_c = ct.poles(C), ct.zeros(C)
        p_t, z_t = ct.poles(T), ct.zeros(T)
        gm, pm, _, wc = ct.margin(Lz)
        estable = np.all(np.abs(p_t) < 1)
        print(f"\n{nombre}")
        print(f"  Polos del controlador: {np.round(p_c, 4)}")
        print(f"  Ceros del controlador: {np.round(z_c, 4)}")
        print(f"  |polo| maximo lazo cerrado: {np.max(np.abs(p_t)):.4f} -> "
              f"{'estable' if estable else 'INESTABLE'}")
        print(f"  Margen de ganancia: {gm:.2f} ({20*np.log10(gm):.1f} dB), "
              f"margen de fase: {pm:.1f} grados, wc = {wc:.2f} rad/s")

        th = np.linspace(0, 2 * np.pi, 400)
        eje.plot(np.cos(th), np.sin(th), "k:", lw=0.8)
        eje.plot(p_t.real, p_t.imag, "x", label="polos lazo cerrado")
        eje.plot(z_t.real, z_t.imag, "o", mfc="none", label="ceros lazo cerrado")
        eje.plot(p_c.real, p_c.imag, "s", mfc="none", label="polos controlador")
        fuera = z_t[np.abs(z_t) > 1.3]   # ceros del retardo fraccionario: no afectan la estabilidad
        if fuera.size:
            print(f"  Ceros fuera del circulo (retardo discretizado): {np.round(fuera, 2)}")
            eje.annotate("cero en z = " + ", ".join(f"{c.real:.1f}" for c in fuera),
                         xy=(-1.28, -1.2), fontsize=8)
        eje.set_xlim(-1.3, 1.3); eje.set_ylim(-1.3, 1.3)
        eje.set_title(f"Plano z ({nombre})")
        eje.set_aspect("equal"); eje.grid(True); eje.set_xlabel("Re"); eje.set_ylabel("Im")
    fig_pz.legend(*ejes[0].get_legend_handles_labels(), loc="lower center", ncol=3, fontsize=8)
    fig_pz.tight_layout(rect=(0, 0.08, 1, 1))
    fig_pz.savefig(FIGURAS / "polos_ceros_z.png", dpi=200)

    # Simulacion de la rampa
    J, b, tc, kb = 0.006, 0.0012, 0.002, 0.035
    w0, a = 120 * 2 * np.pi / 60, 4.0
    print(f"\nRampa: w0 = {w0:.2f} rad/s, a = {a:.1f} rad/s^2")
    print(f"  Desaceleracion libre ~ {(b*w0 + tc)/J:.2f} rad/s^2, "
          f"con freno a fondo ~ {(b*w0 + tc + kb)/J:.2f} rad/s^2")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 5.5), sharex=True)
    for nombre, g in casos.items():
        t, r, w, u = simular(g, w0, a)
        act = r > 0
        rmse = np.sqrt(np.mean((w[act] - r[act]) ** 2))
        print(f"  {nombre}: RMSE = {rmse:.3f} rad/s, saturacion = {100*np.mean(u[act] >= 1):.1f} % del tiempo")
        ax1.plot(t, w, label=f"$\\omega$ {nombre}")
        ax2.step(t, u, where="post", label=f"u {nombre}")
    ax1.plot(t, r, "k--", label="referencia")
    ax1.set_ylabel(r"$\omega$ [rad/s]"); ax1.grid(True); ax1.legend()
    ax2.set_ylabel("u"); ax2.set_xlabel("t [s]"); ax2.set_ylim(-0.05, 1.05); ax2.grid(True); ax2.legend()
    fig.tight_layout()
    fig.savefig(FIGURAS / "simulacion_rampa.png", dpi=200)
    print(f"\nFiguras guardadas en {FIGURAS}")


if __name__ == "__main__":
    main()
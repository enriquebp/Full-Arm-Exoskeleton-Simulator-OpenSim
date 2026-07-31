"""
precompute_sweep.py
=====================
Paso "Opcion 1": barre sistematicamente el rango de movimiento completo
(sh_abd x sh_flex x elbow_flex) y calcula, en cada punto:
  - el torque de tu formula simple (seno/coseno)
  - el torque 3D completo (torque_3d.py, sin OpenSim)
  - el torque real de OpenSim (opensim_bridge.py)

Guarda todo en 'torque_comparison_table.npz', que:
  1. Sirve para cuantificar el error de tu modelo simplificado en TODO el
     rango de movimiento (no solo las ~10 posturas puntuales probadas hasta
     ahora), con estadisticas y un mapa de error.
  2. Alimenta las curvas "OpenSim (real)" que el simulador PyQt6 ahora
     dibuja en los graficos de Torque vs Angulo (ver _lookup_osim_curve
     en urdf_arm_simulator_EN_Final.py).

DISEÑO EFICIENTE (importante, evita recalcular de mas):
El bridge de hombro (elv_angle, shoulder_elv) y su Jacobiana NO dependen
de elbow_flex -- solo de (sh_abd, sh_flex). Asi que por cada combinacion
(sh_abd, sh_flex) se resuelve el bridge+Jacobiana UNA SOLA VEZ (la parte
cara), y despues se barre elbow_flex reutilizando ese mismo resultado,
solo recalculando el torque gravitacional (barato, sin optimizador) para
cada valor de elbow_flex.

Uso:
    conda activate exoarm_osim
    cd "...\\Bimanual Upper Arm Model"
    python precompute_sweep.py

ADVERTENCIA DE TIEMPO: con la resolucion por defecto (paso de 15 grados en
hombro, 10 grados en codo), esto puede tardar entre 10 y 30 minutos segun
tu maquina -- es un precalculo de una sola vez, no necesita repetirse salvo
que cambies las masas/longitudes de los sliders de forma importante.
"""

import math
import time
import numpy as np

from opensim_bridge import (
    OpenSimArmModel, solve_osim_angles, gravity_only_generalized_forces,
    project_shoulder_torque_to_urdf,
)

OSIM_PATH = "MoBL_ARMS_bimanual_6_2_21.osim"
OUTPUT_PATH = "torque_comparison_table.npz"

# Defaults de tu simulador (ajusta si usas otros valores en los sliders)
U_MASS, U_LEN = 2.0, 0.30
F_MASS, F_LEN = 1.5, 0.27
P_MASS = 0.0
G = 9.81

# Resolucion de la grilla (grados). Reducir el paso = mas preciso pero
# mucho mas lento (el costo escala con n_abd * n_flex).
SH_ABD_RANGE = np.arange(0, 181, 15)     # 0 a 180, paso 15 -> 13 puntos
SH_FLEX_RANGE = np.arange(-60, 181, 15)  # -60 a 180, paso 15 -> 17 puntos
ELBOW_RANGE = np.arange(0, 151, 10)      # 0 a 150, paso 10 -> 16 puntos


def urdf_torques_simple(sh_abd, sh_flex, elbow_flex):
    torque_elbow = (F_MASS * G * (F_LEN / 2.0) + P_MASS * G * F_LEN) * math.sin(elbow_flex)
    torque_sh_flex = (U_MASS * G * (U_LEN / 2.0) + (F_MASS + P_MASS) * G * U_LEN) * math.sin(sh_flex) + torque_elbow
    torque_sh_abd = (U_MASS * G * (U_LEN / 2.0) + (F_MASS + P_MASS) * G * U_LEN) * math.cos(sh_flex) * math.sin(sh_abd)
    return torque_sh_flex, torque_sh_abd, torque_elbow


def main():
    print(f"Cargando modelo: {OSIM_PATH} ...")
    arm = OpenSimArmModel(OSIM_PATH, side="r")

    n_abd, n_flex, n_elbow = len(SH_ABD_RANGE), len(SH_FLEX_RANGE), len(ELBOW_RANGE)
    total_combos = n_abd * n_flex
    print(f"Grilla: {n_abd} x {n_flex} x {n_elbow} = {n_abd*n_flex*n_elbow} puntos totales")
    print(f"({total_combos} resoluciones de bridge+Jacobiana, {n_abd*n_flex*n_elbow} evaluaciones de torque)\n")

    tau_sh_flex_osim = np.zeros((n_abd, n_flex, n_elbow))
    tau_sh_abd_osim = np.zeros((n_abd, n_flex, n_elbow))
    tau_elbow_osim = np.zeros((n_abd, n_flex, n_elbow))
    tau_sh_flex_simple = np.zeros((n_abd, n_flex, n_elbow))
    tau_sh_abd_simple = np.zeros((n_abd, n_flex, n_elbow))
    tau_elbow_simple = np.zeros((n_abd, n_flex, n_elbow))

    t0 = time.time()
    combo_count = 0
    for i, abd_deg in enumerate(SH_ABD_RANGE):
        for j, flex_deg in enumerate(SH_FLEX_RANGE):
            sh_abd = math.radians(abd_deg)
            sh_flex = math.radians(flex_deg)

            # Parte cara: UNA sola vez por (sh_abd, sh_flex)
            elv_angle, shoulder_elv, err_deg = solve_osim_angles(arm, sh_abd, sh_flex)

            combo_count += 1
            elapsed = time.time() - t0
            avg = elapsed / combo_count
            remaining = avg * (total_combos - combo_count)
            print(f"[{combo_count}/{total_combos}] sh_abd={abd_deg:.0f} sh_flex={flex_deg:.0f} "
                  f"(ajuste={err_deg:.2f}°) -- ETA restante: {remaining/60:.1f} min", flush=True)

            for k, elbow_deg in enumerate(ELBOW_RANGE):
                elbow_flex = math.radians(elbow_deg)
                arm.set_pose(elv_angle, shoulder_elv, shoulder_rot=0.0, elbow_flex=elbow_flex)

                # Parte barata: se repite por cada elbow_flex (sin optimizador)
                forces = gravity_only_generalized_forces(
                    arm, coord_names=["elv_angle_r", "shoulder_elv_r", "elbow_flexion_r"],
                    payload_mass=P_MASS, payload_f_len=F_LEN,
                )
                tsa, tsf = project_shoulder_torque_to_urdf(
                    arm, sh_abd, sh_flex, elv_angle, shoulder_elv,
                    forces["elv_angle_r"], forces["shoulder_elv_r"],
                )
                tau_sh_flex_osim[i, j, k] = tsf
                tau_sh_abd_osim[i, j, k] = tsa
                tau_elbow_osim[i, j, k] = forces["elbow_flexion_r"]

                sf_s, sa_s, el_s = urdf_torques_simple(sh_abd, sh_flex, elbow_flex)
                tau_sh_flex_simple[i, j, k] = sf_s
                tau_sh_abd_simple[i, j, k] = sa_s
                tau_elbow_simple[i, j, k] = el_s

    np.savez(
        OUTPUT_PATH,
        sh_abd_grid=SH_ABD_RANGE.astype(float),
        sh_flex_grid=SH_FLEX_RANGE.astype(float),
        elbow_grid=ELBOW_RANGE.astype(float),
        tau_sh_flex=tau_sh_flex_osim,
        tau_sh_abd=tau_sh_abd_osim,
        tau_elbow=tau_elbow_osim,
        tau_sh_flex_simple=tau_sh_flex_simple,
        tau_sh_abd_simple=tau_sh_abd_simple,
        tau_elbow_simple=tau_elbow_simple,
    )
    print(f"\nGuardado: {OUTPUT_PATH}")

    # --- Estadisticas de error (Opcion 1: cuantificar en todo el rango) ---
    def pct_err(osim, simple):
        mask = np.abs(osim) > 0.5  # evitar dividir cerca de cero
        return 100.0 * np.abs(osim[mask] - simple[mask]) / np.abs(osim[mask])

    print("\n=== Estadisticas de error (formula simple vs OpenSim real) ===")
    for name, osim_arr, simple_arr in [
        ("Sh. Flexion", tau_sh_flex_osim, tau_sh_flex_simple),
        ("Sh. Abduction", tau_sh_abd_osim, tau_sh_abd_simple),
        ("Elbow", tau_elbow_osim, tau_elbow_simple),
    ]:
        err = pct_err(osim_arr, simple_arr)
        print(f"{name:15s}  media={err.mean():6.1f}%  mediana={np.median(err):6.1f}%  "
              f"max={err.max():6.1f}%  p90={np.percentile(err,90):6.1f}%")

    print(f"\nTiempo total: {(time.time()-t0)/60:.1f} minutos")


if __name__ == "__main__":
    main()

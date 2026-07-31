"""
plot_error_map.py
===================
Genera mapas de calor del error (%) entre tu formula simple y OpenSim,
a partir de la tabla precalculada por precompute_sweep.py.

Uso (despues de correr precompute_sweep.py):
    python plot_error_map.py

Genera 'error_map.png' con 3 mapas de calor (uno por torque), cada uno
mostrando el error % en el plano sh_abd x sh_flex, para un valor de
elbow_flex representativo (90 grados, configurable abajo).
"""

import numpy as np
import matplotlib.pyplot as plt

TABLE_PATH = "torque_comparison_table.npz"
ELBOW_SLICE_DEG = 90.0  # que corte de elbow_flex mostrar en el mapa


def main():
    data = np.load(TABLE_PATH)
    abd_grid = data["sh_abd_grid"]
    flex_grid = data["sh_flex_grid"]
    elbow_grid = data["elbow_grid"]
    k = int(np.argmin(np.abs(elbow_grid - ELBOW_SLICE_DEG)))
    actual_elbow = elbow_grid[k]

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    specs = [
        ("Sh. Flexion", data["tau_sh_flex"], data["tau_sh_flex_simple"]),
        ("Sh. Abduction", data["tau_sh_abd"], data["tau_sh_abd_simple"]),
        ("Elbow", data["tau_elbow"], data["tau_elbow_simple"]),
    ]

    for ax, (name, osim_full, simple_full) in zip(axes, specs):
        osim = osim_full[:, :, k]
        simple = simple_full[:, :, k]
        with np.errstate(divide='ignore', invalid='ignore'):
            err_pct = 100.0 * np.abs(osim - simple) / np.abs(osim)
        err_pct = np.where(np.abs(osim) > 0.5, err_pct, np.nan)

        im = ax.imshow(err_pct.T, origin='lower', aspect='auto', cmap='inferno',
                        extent=[abd_grid.min(), abd_grid.max(), flex_grid.min(), flex_grid.max()],
                        vmin=0, vmax=100)
        ax.set_title(f"{name}\nerror %  (elbow_flex={actual_elbow:.0f}°)")
        ax.set_xlabel("shoulder_abd (°)")
        ax.set_ylabel("shoulder_flex (°)")
        fig.colorbar(im, ax=ax, label="error %")

    fig.suptitle("Error del modelo simplificado vs. OpenSim (masa/CoM real)", fontsize=13)
    fig.tight_layout()
    fig.savefig("error_map.png", dpi=150, facecolor="white")
    print("Guardado: error_map.png")


if __name__ == "__main__":
    main()

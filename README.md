# Full-Arm-Exoskeleton-Simulator-OpenSim

*Comparing a simplified anthropometric torque model against a real cadaveric musculoskeletal model (MoBL-ARMS), live, in an interactive PyQt6 simulator.*

*Comparando un modelo de torque antropométrico simplificado contra un modelo musculoesquelético cadavérico real (MoBL-ARMS), en vivo, en un simulador interactivo PyQt6.*

---

## What is this? / ¿Qué es esto?

This project extends an IMU-driven, quaternion-based arm/exoskeleton kinematics simulator (originally using a simplified sine/cosine anthropometric torque model) with a live bridge to **OpenSim** and the **MoBL-ARMS** musculoskeletal model — allowing direct, real-time comparison of three ways of estimating joint torque:

1. **Simple** — the original sine/cosine anthropometric formula (fast, approximate).
2. **3D** — full 3D cross-product torque (`τ = r × F`), using the same simplified anthropometry but exact geometry.
3. **OpenSim (real)** — real cadaveric mass distribution, muscle-level force analysis (Static Optimization), and comparison with/without exoskeleton assist.

*Este proyecto extiende un simulador de cinemática de brazo/exoesqueleto basado en cuaterniones de IMU (originalmente con un modelo de torque antropométrico simplificado seno/coseno) con un puente en vivo hacia **OpenSim** y el modelo musculoesquelético **MoBL-ARMS** — permitiendo comparar en tiempo real tres formas de estimar el torque articular:*

*1. **Simple** — la fórmula antropométrica original seno/coseno (rápida, aproximada).*
*2. **3D** — torque de producto cruzado 3D completo (`τ = r × F`), con la misma antropometría simplificada pero geometría exacta.*
*3. **OpenSim (real)** — distribución de masa cadavérica real, análisis de fuerza a nivel muscular (Static Optimization), y comparación con/sin asistencia del exoesqueleto.*

📖 **Full technical documentation (bilingual, 25 sections, the complete debugging journey):** [`Tutorial_Integracion_OpenSim_EN_ES.md`](./Tutorial_Integracion_OpenSim_EN_ES.md)

---

## Relationship to the original simulator / Relación con el simulador original

This project is an **upgrade/extension** of an earlier, simpler simulator: [**Full-Arm Kinematics & Dynamics Simulator**](https://github.com/enriquebp/Full-ARm-Kinematics-Dynamics-Simulator) — which used IMU quaternions and a simplified sine/cosine anthropometric torque model. Everything in that original project still works here (same kinematics, same UI foundations); this repository adds the full OpenSim/MoBL-ARMS comparison layer, real 3D torque, muscle-level force analysis, and the real-bone 3D viewer on top of it.

*Este proyecto es una **actualización/extensión** de un simulador anterior más simple: [**Full-Arm Kinematics & Dynamics Simulator**](https://github.com/enriquebp/Full-ARm-Kinematics-Dynamics-Simulator) — que usaba cuaterniones de IMU y un modelo de torque antropométrico simplificado seno/coseno. Todo lo de ese proyecto original sigue funcionando aquí (misma cinemática, misma base de interfaz); este repositorio agrega encima la capa completa de comparación con OpenSim/MoBL-ARMS, el torque 3D real, el análisis de fuerza a nivel muscular, y el visor 3D de huesos reales.*


https://github.com/user-attachments/assets/4451c70d-f620-41a3-864e-0dd1809fb069



https://github.com/user-attachments/assets/57fe7b88-be6c-47aa-9cbb-7f283aa2c049




---

## Quick start / Inicio rápido

```bash
# 1. Create a dedicated conda environment
conda create -n exoarm_osim python=3.11
conda activate exoarm_osim

# 2. Install OpenSim (see Tutorial Section 2 for troubleshooting)
conda install -c opensim-org opensim

# 3. Install the rest of the dependencies
pip install -r requirements.txt

# 4. Get the MoBL-ARMS model (see "Model file" section below)
#    Place MoBL_ARMS_bimanual_6_2_21.osim + Geometry/ folder in this directory

# 5. Run the simulator
python urdf_arm_simulator_EN_Final.py
```

---

## ⚠️ Model file / Archivo del modelo

**Not included in this repository.** `MoBL_ARMS_bimanual_6_2_21.osim` and its `Geometry/` folder (bone mesh files) are a third-party research model (Saul et al., 2015 — "Benchmarking of dynamic simulation predictions in two software platforms using an upper limb musculoskeletal model", *Computer Methods in Biomechanics and Biomedical Engineering*). Obtain it from the [OpenSim model repository / SimTK](https://simtk.org/projects/upexdyn) and place both the `.osim` file and the `Geometry/` folder directly in this project's root directory before running the simulator.

*No incluido en este repositorio.* `MoBL_ARMS_bimanual_6_2_21.osim` y su carpeta `Geometry/` (mallas óseas) son un modelo de investigación de terceros (Saul et al., 2015). Consíguelo desde el repositorio de modelos de OpenSim / SimTK y coloca tanto el archivo `.osim` como la carpeta `Geometry/` directamente en la raíz de este proyecto antes de correr el simulador.

If you cite this project's OpenSim comparison results, please also cite the MoBL-ARMS model itself.

---

## Project structure / Guía de archivos

Everything runs from a single flat folder (all scripts assume the others are alongside them). Grouped here by purpose:

*Todo corre desde una sola carpeta plana (todos los scripts asumen que los demás están al lado). Agrupados aquí por propósito:*

### Core application / Aplicación principal
| File | Purpose |
|---|---|
| `urdf_arm_simulator_EN_Final.py` | Main PyQt6 simulator — sliders, live 3D view, torque graphs, muscle analysis, real-bone viewer. |
| `opensim_bridge.py` | The core library: URDF↔OpenSim angle conversion, gravitational torque extraction, Static Optimization (muscle forces), Jacobian-based basis projection. |
| `torque_3d.py` | Full 3D cross-product torque calculation (no OpenSim needed). |
| `bone_viewer.py` | Parses `.osim` mesh↔body associations and computes world transforms for the PyVista "Realistic View" tab. |
| `arm_shoulder_elbow.urdf` | The corrected URDF (elbow axis fixed for anatomically correct sagittal flexion). |

### Offline analysis / Análisis fuera de línea
| File | Purpose |
|---|---|
| `precompute_sweep.py` | Sweeps the full range of motion offline, generates `torque_comparison_table.npz` (feeds the live "OpenSim (real)" curves and quantifies error). |
| `plot_error_map.py` | Generates a heatmap image of simplified-vs-real error across the range of motion, from the precomputed table. |

### Validation test scripts / Scripts de validación
*(Each corresponds to a specific tutorial section — see the docstring at the top of each file.)*
| File | Validates |
|---|---|
| `test_opensim_bridge.py` | Shoulder kinematic bridge (Tutorial §6-8). |
| `test_shoulder_rot.py` | `shoulder_rot` calibration (§10). |
| `test_gravity_torque.py` | Gravitational torque extraction, elbow (§11). |
| `test_payload.py` | Virtual payload simulation (§14). |
| `test_shoulder_torque.py` | Jacobian-based shoulder torque projection (§13, §20). |
| `test_torque_3d.py` | Full 3D torque vs. simplified formula (§15). |
| `test_muscle_forces.py` | Static Optimization, elbow only (§20.3). |
| `test_muscle_forces_full_arm.py` | Static Optimization, full arm (§20.3). |
| `test_bone_viewer.py` | Bone mesh world transforms (§24.2). |
| `test_all_bones_diagnostic.py` | Full kinematic chain sanity check (§24.4). |

### Debugging history / Historial de depuración
*(In `debug_history/` — kept for reference; document real bugs found and how they were diagnosed. Not needed for normal use. See Tutorial for the full narrative.)*

**Run these from the project root** (not from inside `debug_history/`), so the relative path to the `.osim` model still resolves correctly:
```bash
python debug_history/test_flex_vs_abd.py
```
*Corre estos desde la raíz del proyecto (no desde adentro de `debug_history/`), para que la ruta relativa al modelo `.osim` siga resolviendo correctamente.*

| File | What it found |
|---|---|
| `debug_flexion_grid.py` | Brute-force grid search that proved a solver local-minimum, not a code bug (§6.3). |
| `inspect_direction_map.py` | Direct pose inspection that revealed the axis-mapping error (§6.4). |
| `visualize_calibration_pose.py` | Visual confirmation via Simbody Visualizer that settled the axis convention (§6.5). |
| `test_flex_vs_abd.py` | Proved the bone-viewer data was correct when the rendering still looked wrong (§24). |
| `test_pyvista_view.py` | Standalone PyVista window (no PyQt6 app needed) — useful for iterating on camera calibration (§24.5, still pending). |

### Documentation / Documentación
| File | Purpose |
|---|---|
| `Tutorial_Integracion_OpenSim_EN_ES.md` | The full bilingual technical log — every step, every bug, every fix, with plain-language explanations. **Start here for the full story.** |

---

## Key findings / Hallazgos clave

- The simplified anthropometric model **underestimates** gravitational holding torque by ~65-75% vs. real cadaveric mass distribution (Tutorial §11.5).
- OpenSim's own coordinate basis (`elv_angle_r`/`shoulder_elv_r`) is **not** equivalent to independent-axis `shoulder_flex`/`shoulder_abd` sliders — converting between them requires solving a real geometric matching problem, not a simple relabeling (§1, §5-7).
- A performance optimization (`enforceContraints=False`) silently broke the scapulohumeral rhythm coupling for months of development before a visual debugging session surfaced it (§24.3) — a reminder that "cosmetic" bugs can hide real correctness issues.

*El modelo antropométrico simplificado **subestima** el torque de sostenimiento gravitacional en ~65-75% vs. la distribución de masa cadavérica real. La base propia de coordenadas de OpenSim **no** es equivalente a los sliders de ejes independientes — convertir entre ellas requiere resolver un problema geométrico real. Una optimización de rendimiento rompió en silencio el acoplamiento escapulohumeral hasta que una sesión de depuración visual lo sacó a la luz — un recordatorio de que los bugs "cosméticos" pueden esconder problemas reales de corrección.*

---

## Known limitations / Limitaciones conocidas

- **"Realistic View" (PyVista) camera presets are unreliable.** The underlying bone position data is proven correct (Tutorial §24.4), but the default camera framing needs manual tuning — use free mouse rotation in that tab instead of the Top/Side/Front buttons for now.
- `precompute_sweep.py` has not yet been run in this repo snapshot — the "OpenSim (real)" reference curves in the live graphs will be empty until you run it once (~10-30 min).

---

## Requirements

See [`requirements.txt`](./requirements.txt). OpenSim itself must be installed via conda (`conda install -c opensim-org opensim`), not pip — see Tutorial §2.

---

## License / Citation

This code is provided as teaching material. If you use the OpenSim/MoBL-ARMS comparison results, please cite:

> Saul, K.R., Hu, X., Goehler, C.M., Vidt, M.E., Daly, M., Velisar, A., Murray, W.M. (2015). Benchmarking of dynamic simulation predictions in two software platforms using an upper limb musculoskeletal model. *Computer Methods in Biomechanics and Biomedical Engineering*, 18(13), 1445-1458.

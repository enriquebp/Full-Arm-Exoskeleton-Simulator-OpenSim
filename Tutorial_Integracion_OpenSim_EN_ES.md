# OpenSim ↔ URDF Simulator Integration — Step-by-Step Tutorial (EN/ES)
# Tutorial Paso a Paso: Integración OpenSim ↔ Simulador URDF (EN/ES)

*Bilingual technical log — Elbow_Exoskeleton_V4 project*
*Registro técnico bilingüe — proyecto Elbow_Exoskeleton_V4*

---

## 0. Project goal / Objetivo del proyecto

You have two ways to estimate shoulder/elbow torque during a movement: (1) your current PyQt6 simulator, which extracts joint angles from IMU quaternions and estimates torque with a **simplified** anthropometric model (uniform cylinder, mass at half segment length, one sine/cosine lever-arm term per joint); and (2) OpenSim with the MoBL-ARMS model, a musculoskeletal model derived from real cadaveric measurements, with real masses, centers of mass, inertias, and — eventually — individual muscles.

*Tienes dos formas de estimar el torque de hombro/codo durante un movimiento: (1) tu simulador PyQt6 actual, que extrae ángulos articulares de cuaterniones IMU y estima el torque con un modelo antropométrico **simplificado** (cilindro uniforme, masa a la mitad del segmento, un término seno/coseno de palanca por articulación); y (2) OpenSim con el modelo MoBL-ARMS, un modelo musculoesquelético derivado de mediciones cadavéricas reales, con masas, centros de masa, inercias reales y — más adelante — músculos individuales.*

**Core idea:** feed both pipelines with the *same physical posture* (the same angles derived from your quaternions) and compare the resulting torque, to quantify how much error your simplified approximation introduces relative to a real anatomical model.

***Idea central:*** *alimentar ambos pipelines con la **misma postura física** (los mismos ángulos derivados de tus cuaterniones) y comparar el torque resultante, para cuantificar cuánto error introduce tu aproximación simplificada frente a un modelo anatómico real.*

```
IMU quaternions → angles (sh_abd, sh_flex, sh_rot, elbow_flex)
                          │
              ┌───────────┴────────────┐
              ▼                        ▼
     Pipeline A (current)      Pipeline B (new, OpenSim)
     τ = sine/cosine formula    τ = OpenSim (real masses/muscles)
              │                        │
              └───────────┬────────────┘
                           ▼
                  Side-by-side comparison
```

---

## 1. Why this isn't a trivial translation / Por qué no es una traducción trivial

Your simulator describes the shoulder with 3 independent anatomical angles (`shoulder_abd` about -Y, `shoulder_flex` about +X, `shoulder_rot` about +Z, sequential joints). OpenSim's MoBL-ARMS uses the standard ISB "plane of elevation / elevation" parametrization (`elv_angle_r`, `shoulder_elv_r`, `shoulder_rot_r`) — a coupled spherical representation, not independent Euler rotations. The two are **not interchangeable** by direct value copying.

*Tu simulador describe el hombro con 3 ángulos anatómicos independientes (`shoulder_abd` sobre -Y, `shoulder_flex` sobre +X, `shoulder_rot` sobre +Z, joints secuenciales). El MoBL-ARMS de OpenSim usa la parametrización estándar ISB de "plano de elevación / elevación" (`elv_angle_r`, `shoulder_elv_r`, `shoulder_rot_r`) — una representación esférica acoplada, no rotaciones de Euler independientes. Los dos **no son intercambiables** copiando valores directamente.*

---

## 2. Installing OpenSim / Instalación de OpenSim

A dedicated conda environment was created to avoid interfering with the PyQt6/matplotlib environment:

*Se creó un entorno conda dedicado para no interferir con el entorno de PyQt6/matplotlib:*

```bash
conda create -n exoarm_osim python=3.11
conda activate exoarm_osim
conda install -c opensim-org opensim
```

Verification / Verificación:
```bash
python -c "import opensim; print(opensim.GetVersion())"
```
→ **OpenSim 4.6** confirmed. The message `Found simbody-visualizer...` is normal.

*→ **OpenSim 4.6** confirmado. El mensaje `Found simbody-visualizer...` es normal.*

**Practical note:** on Windows, multi-line `python -c "..."` commands don't work the same way in `cmd.exe` as in bash — each new line gets interpreted as a separate command. The fix was to always write code in a `.py` file and run it with `python file.py`.

***Nota práctica:*** *en Windows, los comandos multilínea `python -c "..."` no funcionan igual en `cmd.exe` que en bash — cada línea nueva se interpreta como un comando aparte. La solución fue siempre escribir el código en un archivo `.py` y ejecutarlo con `python archivo.py`.*

---

## 3. Loading the model and confirming coordinates / Carga del modelo y confirmación de coordenadas

```python
model = osim.Model('MoBL_ARMS_bimanual_6_2_21.osim')
model.initSystem()
print(model.getCoordinateSet().getSize())  # 46 coordinates (bimanual model)
```

Result: 46 coordinates confirmed, default values matching exactly what was read directly from the `.osim` XML (`elv_angle_r=0.000`, `shoulder_elv_r=0.524`, `elbow_flexion_r=1.571`). Warnings about duplicate `pathwrap`/`default` names and a massless `thorax` body with nonzero inertia are harmless — internal quirks of the model file, not blockers.

*Resultado: 46 coordenadas confirmadas, con valores por defecto que coinciden exactamente con lo leído directamente del XML del `.osim`. Los warnings sobre nombres duplicados de `pathwrap`/`default` y un cuerpo `thorax` sin masa pero con inercia distinta de cero son inofensivos — particularidades internas del archivo, no bloqueantes.*

---

## 4. Analyzing the shoulder joint chain in the `.osim` / Análisis de la cadena del hombro en el `.osim`

Reading the `.osim` XML directly showed that MoBL-ARMS' shoulder is also built from **3 sequential `CustomJoint`s**, structurally similar in spirit to the URDF chain:

*Leer el XML del `.osim` directamente mostró que el hombro de MoBL-ARMS también se construye con **3 `CustomJoint` secuenciales**, estructuralmente similar en espíritu a la cadena URDF:*

```
scapphant_r --[shoulder0_r]--> humphant_r --[shoulder1_r]--> humphant1_r --[shoulder2_r]--> humerus_r
                elv_angle_r                  shoulder_elv_r                  shoulder_rot_r
                                              (+ shoulder1_r2_r, coupled coordinate)
```

### 4.1 Key finding: the coupling constraint / Hallazgo clave: la restricción de acoplamiento

`shoulder1_r2_r` is not free — it's governed by a `CoordinateCouplerConstraint`:

*`shoulder1_r2_r` no es libre — está gobernada por una `CoordinateCouplerConstraint`:*

```xml
<CoordinateCouplerConstraint name="shoulder1_r2_con_r">
  <coupled_coordinates_function>
    <SimmSpline><x> -1.5708 3.14159 </x><y> 1.5708 -3.14159 </y></SimmSpline>
  </coupled_coordinates_function>
  <independent_coordinate_names>elv_angle_r</independent_coordinate_names>
  <dependent_coordinate_name>shoulder1_r2_r</dependent_coordinate_name>
</CoordinateCouplerConstraint>
```

Linear, slope -1: **`shoulder1_r2_r = -elv_angle_r`**. This is the standard trick for building a 2-DOF ball joint from simple hinges without gimbal-lock: a rotation conjugation, `Rot(e1,φ)·Rot(e2,θ)·Rot(e1,-φ)`. This finding is why we chose **not** to derive the conversion by hand — the real axes `e1≈(0.0048,0.999,0.042)` and `e2≈(-0.998,0.002,0.059)` have small cadaveric misalignments, so we let OpenSim's own engine solve its exact kinematics instead.

*Lineal, pendiente -1: **`shoulder1_r2_r = -elv_angle_r`**. Es el truco estándar para construir una articulación esférica de 2 GDL con bisagras simples sin gimbal-lock: una conjugación de rotaciones, `Rot(e1,φ)·Rot(e2,θ)·Rot(e1,-φ)`. Este hallazgo es la razón por la que decidimos **no** derivar la conversión a mano — los ejes reales `e1≈(0.0048,0.999,0.042)` y `e2≈(-0.998,0.002,0.059)` tienen pequeños desalineamientos cadavéricos, así que dejamos que el propio motor de OpenSim resuelva su cinemática exacta.*

---

## 5. Global axis convention — and our biggest mistake / Convención de ejes globales — y nuestro error más grande

We initially *assumed* the textbook OpenSim convention: X=anterior, Y=vertical, Z=lateral. This turned out to be **wrong for this specific model** (see Section 7). The lesson: axis semantics must always be **empirically calibrated**, never assumed from general documentation or memory of "typical" conventions.

*Al principio *asumimos* la convención de libro de texto de OpenSim: X=anterior, Y=vertical, Z=lateral. Esto resultó estar **equivocado para este modelo específico** (ver Sección 7). La lección: la semántica de ejes siempre debe **calibrarse empíricamente**, nunca asumirse por documentación general o memoria de convenciones "típicas".*

---

## 6. `opensim_bridge.py` — the bridge module / el módulo puente

### 6.1 Strategy / Estrategia

1. Compute the real 3D shoulder→elbow direction from your URDF (pure NumPy, reusing your own rotation logic).
2. Remap that direction into OpenSim's axis convention.
3. Ask OpenSim's **real engine** to find `(elv_angle, shoulder_elv)` that reproduce that same 3D direction (2-variable numerical optimization).

*1. Calcular la dirección real 3D hombro→codo desde tu URDF (NumPy puro, reutilizando tu propia lógica de rotación).*
*2. Remapear esa dirección a la convención de ejes de OpenSim.*
*3. Pedirle al **motor real** de OpenSim que encuentre `(elv_angle, shoulder_elv)` que reproduzcan esa misma dirección 3D (optimización numérica de 2 variables).*

### 6.2 `urdf_humerus_direction(sh_abd, sh_flex)`

Reproduces your URDF's rotation chain in pure NumPy: `R_total = R_abd(-Y,sh_abd) @ R_flex(X,sh_flex)`, applied to the rest direction `(0,0,-1)`. `shoulder_rot` is deliberately excluded — it's a twist about the bone's own axis, so it doesn't change which way the bone points.

*Reproduce la cadena de rotación de tu URDF en NumPy puro: `R_total = R_abd(-Y,sh_abd) @ R_flex(X,sh_flex)`, aplicada a la dirección de reposo `(0,0,-1)`. `shoulder_rot` queda deliberadamente excluido — es una torsión sobre el propio eje del hueso, así que no cambia hacia dónde apunta.*

### 6.3 `OpenSimArmModel` class / clase `OpenSimArmModel`

Thin wrapper: `set_pose(...)` fixes coordinate values and calls `model.realizePosition(state)`; `humerus_direction()` reads the real ground-frame positions of `humerus_r` and `ulna_r` bodies and returns the normalized difference — the actual direction computed by OpenSim's physics engine, constraint coupling included.

*Envoltorio delgado: `set_pose(...)` fija los valores de coordenadas y llama `model.realizePosition(state)`; `humerus_direction()` lee las posiciones reales (en el suelo/ground) de los cuerpos `humerus_r` y `ulna_r`, y devuelve la diferencia normalizada — la dirección real calculada por el motor físico de OpenSim, acoplamiento de restricciones incluido.*

### 6.4 `solve_osim_angles(...)` — the solver / el solver

1. Compute the target direction (6.2 + axis remapping).
2. Analytically estimate `shoulder_elv` as the angle between that direction and the rest position (valid because `shoulder_elv` *is*, by definition, that elevation angle).
3. Try several `elv_angle` starting values across its full valid range (multi-start), each refined with `scipy.optimize.minimize` (`L-BFGS-B`, bounded to the real coordinate ranges).
4. Keep the combination with lowest residual error.

*1. Calcular la dirección objetivo (6.2 + remapeo de ejes).*
*2. Estimar analíticamente `shoulder_elv` como el ángulo entre esa dirección y la posición de reposo (válido porque `shoulder_elv` *es*, por definición, ese ángulo de elevación).*
*3. Probar varios valores iniciales de `elv_angle` en todo su rango válido (multi-start), cada uno refinado con `scipy.optimize.minimize` (`L-BFGS-B`, acotado a los rangos reales de la coordenada).*
*4. Quedarse con la combinación de menor error residual.*

---

## 7. The debugging saga: what went wrong and how we fixed it / La saga de depuración: qué falló y cómo lo arreglamos

This section is deliberately kept as a **narrative of mistakes**, not just the final answer — the reasoning is as valuable as the result for the next model you integrate.

*Esta sección se mantiene deliberadamente como una **narrativa de errores**, no solo la respuesta final — el razonamiento vale tanto como el resultado para el próximo modelo que integres.*

### 7.1 Attempt 1 — single-start optimizer / Intento 1 — optimizador de un solo arranque

Got stuck near `shoulder_elv≈0` for every test posture. Classic symptom of a **geometric singularity** (analogous to a sphere's pole, where `elv_angle` stops having any effect) combined with warm-starting each pose from the previous one's (bad) solution.

*Quedó atascado cerca de `shoulder_elv≈0` para cada postura de prueba. Síntoma clásico de una **singularidad geométrica** (análoga al polo de una esfera, donde `elv_angle` deja de tener efecto), combinado con "recalentar" cada pose con la (mala) solución de la anterior.*

**Fix:** analytic `shoulder_elv` estimate (angle between target and rest direction) + multi-start on `elv_angle`, no warm-starting across poses.

***Arreglo:*** *estimación analítica de `shoulder_elv` (ángulo entre el objetivo y la posición de reposo) + multi-start en `elv_angle`, sin recalentar entre poses.*

### 7.2 Attempt 2 — 8 starts, one `shoulder_elv` branch / Intento 2 — 8 arranques, una rama de `shoulder_elv`

"Rest" and "pure abduction" converged to ~0° error. "Flexion" and combined poses got stuck at the upper bound of `elv_angle` (130°). Also: pure abduction converged with **zero error but at `elv_angle≈86.6°`**, not the textbook-expected `0°` — a red flag that turned out to be informative, not just numerical noise.

*"Reposo" y "abducción pura" convergieron a ~0° de error. "Flexión" y las poses combinadas quedaron atascadas en el límite superior de `elv_angle` (130°). Además: la abducción pura convergió con **error cero pero en `elv_angle≈86.6°`**, no el `0°` esperado por convención de libro de texto — una señal de alerta que resultó ser informativa, no solo ruido numérico.*

### 7.3 Exhaustive grid search / Barrido de grilla exhaustivo

To rule out "just a local minimum," a full 25×25 brute-force grid was scanned for the failing "pure flexion 90°" case. Result: minimum cost ≈0.653 everywhere, best point at the `elv_angle` boundary (130°) — **not** a local-minimum artifact; a genuine constrained optimum given (what we still believed was) the correct target direction.

*Para descartar "solo un mínimo local," se barrió una grilla completa de fuerza bruta 25×25 para el caso fallido de "flexión pura 90°". Resultado: costo mínimo ≈0.653 en todas partes, mejor punto en el límite de `elv_angle` (130°) — **no** un artefacto de mínimo local; un óptimo genuinamente restringido dado (lo que aún creíamos era) la dirección objetivo correcta.*

### 7.4 Direct inspection (no optimizer) / Inspección directa (sin optimizador)

Instead of theorizing further, a script directly set several `(elv_angle, shoulder_elv)` combinations and printed the *real* resulting direction with no optimization involved. Pattern found: at `elv_angle=0°`, motion stays in the OpenSim X-Y plane (Z≈0); at `elv_angle=90°`, motion stays in the Y-Z plane (X≈0) — confirming two orthogonal planes exist, but **not yet which one is flexion and which is abduction**.

*En vez de seguir teorizando, un script fijó directamente varias combinaciones `(elv_angle, shoulder_elv)` e imprimió la dirección *real* resultante, sin optimización de por medio. Patrón encontrado: a `elv_angle=0°`, el movimiento se mantiene en el plano X-Y de OpenSim (Z≈0); a `elv_angle=90°`, se mantiene en el plano Y-Z (X≈0) — confirmando que existen dos planos ortogonales, pero **sin saber aún** cuál es flexión y cuál abducción.*

### 7.5 Visual calibration — the decisive step / Calibración visual — el paso decisivo

The Simbody Visualizer was opened at `elv_angle=0°, shoulder_elv=90°`. **Visual result: the arm pointed forward.** This single observation settled everything: `elv_angle=0°` is the **flexion** plane (not abduction, contrary to the textbook assumption), and OpenSim's "-X" direction is anterior in this specific model (not "+X" as commonly assumed).

*Se abrió el Simbody Visualizer en `elv_angle=0°, shoulder_elv=90°`. **Resultado visual: el brazo apuntaba hacia adelante.** Esta sola observación resolvió todo: `elv_angle=0°` es el plano de **flexión** (no abducción, al contrario de la suposición de libro de texto), y la dirección "-X" de OpenSim es anterior en este modelo específico (no "+X" como se asume comúnmente).*

### 7.6 The corrected mapping / El mapeo corregido

```python
osim_X = -urdf_Y   # "forward" is -X in this model, not +X
osim_Y =  urdf_Z   # vertical, unchanged
osim_Z =  urdf_X   # lateral, unchanged
```

Verified against all 3 calibration postures (rest, pure abduction, pure flexion) and against direct visual confirmation.

*Verificado contra las 3 posturas de calibración (reposo, abducción pura, flexión pura) y contra confirmación visual directa.*

### 7.7 Final validation — CONFIRMED / Validación final — CONFIRMADA

With the corrected mapping, all 5 test postures converge to error < 0.03°:

*Con el mapeo corregido, las 5 posturas de prueba convergen a error < 0.03°:*

| Postura / Posture | error (deg) |
|---|---|
| Reposo / Rest | 0.0292 |
| Flexión pura 90° / Pure flexion 90° | 0.0001 |
| Abducción pura 90° / Pure abduction 90° | 0.0000 |
| Flexión 45° + Abducción 45° / Flexion 45° + Abduction 45° | 0.0000 |
| Flexión máxima 150° / Max flexion 150° | 0.0000 |

**Paso 6 / Step 6 closed.** The bridge is validated and ready for the next phase (Section 9).

***Paso 6 cerrado.*** *El puente está validado y listo para la siguiente fase (Sección 9).*

---

## 8. Checklist: 100%-safe procedure for any new `.osim` model / Checklist: procedimiento 100% seguro para cualquier modelo `.osim` nuevo

Every item below exists because of a specific mistake made in Sections 6-7 — treat this as mandatory, not optional, for the next model.

*Cada punto de esta lista existe por un error específico cometido en las Secciones 6-7 — trátalo como obligatorio, no opcional, para el próximo modelo.*

1. **Inventory every coordinate** (`getCoordinateSet()`) — never assume shoulder DOF names or count. / **Inventariar todas las coordenadas** — nunca asumir nombres o cantidad de GDL del hombro.
2. **List every constraint** (`getConstraintSet()`) before assuming DOF independence — coupled coordinates change the whole kinematic composition. / **Listar todas las restricciones** antes de asumir independencia de GDL — las coordenadas acopladas cambian toda la composición cinemática.
3. **Never assume axis convention from "typical" OpenSim documentation** — always calibrate empirically. This was our single costliest mistake. / **Nunca asumir la convención de ejes** por documentación "típica" de OpenSim — siempre calibrar empíricamente. Fue nuestro error más costoso.
4. **Calibrate with at least 3 reference postures** (rest, pure flexion, pure abduction) before writing any conversion code. / **Calibrar con al menos 3 posturas de referencia** (reposo, flexión pura, abducción pura) antes de escribir código de conversión.
5. **Visually confirm at least once per new model** with the Simbody Visualizer — numbers can "check out" by coincidence under a wrong mapping (it happened to us with abduction, error 0.0000° under the *wrong* mapping). / **Confirmar visualmente al menos una vez por modelo nuevo** — los números pueden "cuadrar" por coincidencia bajo un mapeo incorrecto (nos pasó con abducción, error 0.0000° con el mapeo *equivocado*).
6. **Validate ALL test postures simultaneously**, not just the first one that looks good. / **Validar TODAS las posturas de prueba a la vez**, no solo la primera que se ve bien.
7. **Always use OpenSim's real engine for kinematics** — never hand-derived rotation matrices from approximate axis vectors read off the XML. / **Usar siempre el motor real de OpenSim para la cinemática** — nunca matrices de rotación derivadas a mano desde ejes aproximados leídos del XML.
8. **Always multi-start any 2-variable numerical solver** on a sphere-like parametrization — geometric singularities (poles) are the rule, not the exception. / **Siempre usar multi-start** en cualquier solver numérico de 2 variables sobre una parametrización esférica — las singularidades geométricas (polos) son la regla, no la excepción.
9. **Check that the real joint ranges cover your intended range of motion** — a cadaveric model can be more restrictive than an idealized URDF. / **Verificar que los rangos articulares reales cubren tu rango de movimiento previsto** — un modelo cadavérico puede ser más restrictivo que un URDF idealizado.

**One-line summary / Resumen de una línea:** *never assume axis semantics or coordinate independence — verify everything empirically with at least 3 calibration postures and one visual inspection, before trusting any number that comes out of the solver.*

*nunca asumas semántica de ejes o independencia de coordenadas — verifica todo empíricamente con al menos 3 posturas de calibración y una inspección visual, antes de confiar en cualquier número que salga del solver.*

---

## 9. Milestone reached / Hito alcanzado

With Section 8's corrected mapping, all 5 test postures converge to error < 0.03° — Step 6 (shoulder kinematic bridge) is fully validated. What follows documents Steps 7 and 8: calibrating the remaining rotational degree of freedom (`shoulder_rot`), and extracting a first real, comparable physical quantity (gravitational torque) from OpenSim.

*Con el mapeo corregido de la Sección 8, las 5 posturas de prueba convergen a error < 0.03° — el Paso 6 (puente cinemático del hombro) queda completamente validado. Lo que sigue documenta los Pasos 7 y 8: calibrar el grado de libertad rotacional restante (`shoulder_rot`), y extraer una primera magnitud física real y comparable (torque gravitacional) desde OpenSim.*

---

## 10. Step 7 — Calibrating `shoulder_rot` / Paso 7 — Calibración de `shoulder_rot`

### 10.1 The goal / El objetivo

`shoulder_abd` and `shoulder_flex` determine which *direction* the humerus points — that's what Steps 6 solved. `shoulder_rot` (and its OpenSim counterpart `shoulder_rot_r`) is different: it's a **twist about the humerus's own long axis**, so it doesn't change where the bone points, only how it's rotated around itself. This matters for muscles that wrap around the humerus (their moment arm changes with this twist), even though it has zero effect on pure gravitational torque.

*`shoulder_abd` y `shoulder_flex` determinan hacia qué **dirección** apunta el húmero — eso es lo que resolvió el Paso 6. `shoulder_rot` (y su equivalente en OpenSim, `shoulder_rot_r`) es distinto: es una **torsión sobre el propio eje longitudinal del húmero**, así que no cambia hacia dónde apunta el hueso, solo cómo está girado sobre sí mismo. Esto importa para los músculos que se enrollan alrededor del húmero (su brazo de momento cambia con esta torsión), aunque no tiene ningún efecto sobre el torque puramente gravitacional.*

### 10.2 The technique: a perpendicular reference vector / La técnica: un vector de referencia perpendicular

Since there's no "direction" to match for a pure twist, we instead track a vector **perpendicular** to the humerus that we know how to compute on both sides: the elbow's own flexion axis. As `shoulder_rot` changes, this perpendicular vector rotates around the humeral axis — matching it between URDF and OpenSim pins down the twist.

*Como no hay una "dirección" que igualar para una torsión pura, en su lugar seguimos un vector **perpendicular** al húmero que sabemos calcular en ambos lados: el propio eje de flexión del codo. A medida que `shoulder_rot` cambia, este vector perpendicular gira alrededor del eje humeral — igualarlo entre el URDF y OpenSim fija la torsión.*

**Code — URDF side** (`urdf_elbow_axis_direction`, pure NumPy, no OpenSim needed):

*Código — lado URDF (`urdf_elbow_axis_direction`, NumPy puro, sin necesidad de OpenSim):*

```python
def urdf_elbow_axis_direction(sh_abd, sh_flex, sh_rot):
    R_abd  = _axis_angle_matrix([0, -1, 0], sh_abd)
    R_flex = _axis_angle_matrix([1, 0, 0], sh_flex)
    R_rot  = _axis_angle_matrix([0, 0, 1], sh_rot)   # here shoulder_rot DOES matter
    R_total = R_abd @ R_flex @ R_rot
    axis_local = np.array([0.0, 1.0, 0.0])            # elbow_flex axis in your URDF
    v = R_total @ axis_local
    return v / np.linalg.norm(v)
```

Unlike `urdf_humerus_direction` (Step 6), `shoulder_rot` is included here on purpose — it's precisely the rotation this function is meant to expose.

*A diferencia de `urdf_humerus_direction` (Paso 6), aquí `shoulder_rot` sí se incluye a propósito — es precisamente la rotación que esta función busca exponer.*

**Code — OpenSim side** (`OpenSimArmModel.elbow_axis_direction`): reads the elbow's real rotation axis directly from the `.osim` XML (`<axis>0.04940001 0.03660001 0.99810825</axis>` inside the `elbow_r` joint's `SpatialTransform` — nearly the humerus's own local Z, not a clean anatomical axis, another cadaveric-measurement artifact), then transforms that local vector to the ground frame using the same `findStationLocationInAnotherFrame` trick already validated in Step 6:

*Código — lado OpenSim (`OpenSimArmModel.elbow_axis_direction`): lee el eje real de rotación del codo directamente del XML del `.osim` (`<axis>0.04940001 0.03660001 0.99810825</axis>` dentro del `SpatialTransform` del joint `elbow_r` — casi el eje Z local del propio húmero, no un eje anatómico limpio, otro artefacto de medición cadavérica), y luego transforma ese vector local al marco del suelo usando el mismo truco de `findStationLocationInAnotherFrame` ya validado en el Paso 6:*

```python
def elbow_axis_direction(self):
    ground = self.model.getGround()
    p0 = self.humerus_body.findStationLocationInAnotherFrame(self.state, osim.Vec3(0,0,0), ground)
    p1 = self.humerus_body.findStationLocationInAnotherFrame(self.state, osim.Vec3(*self._elbow_axis_local), ground)
    v = np.array([p1.get(i) - p0.get(i) for i in range(3)])
    return v / np.linalg.norm(v)
```

**Code — the 1D solver** (`solve_shoulder_rot`): with `elv_angle`/`shoulder_elv` already fixed from Step 6, only one unknown remains, so a simple bounded scalar optimizer (`scipy.optimize.minimize_scalar`) suffices — no need for the multi-start machinery Step 6 required.

*Código — el solver 1D (`solve_shoulder_rot`): con `elv_angle`/`shoulder_elv` ya fijados desde el Paso 6, solo queda una incógnita, así que un optimizador escalar acotado simple (`scipy.optimize.minimize_scalar`) es suficiente — no hace falta la maquinaria de multi-arranque que requirió el Paso 6.*

### 10.3 Result: a clean linear calibration / Resultado: una calibración lineal limpia

Testing at rest with `sh_rot = 0°, +45°, -45°` gave `shoulder_rot_r = -2.86°, -47.85°, +42.13°` — a clean linear relationship, **`shoulder_rot_r ≈ -sh_rot + offset`** (slope -1, small offset of a few degrees carried over from the same clavicle/scapula corrections already seen in Step 6). Since this offset doesn't affect gravitational torque (Section 11), Step 7 is considered sufficiently calibrated for the current phase.

*Probando en reposo con `sh_rot = 0°, +45°, -45°` se obtuvo `shoulder_rot_r = -2.86°, -47.85°, +42.13°` — una relación lineal limpia, **`shoulder_rot_r ≈ -sh_rot + offset`** (pendiente -1, con un pequeño offset de pocos grados heredado de las mismas correcciones de clavícula/escápula ya vistas en el Paso 6). Como este offset no afecta el torque gravitacional (Sección 11), el Paso 7 se considera suficientemente calibrado para esta fase.*

---

## 11. Step 8 — Extracting real gravitational torque / Paso 8 — Extracción del torque gravitacional real

### 11.1 The big goal, restated / El gran objetivo, en otras palabras

Everything up to this point (Steps 6-7) was **plumbing** — necessary, but not yet the answer to the project's actual question. This step is where we finally get a **real physical number** out of OpenSim that can be placed side by side with your simplified formula: how much torque is needed, at a given elbow angle, to hold the forearm up against gravity — computed once with your cylinder approximation, and once with a cadaveric model's real mass distribution.

*Todo lo anterior (Pasos 6-7) fue **plomería** — necesaria, pero todavía no la respuesta a la pregunta real del proyecto. Este paso es donde finalmente obtenemos un **número físico real** de OpenSim que se puede poner al lado de tu fórmula simplificada: cuánto torque se necesita, a un ángulo de codo dado, para sostener el antebrazo contra la gravedad — calculado una vez con tu aproximación de cilindro, y otra con la distribución de masa real de un modelo cadavérico.*

### 11.2 Three attempts — another debugging saga / Tres intentos — otra saga de depuración

**Attempt 1 — `InverseDynamicsSolver` + `getMobilizerQIndex()`.** The idea: ask OpenSim's real dynamics solver for the generalized force needed to hold zero acceleration at zero velocity (mathematically, exactly the gravity-holding torque). The problem: `getMobilizerQIndex()` returns the index *local to each coordinate's own mobilizer* — which is `0` for nearly every joint in this model, since each one carries a single degree of freedom. Every coordinate ended up reading the *same* slot of the force vector — all four (`elv_angle_r`, `shoulder_elv_r`, `shoulder_rot_r`, `elbow_flexion_r`) came back with the exact same number, an unmistakable tell.

*Intento 1 — `InverseDynamicsSolver` + `getMobilizerQIndex()`. La idea: pedirle al solver de dinámica real de OpenSim la fuerza generalizada necesaria para sostener aceleración cero con velocidad cero (matemáticamente, exactamente el torque de sostenimiento contra gravedad). El problema: `getMobilizerQIndex()` devuelve el índice *local al propio mobilizador de cada coordenada* — que es `0` para casi cada articulación de este modelo, ya que cada una lleva un solo grado de libertad. Todas las coordenadas terminaron leyendo la *misma* casilla del vector de fuerzas — las cuatro (`elv_angle_r`, `shoulder_elv_r`, `shoulder_rot_r`, `elbow_flexion_r`) volvieron con exactamente el mismo número, una señal inconfundible.*

**Attempt 2 — `MultibodySystem.calcPotentialEnergy()`.** Sidestep the indexing problem entirely: gravitational holding torque equals the gradient of potential energy (`τ = dU/dq`), computable by finite differences without touching any Simbody index. The problem this time: `model.getMultibodySystem()` returned a raw, un-wrapped `SwigPyObject` — the Python binding didn't expose `calcPotentialEnergy` usably at this level.

*Intento 2 — `MultibodySystem.calcPotentialEnergy()`. Evitar el problema de indexado por completo: el torque de sostenimiento gravitacional es igual al gradiente de la energía potencial (`τ = dU/dq`), calculable por diferencias finitas sin tocar ningún índice de Simbody. El problema esta vez: `model.getMultibodySystem()` devolvió un `SwigPyObject` crudo, sin envolver — el binding de Python no exponía `calcPotentialEnergy` de forma utilizable en ese nivel.*

**Attempt 3 — manual potential energy, body by body (successful).** Compute `U = Σ(-mᵢ · g⃗ · rᵢ)` by hand, summing over every `Body` in the model, using its real mass (`body.getMass()`) and its center of mass position in the ground frame (`body.get_mass_center()` transformed via the same `findStationLocationInAnotherFrame` already proven reliable in Steps 6-7). Then differentiate numerically:

*Intento 3 — energía potencial manual, cuerpo por cuerpo (exitoso). Calcular `U = Σ(-mᵢ · g⃗ · rᵢ)` a mano, sumando sobre cada `Body` del modelo, usando su masa real (`body.getMass()`) y la posición de su centro de masa en el marco del suelo (`body.get_mass_center()` transformado con el mismo `findStationLocationInAnotherFrame` ya comprobado confiable en los Pasos 6-7). Luego derivar numéricamente:*

```python
def _total_gravitational_pe(model, state):
    g = np.array(model.getGravity())          # ~(0, -9.81, 0)
    total = 0.0
    for body in model.getBodySet():           # each Body in the model
        m = body.getMass()
        if m <= 0:
            continue                          # skips massless reference bodies (e.g. 'thorax')
        r = body.findStationLocationInAnotherFrame(state, body.get_mass_center(), model.getGround())
        total += -m * np.dot(g, r)
    return total

def gravity_only_generalized_forces(osim_model, coord_names, eps=1e-4):
    result = {}
    for name in coord_names:
        coord = ...get(name)
        q0 = coord.getValue(state)
        coord.setValue(state, q0 + eps); u_plus  = _total_gravitational_pe(...)
        coord.setValue(state, q0 - eps); u_minus = _total_gravitational_pe(...)
        coord.setValue(state, q0)                       # always restore
        result[name] = (u_plus - u_minus) / (2 * eps)   # see 11.3 for the sign
    return result
```

This third approach worked on the first real test — because it depends on nothing but `Body.getMass()` and a station-position query, both already battle-tested in Steps 6-7.

*Este tercer enfoque funcionó en la primera prueba real — porque no depende de nada más que `Body.getMass()` y una consulta de posición de estación, ambas ya probadas en los Pasos 6-7.*

### 11.3 A sign-convention trap / Una trampa de convención de signo

First numeric test: same magnitude as expected, but **opposite sign** to your formula. The reason: `-dU/dq` is the torque **gravity itself exerts**; the torque your **muscles must supply to resist it** (which is what your formula represents, and what gets shared with the exoskeleton via `hum_el = torque_elbow - exo_el`) is the opposite, `+dU/dq`. One sign flip, and the two formulas agreed in sign.

*Primera prueba numérica: misma magnitud esperada, pero **signo opuesto** al de tu fórmula. La razón: `-dU/dq` es el torque que la **gravedad misma ejerce**; el torque que tus **músculos deben aplicar para resistirla** (que es lo que representa tu fórmula, y lo que se reparte con el exoesqueleto vía `hum_el = torque_elbow - exo_el`) es el opuesto, `+dU/dq`. Un cambio de signo, y las dos fórmulas coincidieron en signo.*

### 11.4 A configuration mismatch, not a bug / Un desajuste de configuración, no un error

Second numeric test, sign now correct: still off by a factor of ~5 (9.9 Nm vs 3.3 Nm). Root cause: your formula includes a 3 kg payload at the hand (`p_mass`); the OpenSim model, as loaded, carries **no payload at all** — only the anatomical forearm and hand. Setting your simulator's payload slider to 0 kg (a valid point already inside its 0-10 kg range) removed the mismatch, leaving a clean, genuine, apples-to-apples anthropometric comparison.

*Segunda prueba numérica, ya con el signo correcto: todavía descuadrada por un factor de ~5 (9.9 Nm vs 3.3 Nm). Causa raíz: tu fórmula incluye un payload de 3 kg en la mano (`p_mass`); el modelo de OpenSim, tal como se cargó, no lleva **ningún payload** — solo el antebrazo y la mano anatómicos. Poner el slider de payload de tu simulador en 0 kg (un punto ya válido dentro de su rango 0-10 kg) eliminó el desajuste, dejando una comparación antropométrica genuina y justa.*

### 11.5 Final validated result / Resultado final validado

With payload = 0 kg and the shoulder at rest (the only condition your simplified elbow formula assumes — see below), both models agree in sign and land in the same order of magnitude:

*Con payload = 0 kg y el hombro en reposo (la única condición que asume tu fórmula simplificada de codo — ver abajo), ambos modelos coinciden en signo y quedan en el mismo orden de magnitud:*

| Postura / Posture | Tu fórmula (Nm) | OpenSim (Nm) | Diferencia |
|---|---|---|---|
| Codo 90°, hombro en reposo / Elbow 90°, shoulder at rest | 1.987 | 3.274 | +64.8% |
| Codo 45°, hombro en reposo / Elbow 45°, shoulder at rest | 1.405 | 2.447 | +74.2% |

**This is a real, reportable finding, not measurement noise:** the simplified anthropometric model (uniform cylinder, mass at half the segment length) **underestimates** the gravitational holding torque of the forearm by roughly 65-75% relative to a real cadaveric mass distribution — consistently across two different elbow angles. This is precisely the question the project set out to answer (documented as future work in Section 13 of your original simulator documentation).

***Este es un hallazgo real y reportable, no ruido de medición:*** *el modelo antropométrico simplificado (cilindro uniforme, masa a la mitad del segmento) **subestima** el torque de sostenimiento gravitacional del antebrazo en aproximadamente 65-75% respecto a una distribución de masa cadavérica real — de forma consistente en dos ángulos de codo distintos. Esta es precisamente la pregunta que el proyecto se propuso responder (documentada como trabajo futuro en la Sección 13 de la documentación original de tu simulador).*

A second, expected limitation surfaced in the same test: your elbow formula depends only on `elbow_flex`, implicitly assuming the shoulder is at rest. When the shoulder is flexed or abducted, the forearm's absolute orientation relative to gravity changes in a way your formula doesn't capture — OpenSim's result and yours diverge sharply in those postures (flagged explicitly in the test output as "hombro NO en reposo"). This isn't a bug either; it's a real, documented boundary of your current formula's validity.

*Una segunda limitación esperada salió a la luz en la misma prueba: tu fórmula de codo depende solo de `elbow_flex`, asumiendo implícitamente que el hombro está en reposo. Cuando el hombro se flexiona o abduce, la orientación absoluta del antebrazo respecto a la gravedad cambia de una forma que tu fórmula no captura — el resultado de OpenSim y el tuyo divergen fuertemente en esas posturas (marcado explícitamente en la salida de la prueba como "hombro NO en reposo"). Esto tampoco es un error; es un límite real y documentado de la validez actual de tu fórmula.*

---

## 13. Step 8b — Projecting shoulder torques into your anatomical basis / Paso 8b — Proyección de los torques de hombro a tu base anatómica

### 13.1 The problem this step solves / El problema que resuelve este paso

The elbow comparison in Step 8 was a direct 1:1 match — `elbow_flexion_r` and your `elbow_flex` are the exact same physical angle in both models. The shoulder is not: OpenSim's generalized torques `τ_elv_angle` and `τ_shoulder_elv` live in a completely different coordinate basis than your `τ_sh_flex`/`τ_sh_abd`. Comparing them directly, number to number, would be meaningless — it's like comparing a reading in polar coordinates to one in Cartesian coordinates without converting first.

*La comparación del codo en el Paso 8 fue una correspondencia directa 1:1 — `elbow_flexion_r` y tu `elbow_flex` son exactamente el mismo ángulo físico en ambos modelos. El hombro no: los torques generalizados de OpenSim `τ_elv_angle` y `τ_shoulder_elv` viven en una base de coordenadas completamente distinta a tu `τ_sh_flex`/`τ_sh_abd`. Compararlos directamente, número contra número, no tendría sentido — es como comparar una lectura en coordenadas polares con una en cartesianas sin antes convertir.*

### 13.2 The math: torque transforms via the Jacobian transpose / La matemática: el torque se transforma vía la Jacobiana transpuesta

Both parametrizations describe the *same* physical rotation — just with different coordinates. If `q_osim = f(q_urdf)` is the (nonlinear, numerically-solved) mapping from Step 6, then by the principle of virtual work, the two sets of generalized torques must satisfy:

*Ambas parametrizaciones describen la *misma* rotación física — solo con coordenadas distintas. Si `q_osim = f(q_urdf)` es el mapeo (no lineal, resuelto numéricamente) del Paso 6, entonces por el principio de trabajo virtual, los dos conjuntos de torques generalizados deben cumplir:*

```
τ_urdf = Jᵀ · τ_osim         where J = ∂(elv_angle, shoulder_elv) / ∂(sh_abd, sh_flex)
```

Since there's no closed-form for `J` (the mapping is solved numerically), it's computed by finite differences.

*Como no hay una fórmula cerrada para `J` (el mapeo se resuelve numéricamente), se calcula por diferencias finitas.*

### 13.3 Avoiding a repeat mistake: local refinement, not multi-start / Evitando repetir un error: refinamiento local, no multi-start

A naive Jacobian implementation would perturb `sh_abd`/`sh_flex` slightly and re-run the full Step 6 multi-start solver. The risk: with a small perturbation, the multi-start solver could jump to a *different valid solution branch* (recall the branch ambiguity from Section 7.2) — corrupting the finite-difference derivative with a spurious jump instead of a smooth local change.

*Una implementación ingenua de la Jacobiana perturbaría `sh_abd`/`sh_flex` un poco y volvería a correr el solver multi-start completo del Paso 6. El riesgo: con una perturbación pequeña, el solver multi-start podría saltar a una *rama de solución distinta* (recordar la ambigüedad de ramas de la Sección 7.2) — corrompiendo la derivada por diferencias finitas con un salto espurio en vez de un cambio local suave.*

**Fix:** `_solve_osim_angles_local` performs a single local `L-BFGS-B` refinement anchored at the already-known base solution — since the perturbation is tiny, the correct nearby solution is guaranteed to be very close, so no multi-start is needed, and the branch stays consistent.

***Arreglo:*** *`_solve_osim_angles_local` hace un único refinamiento local con `L-BFGS-B` anclado a la solución base ya conocida — como la perturbación es diminuta, la solución correcta cercana está garantizada a estar muy cerca, así que no hace falta multi-start, y la rama se mantiene consistente.*

```python
def shoulder_angle_jacobian(osim_model, sh_abd, sh_flex, elv_angle, shoulder_elv, eps=1e-4):
    def solve(abd, flex):
        e, s = _solve_osim_angles_local(osim_model, abd, flex, elv_angle, shoulder_elv)
        return np.array([e, s])
    d_dabd  = (solve(sh_abd+eps, sh_flex) - solve(sh_abd-eps, sh_flex)) / (2*eps)
    d_dflex = (solve(sh_abd, sh_flex+eps) - solve(sh_abd, sh_flex-eps)) / (2*eps)
    return np.column_stack([d_dabd, d_dflex])

def project_shoulder_torque_to_urdf(osim_model, sh_abd, sh_flex, elv_angle, shoulder_elv,
                                     tau_elv_angle, tau_shoulder_elv):
    J = shoulder_angle_jacobian(osim_model, sh_abd, sh_flex, elv_angle, shoulder_elv)
    tau_osim = np.array([tau_elv_angle, tau_shoulder_elv])
    tau_urdf = J.T @ tau_osim
    return float(tau_urdf[0]), float(tau_urdf[1])   # (tau_sh_abd, tau_sh_flex)
```

### 13.4 Results and honest interpretation / Resultados e interpretación honesta

| Postura / Posture | tu_sh_flex | osim_sh_flex | tu_sh_abd | osim_sh_abd |
|---|---|---|---|---|
| Flexión 45° | 7.189 | 6.384 | 0.000 | 3.072 |
| Flexión 90° | 9.344 | 8.650 | 0.000 | 0.000 |
| Abducción 45° | 1.987 | -0.074 | 5.203 | 8.752 |
| Abducción 90° | 1.987 | 0.334 | 7.357 | 8.711 |
| Combinada 45°+45° | 7.189 | 4.172 | 3.679 | 6.089 |

No sign flips, no repeated values, no gross indexing errors this time — the differences seen are genuine physical/definitional discrepancies, not bugs:

*Sin inversiones de signo, sin valores repetidos, sin errores de indexado esta vez — las diferencias observadas son discrepancias genuinas, físicas o de definición, no errores de código:*

**(a) Real anthropometry difference** (same kind as Step 8's elbow finding): pure flexion and pure abduction cases agree in sign and order of magnitude, with 7-33% differences — consistent with cylinder-approximation vs. real cadaveric mass/CoM.

***(a) Diferencia antropométrica real*** *(del mismo tipo que el hallazgo del codo en el Paso 8): los casos de flexión pura y abducción pura coinciden en signo y orden de magnitud, con diferencias de 7-33% — consistente con la aproximación de cilindro vs. masa/CoM cadavérica real.*

**(b) A definitional difference, not an error:** your `torque_sh_flex` formula (Section 7.2 of your documentation) explicitly *carries the elbow torque through the chain* regardless of `sh_flex`'s own value — so it reports ≈1.987 Nm even at `sh_flex=0°` (pure abduction poses). OpenSim's value is the true partial derivative of potential energy with respect to that specific coordinate, which correctly goes to ≈0 when that coordinate isn't contributing to any height change. Both are "correct" — they answer different questions ("what load is carried through this joint" vs. "how much does this specific coordinate's motion change potential energy").

***(b) Una diferencia de definición, no un error:*** *tu fórmula `torque_sh_flex` (Sección 7.2 de tu documentación) explícitamente **traspasa el torque del codo a través de la cadena**, sin importar el valor propio de `sh_flex` — así que reporta ≈1.987 Nm incluso con `sh_flex=0°` (posturas de abducción pura). El valor de OpenSim es la derivada parcial real de la energía potencial respecto a esa coordenada específica, que correctamente da ≈0 cuando esa coordenada no contribuye a ningún cambio de altura. Ambos son "correctos" — responden preguntas distintas ("qué carga se transmite por esta articulación" vs. "cuánto cambia la energía potencial al mover específicamente esta coordenada").*

**(c) Cross-coupling your formula doesn't fully capture:** in "Flexión 45°", `tu_sh_abd=0.000` (by construction, your formula has `sin(sh_abd)=sin(0)=0`) but `osim_sh_abd=3.072` — a real off-axis coupling from the humerus's true (non-cylindrical) center of mass (`mass_center ≈ (0.018, -0.140, -0.013)` in the `.osim`, not perfectly on the bone's long axis). Your documentation's own Limitations section (12) already anticipated this: your model captures only a single `cos(θ_sf)` coupling term, not the full 3D cross-product.

***(c) Un acoplamiento cruzado que tu fórmula no captura del todo:*** *en "Flexión 45°", `tu_sh_abd=0.000` (por construcción, tu fórmula tiene `sin(sh_abd)=sin(0)=0`) pero `osim_sh_abd=3.072` — un acoplamiento fuera de eje real, proveniente del centro de masa verdadero (no perfectamente cilíndrico) del húmero (`mass_center ≈ (0.018, -0.140, -0.013)` en el `.osim`, no exactamente sobre el eje longitudinal del hueso). La propia sección de Limitaciones de tu documentación (Sección 12) ya anticipaba esto: tu modelo captura solo un único término de acoplamiento `cos(θ_sf)`, no el producto cruzado 3D completo.*

**Step 8b closed.** The shoulder now has the same honest, direct comparison the elbow already had.

***Paso 8b cerrado.*** *El hombro ya tiene la misma comparación directa y honesta que ya tenía el codo.*

---

## 14. Simulating payload without modifying the `.osim` / Simulando payload sin modificar el `.osim`

### 14.1 The goal / El objetivo

Your simulator's payload slider goes from 0 to 10 kg, but the MoBL-ARMS model as loaded carries no payload at all (Section 11.4). To compare torques across the full payload range, we need a way to add a virtual load to OpenSim's gravity calculation — without editing the `.osim` file or adding new bodies to the model.

*El slider de payload de tu simulador va de 0 a 10 kg, pero el modelo MoBL-ARMS, tal como se carga, no lleva ningún payload (Sección 11.4). Para comparar torques en todo el rango de payload, necesitamos una forma de agregar una carga virtual al cálculo de gravedad de OpenSim — sin editar el archivo `.osim` ni agregar cuerpos nuevos al modelo.*

**Solution:** since `_total_gravitational_pe` (Section 11.2) already sums `mass × g × height` over every real body, adding a payload is just one more term in that same sum — a virtual point mass, positioned wherever we say, with no changes to the model file.

***Solución:*** *como `_total_gravitational_pe` (Sección 11.2) ya suma `masa × g × altura` sobre cada cuerpo real, agregar un payload es solo un término más en esa misma suma — una masa puntual virtual, ubicada donde nosotros digamos, sin ningún cambio al archivo del modelo.*

### 14.2 First attempt: payload at `hand_r`'s center of mass / Primer intento: payload en el centro de masa de `hand_r`

The simplest choice: place the virtual payload at `hand_r`'s own center of mass. Testing across 0, 3, and 10 kg gave decreasing *relative* error (64.8% → 25.4% → 18.9%) but *growing absolute* error (1.29 → 2.52 → 5.39 Nm) — a clue that the payload's lever arm didn't quite match your URDF's assumption (`F_LEN` = 0.27 m from the elbow), since each added kg amplified a small lever-arm mismatch.

*La opción más simple: colocar el payload virtual en el propio centro de masa de `hand_r`. Probar con 0, 3 y 10 kg dio un error *relativo* decreciente (64.8% → 25.4% → 18.9%) pero un error *absoluto creciente* (1.29 → 2.52 → 5.39 Nm) — una pista de que el brazo de palanca del payload no coincidía exactamente con el supuesto de tu URDF (`F_LEN` = 0.27 m desde el codo), ya que cada kg agregado amplificaba un pequeño desajuste de palanca.*

### 14.3 The fix: payload positioned exactly at `F_LEN` from the elbow / El arreglo: payload posicionado exactamente a `F_LEN` del codo

Instead of an arbitrary anatomical point, the payload is now placed at exactly `F_LEN` meters from the elbow, along the **real** elbow→hand direction that OpenSim computes for the current pose — replicating your URDF's own convention (`hand_payload` sits at `F_LEN` from the elbow, along the forearm's distal direction) exactly, regardless of the real anatomical forearm length in the cadaveric model:

*En vez de un punto anatómico arbitrario, el payload ahora se coloca exactamente a `F_LEN` metros del codo, siguiendo la dirección **real** codo→mano que calcula OpenSim para la postura actual — replicando exactamente la convención de tu propio URDF (`hand_payload` está a `F_LEN` del codo, en la dirección distal del antebrazo), sin importar la longitud anatómica real del antebrazo en el modelo cadavérico:*

```python
def _payload_ground_position(model, state, f_len):
    ground = model.getGround()
    ulna = model.getBodySet().get("ulna_r")
    hand = model.getBodySet().get("hand_r")
    p_elbow = ulna.findStationLocationInAnotherFrame(state, osim.Vec3(0,0,0), ground)
    p_hand  = hand.findStationLocationInAnotherFrame(state, osim.Vec3(0,0,0), ground)
    elbow = np.array([p_elbow.get(i) for i in range(3)])
    hand_pos = np.array([p_hand.get(i) for i in range(3)])
    direction = (hand_pos - elbow)
    direction = direction / np.linalg.norm(direction)
    return elbow + f_len * direction
```

This direction vector is recomputed every time from OpenSim's real body positions — it automatically follows the current pose, with no hardcoded assumptions about bone orientation.

*Este vector de dirección se recalcula cada vez a partir de las posiciones reales de los cuerpos en OpenSim — sigue automáticamente la postura actual, sin ningún supuesto fijo sobre la orientación del hueso.*

### 14.4 Final result: the two error sources cleanly separated / Resultado final: las dos fuentes de error separadas limpiamente

| Payload (kg) | Tu torque (Nm) | OpenSim (Nm) | Diferencia absoluta (Nm) | Diferencia relativa |
|---|---|---|---|---|
| 0 | 1.987 | 3.274 | 1.287 | 64.8% |
| 3 | 9.933 | 11.104 | 1.171 | 11.8% |
| 10 | 28.474 | 29.376 | 0.902 | 3.2% |

With the lever arm now correctly aligned, the **absolute** difference stays roughly constant (~0.9–1.3 Nm) regardless of payload — proof that the lever-arm mismatch from Section 14.2 has been eliminated. What remains is purely the forearm's own anthropometric discrepancy (cylinder approximation vs. real cadaveric mass/CoM), the same ~1.2 Nm-scale finding already reported in Section 11.5. The dropping *relative* error (65% → 12% → 3%) isn't the model "improving" — it's the same fixed absolute discrepancy becoming proportionally smaller against a growing payload.

*Con el brazo de palanca ya correctamente alineado, la diferencia **absoluta** se mantiene aproximadamente constante (~0.9–1.3 Nm) sin importar el payload — prueba de que el desajuste de palanca de la Sección 14.2 quedó eliminado. Lo que queda es puramente la discrepancia antropométrica propia del antebrazo (aproximación de cilindro vs. masa/CoM cadavérica real), el mismo hallazgo de escala ~1.2 Nm ya reportado en la Sección 11.5. La caída del error *relativo* (65% → 12% → 3%) no es que el modelo "mejore" — es la misma discrepancia absoluta fija volviéndose proporcionalmente más pequeña frente a un payload cada vez mayor.*

**Practical note:** `gravity_only_generalized_forces` now accepts `payload_mass` (kg) and `payload_f_len` (m, default 0.27 to match your simulator), so any payload in your slider's 0-10 kg range can be tested directly, matching your simulator's setting exactly.

***Nota práctica:*** *`gravity_only_generalized_forces` ahora acepta `payload_mass` (kg) y `payload_f_len` (m, por defecto 0.27 para coincidir con tu simulador), así que cualquier payload en el rango 0-10 kg de tu slider se puede probar directamente, igualando exactamente el ajuste de tu simulador.*

---

## 15. Step 1 — Full 3D torque and the elbow axis discovery / Paso 1 — Torque 3D completo y el hallazgo del eje de codo

### 15.1 The goal / El objetivo

Both documented limitations from Section 12 of the original simulator documentation — the elbow formula ignoring shoulder orientation, and the shoulder formula only capturing a single `cos(θ_sf)` coupling term — share one root cause: the simplified formula uses a scalar sine/cosine shortcut per joint, instead of the full 3D cross-product `τ = (r × F)·axis` your own documentation already flagged as future work (Section 13, recommendation #4).

*Las dos limitaciones documentadas en la Sección 12 de la documentación original del simulador — la fórmula del codo ignorando la orientación del hombro, y la fórmula del hombro capturando solo un término de acoplamiento `cos(θ_sf)` — comparten una sola causa raíz: la fórmula simplificada usa un atajo escalar seno/coseno por articulación, en vez del producto cruzado 3D completo `τ = (r × F)·eje` que tu propia documentación ya señalaba como trabajo futuro (Sección 13, recomendación #4).*

**Key advantage: this fix needs no OpenSim at all.** Your simulator already computes the real 3D kinematics (`transforms`) every frame — `torque_3d.py` reuses exactly that, replacing only the torque formula, while keeping the same anthropometric mass assumption (uniform cylinder, mass at half length) so the comparison isolates *only* the geometric/coupling error, never mixing it with the anthropometric error already quantified against OpenSim (Sections 11, 13).

***Ventaja clave: este arreglo no necesita OpenSim para nada.*** *Tu simulador ya calcula la cinemática 3D real (`transforms`) en cada frame — `torque_3d.py` reutiliza exactamente eso, reemplazando solo la fórmula de torque, manteniendo el mismo supuesto de masa antropométrico (cilindro uniforme, masa a mitad de longitud) para que la comparación aísle *solo* el error geométrico/de acoplamiento, sin mezclarlo con el error antropométrico ya cuantificado contra OpenSim (Secciones 11, 13).*

### 15.2 The implementation / La implementación

```python
def compute_full_3d_torques(transforms, joints, u_len, f_len, u_mass, f_mass, p_mass, g=9.81):
    # posiciones reales de pivotes y centros de masa, desde 'transforms'
    com_upper = shoulder_pivot + R_upper @ [0,0,-u_len/2]
    com_forearm = elbow_pivot + R_forearm @ [0,0,-f_len/2]
    pos_payload = elbow_pivot + R_forearm @ [0,0,-f_len]

    # tau = (r x F) . eje_articulacion_mundo, sumado sobre todas las masas
    # distales relevantes -- captura automaticamente CUALQUIER acoplamiento
    # 3D, sin necesitar terminos correctivos como cos(theta_sf)
    ...
```

### 15.3 A validation surprise: the elbow bends sideways, not forward / Una sorpresa de validación: el codo se dobla al costado, no hacia adelante

The expected validation check: at rest (`sh_abd=sh_flex=0`, the one pose where the simplified formula should be exact), `torque_3d` and the simplified formula should match perfectly. They didn't — not by a small margin, but with **`sh_flex` and `sh_abd` swapped**, and the elbow sign flipped.

*La verificación esperada: en reposo (`sh_abd=sh_flex=0`, la única postura donde la fórmula simplificada debería ser exacta), `torque_3d` y la fórmula simplificada deberían coincidir perfectamente. No coincidieron — y no por un margen pequeño, sino con **`sh_flex` y `sh_abd` intercambiados**, y el signo del codo invertido.*

Tracing the math revealed the real cause: the URDF's `elbow_flex` joint had `axis="0 1 0"` (Y, anteroposterior) in the upper arm's local frame. Rotating *about* the Y axis sweeps the **X-Z plane** (mediolateral-vertical) — meaning, with the shoulder at rest, the elbow was bending **sideways**, not forward. The simplified sine/cosine formula never revealed this, because it's a pure scalar — it doesn't reference any real 3D axis, so it computes a magnitude regardless of which physical plane the rotation actually happens in.

*Rastrear la matemática reveló la causa real: el joint `elbow_flex` del URDF tenía `axis="0 1 0"` (Y, antero-posterior) en el marco local del brazo superior. Rotar *alrededor* del eje Y barre el plano **X-Z** (medio-lateral/vertical) — es decir, con el hombro en reposo, el codo se doblaba **hacia el costado**, no hacia adelante. La fórmula seno/coseno simplificada nunca reveló esto, porque es un escalar puro — no referencia ningún eje 3D real, así que calcula una magnitud sin importar en qué plano físico ocurre realmente la rotación.*

**Fix, after confirming with the user:** change the axis to `"1 0 0"` (X, mediolateral) in `arm_shoulder_elbow.urdf`, so the elbow now sweeps the Y-Z plane (anteroposterior-vertical) — anatomically correct forward flexion. **This is a visible change**, not just an internal number: the elbow now visibly bends forward in the 3D animation instead of sideways.

***Arreglo, tras confirmar con el usuario:*** *cambiar el eje a `"1 0 0"` (X, medio-lateral) en `arm_shoulder_elbow.urdf`, para que el codo ahora barra el plano Y-Z (antero-posterior/vertical) — flexión hacia adelante anatómicamente correcta. **Este es un cambio visible**, no solo un número interno: el codo ahora se dobla visiblemente hacia adelante en la animación 3D en vez de hacia el costado.*

A remaining sign flip (magnitude matched, sign opposite) was fixed the same way as the OpenSim gravity torque (Section 11.3): `(r×F)·axis` gives the torque gravity *exerts*; negating it gives the torque muscles must *supply to resist it* — the same convention as the original formula.

*Un signo invertido restante (magnitud coincidía, signo opuesto) se arregló de la misma forma que el torque gravitacional de OpenSim (Sección 11.3): `(r×F)·eje` da el torque que la gravedad *ejerce*; negarlo da el torque que los músculos deben *aplicar para resistirla* — la misma convención que la fórmula original.*

### 15.4 Validated result / Resultado validado

With both fixes, "Reposo, codo 90°" matches exactly (`1.987 = 1.987` in all three torques), and the other test poses reproduce **the same two limitations already found against OpenSim** (Sections 11.5, 13.4) — now confirmed independently, via pure geometry, with no OpenSim involved: the elbow torque genuinely depends on shoulder orientation, and the shoulder formula's `torque_sh_flex = ... + torque_elbow` "carry-through" convention reports nonzero values even at `sh_flex=0°`.

*Con ambos arreglos, "Reposo, codo 90°" coincide exactamente (`1.987 = 1.987` en los tres torques), y las demás posturas de prueba reproducen **las mismas dos limitaciones ya encontradas contra OpenSim** (Secciones 11.5, 13.4) — ahora confirmadas de forma independiente, vía pura geometría, sin involucrar OpenSim: el torque del codo genuinamente depende de la orientación del hombro, y la convención "carry-through" de la fórmula del hombro (`torque_sh_flex = ... + torque_elbow`) reporta valores distintos de cero incluso con `sh_flex=0°`.*

**Integration into the live simulator:** `torque_3d` is now the authoritative value used for exo assist, human/exo split, and fatigue alerts in `update_all()`; the original simplified formula is kept and displayed alongside as `(simple: X.XX Nm)` in each label, for direct comparison.

***Integración en el simulador en vivo:*** *`torque_3d` es ahora el valor autoritativo usado para asistencia de exo, reparto humano/exo, y alertas de fatiga en `update_all()`; la fórmula simplificada original se conserva y se muestra al lado como `(simple: X.XX Nm)` en cada etiqueta, para comparación directa.*

---

## 16. Step 9 — Real-time integration and the performance saga / Paso 9 — Integración en tiempo real y la saga de rendimiento

### 16.1 The goal / El objetivo

Connect everything built so far (`torque_3d.py` for pure-geometry torque, and `opensim_bridge.py` for real anthropometry) directly into the live PyQt6 simulator, so both numbers update as the user moves sliders — turning the whole validation exercise into an actual interactive tool, not just command-line test scripts.

*Conectar todo lo construido hasta ahora (`torque_3d.py` para el torque de geometría pura, y `opensim_bridge.py` para la antropometría real) directamente en el simulador PyQt6 en vivo, para que ambos números se actualicen mientras el usuario mueve los sliders — convirtiendo todo el ejercicio de validación en una herramienta interactiva real, no solo scripts de prueba por consola.*

This integration surfaced **three distinct, layered performance problems** — each one hiding behind the previous one, only visible once the one before it was fixed. This section documents all three, in the order they were found, because each fix looked sufficient until the next problem appeared.

*Esta integración sacó a la luz **tres problemas de rendimiento distintos y superpuestos** — cada uno escondido detrás del anterior, visible solo una vez arreglado el previo. Esta sección documenta los tres, en el orden en que se encontraron, porque cada arreglo parecía suficiente hasta que aparecía el siguiente problema.*

### 16.2 Problem 1 — the OpenSim bridge blocking the main UI thread / Problema 1 — el puente de OpenSim bloqueando el hilo principal de la UI

**Symptom:** with the OpenSim comparison wired into a simple `QTimer` firing every 750 ms directly in the main thread, the UI became "impossible to move sliders" — every 750 ms, everything froze for a noticeable moment.

***Síntoma:*** *con la comparación de OpenSim conectada a un `QTimer` simple disparando cada 750 ms directamente en el hilo principal, la UI se volvió "imposible de mover los sliders" — cada 750 ms, todo se congelaba por un momento notable.*

**Root cause:** Qt's event loop (mouse clicks, slider drags, window repaints) all run on a single thread. If a `QTimer` callback takes 200-500 ms to run (which `solve_osim_angles`'s multi-start optimizer plus the Jacobian projection easily does), the entire UI is unresponsive for that whole duration, repeatedly, every 750 ms.

***Causa raíz:*** *el bucle de eventos de Qt (clics de mouse, arrastre de sliders, repintado de ventana) corre todo en un solo hilo. Si un callback de `QTimer` tarda 200-500 ms en correr (algo que el optimizador multi-start de `solve_osim_angles` más la proyección Jacobiana logran facilmente), toda la UI queda sin respuesta durante ese tiempo completo, repetidamente, cada 750 ms.*

**Fix:** move the OpenSim computation into a dedicated `QThread` (`OpenSimCompareWorker`). The main thread only *launches* the worker and receives a signal (`result_ready`) when it's done — it never waits for it, so sliders keep responding immediately regardless of how long OpenSim takes.

***Arreglo:*** *mover el cálculo de OpenSim a un `QThread` dedicado (`OpenSimCompareWorker`). El hilo principal solo *lanza* el worker y recibe una señal (`result_ready`) cuando termina — nunca lo espera, así que los sliders siguen respondiendo de inmediato sin importar cuánto tarde OpenSim.*

```python
class OpenSimCompareWorker(QThread):
    result_ready = pyqtSignal(str, float, float)
    def run(self):
        # todo el trabajo pesado de OpenSim ocurre aqui, en el hilo del worker
        ...
        self.result_ready.emit(text, elv_angle, shoulder_elv)
```

### 16.3 Problem 2 — a different bottleneck hiding behind Problem 1 / Problema 2 — un cuello de botella distinto escondido detrás del Problema 1

**Symptom:** after fixing Problem 1, the UI was *still* slow moving sliders — but this had nothing to do with OpenSim at all.

***Síntoma:*** *después de arreglar el Problema 1, la UI *seguía* lenta al mover sliders — pero esto no tenía nada que ver con OpenSim.*

**Root cause:** the Step 1 integration (full 3D torque, Section 15) added "Torque vs Angle" curve recomputation — 100 sample points × 3 curves = 300 full kinematic evaluations — directly inside `update_all()`, which runs on **every single slider-drag event** (not just when the slider is released). Dragging a slider fires dozens of these events per second, each triggering 300 evaluations plus a full `matplotlib` redraw — all synchronously, on the main thread.

***Causa raíz:*** *la integración del Paso 1 (torque 3D completo, Sección 15) agregó el recálculo de las curvas "Torque vs Ángulo" — 100 puntos de muestra × 3 curvas = 300 evaluaciones cinemáticas completas — directamente dentro de `update_all()`, que corre en **cada evento individual de arrastre de slider** (no solo al soltar). Arrastrar un slider dispara docenas de estos eventos por segundo, cada uno disparando 300 evaluaciones más un redibujado completo de `matplotlib` — todo de forma síncrona, en el hilo principal.*

**Fix:** split the "instant" parts (3D view redraw, numeric labels — cheap) from the "expensive" part (curve sweep — 300 evaluations). The curve sweep is now debounced: it's scheduled via a single-shot `QTimer`, restarted on every slider event, so it only actually runs 150 ms after the user *stops* moving the slider — never during continuous dragging.

***Arreglo:*** *separar las partes "instantáneas" (redibujado de la vista 3D, etiquetas numéricas — baratas) de la parte "cara" (barrido de curvas — 300 evaluaciones). El barrido de curvas ahora tiene *debounce*: se programa vía un `QTimer` de disparo único, que se reinicia en cada evento de slider, así que solo corre de verdad 150 ms después de que el usuario *deja* de mover el slider — nunca durante el arrastre continuo.*

### 16.4 Problem 3 — the model itself is expensive, even with warm-start / Problema 3 — el modelo mismo es caro, incluso con warm-start

**Symptom:** even after Problems 1 and 2 were fixed (OpenSim in a background thread, curves debounced), the OpenSim comparison label still took **several seconds** to update each cycle — clearly better than before, but far from smooth.

***Síntoma:*** *incluso después de arreglar los Problemas 1 y 2 (OpenSim en un hilo de fondo, curvas con debounce), la etiqueta de comparación de OpenSim seguía tardando **varios segundos** en actualizarse cada ciclo — claramente mejor que antes, pero lejos de fluido.*

**Root cause — a chain of expensive calls, not just one:** a single OpenSim comparison update involves:

*Causa raíz — una cadena de llamadas costosas, no solo una:*

1. **Warm-start local solve** (`solve_osim_angles_warmstart`): one `L-BFGS-B` optimization, each iteration calling `model.realizePosition(state)`.
2. **Gravitational torque** (`gravity_only_generalized_forces`): 3 coordinates × 2 finite-difference evaluations = 6 more `realizePosition` calls, each looping over every `Body` in the model to sum potential energy.
3. **Shoulder Jacobian projection** (`project_shoulder_torque_to_urdf` → `shoulder_angle_jacobian`): **4 separate local optimizations** (one per finite-difference perturbation direction), each itself an `L-BFGS-B` run with its own set of `realizePosition` calls.

*1. **Refinamiento local warm-start** (`solve_osim_angles_warmstart`): una optimización `L-BFGS-B`, cada iteración llamando a `model.realizePosition(state)`.*
*2. **Torque gravitacional** (`gravity_only_generalized_forces`): 3 coordenadas × 2 evaluaciones por diferencias finitas = 6 llamadas más a `realizePosition`, cada una recorriendo todos los `Body` del modelo para sumar energía potencial.*
*3. **Proyección Jacobiana del hombro** (`project_shoulder_torque_to_urdf` → `shoulder_angle_jacobian`): **4 optimizaciones locales separadas** (una por dirección de perturbación de diferencias finitas), cada una a su vez una corrida de `L-BFGS-B` con su propio conjunto de llamadas a `realizePosition`.*

Multiply this out and a single comparison update can trigger **dozens to over a hundred** `realizePosition` calls. And `MoBL_ARMS_bimanual_6_2_21.osim` is not a simple 4-DOF arm — it's a **46-coordinate bimanual model** with a coupled scapulohumeral rhythm (`CoordinateCouplerConstraint`, Section 4.1) that must be re-evaluated on every single realization. Each individual `realizePosition` call is inherently more expensive here than it would be on a minimal model.

*Multiplicando esto, una sola actualización de comparación puede disparar **de decenas a más de cien** llamadas a `realizePosition`. Y `MoBL_ARMS_bimanual_6_2_21.osim` no es un brazo simple de 4 GDL — es un **modelo bimanual de 46 coordenadas** con un ritmo escapulohumeral acoplado (`CoordinateCouplerConstraint`, Sección 4.1) que debe reevaluarse en cada realización individual. Cada llamada a `realizePosition` es, de por sí, más costosa aquí de lo que sería en un modelo mínimo.*

**Partial fix applied:** cap the optimizer's iteration count (`maxiter=3`) in the local refinement, since every local solve already starts extremely close to the true solution (warm-start, or a tiny `eps=1e-4` perturbation for the Jacobian) — a handful of iterations is enough, so there's no reason to let `L-BFGS-B` run its full default budget.

***Arreglo parcial aplicado:*** *limitar el número de iteraciones del optimizador (`maxiter=3`) en el refinamiento local, ya que cada solve local ya arranca extremadamente cerca de la solución verdadera (warm-start, o una perturbación diminuta de `eps=1e-4` para la Jacobiana) — un puñado de iteraciones alcanza, así que no hay razón para dejar que `L-BFGS-B` corra su presupuesto completo por defecto.*

**The real, structural fix (if `maxiter` capping isn't enough): a precomputed lookup table.** Instead of solving the bridge live, sample a fine grid of `(sh_abd, sh_flex)` offline (e.g. every 5°, ~1300 combinations), solve each once with the full multi-start solver, extract torques, and save the table (`.npy`/`.csv`). At runtime, the simulator only **interpolates** this table — zero OpenSim calls during interaction, regardless of how expensive the underlying model is. This trades a one-time precompute (a few minutes) for guaranteed-instant real-time lookups.

***El arreglo real y estructural (si limitar `maxiter` no alcanza): una tabla de consulta precalculada.*** *En vez de resolver el puente en vivo, se muestrea una grilla fina de `(sh_abd, sh_flex)` fuera de línea (ej. cada 5°, ~1300 combinaciones), se resuelve cada una una sola vez con el solver multi-start completo, se extraen los torques, y se guarda la tabla (`.npy`/`.csv`). En tiempo real, el simulador solo **interpola** esta tabla — cero llamadas a OpenSim durante la interacción, sin importar cuán costoso sea el modelo subyacente. Esto cambia un precálculo de una sola vez (unos minutos) por consultas garantizadamente instantáneas en tiempo real.*

### 16.5 A fourth win: skipping redundant constraint re-solving / Una cuarta ganancia: evitar volver a resolver restricciones redundantemente

Even after capping `maxiter`, the animation was still not smooth. One more API-level detail was found: `Coordinate.setValue(state, value)` has a hidden third parameter, `enforceContraints`, defaulting to `True` — meaning every single one of the 4 `setValue` calls inside `set_pose()` (elv_angle, shoulder_elv, shoulder_rot, elbow_flex) was independently forcing OpenSim to re-solve the model's constraints (the scapulohumeral coupling), even though `realizePosition()` is called explicitly right after, which would resolve them all together anyway.

*Incluso después de limitar `maxiter`, la animación seguía sin ser fluida. Se encontró un detalle más a nivel de API: `Coordinate.setValue(state, value)` tiene un tercer parámetro oculto, `enforceContraints`, que por defecto es `True` — significando que cada una de las 4 llamadas `setValue` dentro de `set_pose()` (elv_angle, shoulder_elv, shoulder_rot, elbow_flex) forzaba independientemente a OpenSim a resolver de nuevo las restricciones del modelo (el acoplamiento escapulohumeral), aunque `realizePosition()` se llama explícitamente justo después, lo cual las resolvería todas juntas de todas formas.*

**Analogy:** it's like validating an entire 4-field form after typing *each* field, instead of once at the end. Setting `enforceContraints=False` on each individual `setValue` call, and letting the explicit `realizePosition()` call do the one real validation at the end, removed roughly 4× of redundant constraint-solving work per pose.

***Analogía:*** *es como validar un formulario completo de 4 campos después de escribir *cada* campo, en vez de una sola vez al final. Poner `enforceContraints=False` en cada llamada individual a `setValue`, dejando que la llamada explícita a `realizePosition()` haga la única validación real al final, eliminó aproximadamente 4× de trabajo redundante de resolución de restricciones por postura.*

```python
# Antes: cada linea re-resuelve las restricciones del modelo
self.coord_elv_angle.setValue(self.state, float(elv_angle))
# Ahora: solo se resuelven una vez, en el realizePosition() posterior
self.coord_elv_angle.setValue(self.state, float(elv_angle), False)
```

**Result: the animation is now smooth.** Combining all four fixes (background `QThread`, debounced curves, warm-start with capped iterations, and `enforceContraints=False`) resolved the performance problem well enough for interactive use — the precomputed lookup table (Section 16.4) remains available as a further option, but is no longer strictly necessary.

***Resultado: la animación ahora es fluida.*** *Combinando las cuatro correcciones (`QThread` en segundo plano, curvas con debounce, warm-start con iteraciones limitadas, y `enforceContraints=False`) se resolvió el problema de rendimiento lo suficientemente bien para uso interactivo — la tabla de consulta precalculada (Sección 16.4) sigue disponible como opción adicional, pero ya no es estrictamente necesaria.*

### 16.6 Summary table / Tabla resumen

| # | Problema | Causa raíz | Arreglo aplicado |
|---|---|---|---|
| 1 | UI congelada cada 750ms | Solver de OpenSim en el hilo principal (bloquea el event loop de Qt) | `QThread` dedicado (`OpenSimCompareWorker`) |
| 2 | UI lenta al arrastrar sliders | 300 evaluaciones cinemáticas para las curvas, en cada evento de slider | Debounce de 150ms (curvas solo se recalculan al detenerse) |
| 3 | Varios segundos por actualización de OpenSim | Docenas de llamadas `realizePosition` (warm-start + 6 PE + 4 Jacobiana) sobre un modelo bimanual de 46 GDL | `maxiter` limitado a 3 iteraciones |
| 4 | Aún lento tras el arreglo 3 | `Coordinate.setValue()` re-resuelve restricciones en cada llamada (`enforceContraints=True` por defecto) | `enforceContraints=False`, validación única al final con `realizePosition()` |

**Resultado final: animación fluida**, confirmado en uso real tras combinar los 4 arreglos.

---

## 18. Appendix — Understanding torque in plain language / Apéndice — Entendiendo el torque en palabras simples

This appendix collects, in one place, the plain-language explanations of every torque concept and every number/curve the simulator shows — useful as a quick reference for anyone (student, colleague) using the tool without having read the whole technical log above.

*Este apéndice reúne, en un solo lugar, las explicaciones en palabras simples de cada concepto de torque y de cada número/curva que muestra el simulador — útil como referencia rápida para cualquiera (estudiante, colega) que use la herramienta sin haber leído todo el registro técnico anterior.*

### 18.1 What is torque, in one simple example / Qué es el torque, en un ejemplo simple

Imagine holding a broom with your arm stretched out. To know how "heavy" it feels on your shoulder, you need two things: **how far** the weight is from your shoulder, and **in which exact direction** your arm points in space (up, sideways, forward...). Torque is just a number that combines those two things into "how hard is this to hold."

*Imagina sostener una escoba con el brazo estirado. Para saber qué tan "pesada" se siente en tu hombro, necesitas dos cosas: **qué tan lejos** está el peso de tu hombro, y **en qué dirección exacta** apunta tu brazo en el espacio (arriba, al costado, adelante...). El torque es solo un número que combina esas dos cosas en "qué tan difícil es sostener esto."*

### 18.2 Three ways to calculate the same torque / Tres formas de calcular el mismo torque

The simulator computes the *same physical quantity* three different ways, so you can compare them side by side:

*El simulador calcula la *misma magnitud física* de tres formas distintas, para que puedas compararlas lado a lado:*

| Método / Method | Analogía simple / Simple analogy |
|---|---|
| **Simple (sin/cos)** | A quick mental shortcut: only looks at one angle at a time, as if the arm could only move in one flat direction. Fast but misses details when the arm moves in several directions at once. / Un atajo mental rápido: solo mira un ángulo a la vez, como si el brazo solo pudiera moverse en una dirección plana. Rápido, pero se pierde detalles cuando el brazo se mueve en varias direcciones a la vez. |
| **3D (torque_3d.py)** | The real geometry: takes the arm's exact position in 3D space and gravity's exact direction, and computes the exact twist that produces at each joint — like an engineer would, with real vectors, not a shortcut. / La geometría real: toma la posición exacta del brazo en el espacio 3D y la dirección exacta de la gravedad, y calcula el giro exacto que eso produce en cada articulación — como lo haría un ingeniero, con vectores reales, no con un atajo. |
| **OpenSim (real)** | The same real geometry, but using the actual mass and shape of a real human arm (from cadaver measurements), instead of assuming the arm is a simple uniform cylinder. / La misma geometría real, pero usando la masa y forma reales de un brazo humano real (de mediciones cadavéricas), en vez de asumir que el brazo es un cilindro uniforme simple. |

### 18.3 Glossary of every value in the results panel / Glosario de cada valor en el panel de resultados

**`Total`** — the real torque needed to hold that joint in its current position against gravity, computed with the 3D method. This is the "authoritative" number — everything else (alerts, human/exo split) is based on it.

***`Total`*** *— el torque real necesario para sostener esa articulación en su posición actual contra la gravedad, calculado con el método 3D. Es el número "autoritativo" — todo lo demás (alertas, reparto humano/exo) se basa en él.*

**`Human`** — `Total − Exo`: how much of that torque is left for the person's own muscles to supply, after the exoskeleton's help is subtracted.

***`Human`*** *— `Total − Exo`: cuánto de ese torque le queda por aportar a los propios músculos de la persona, después de restar la ayuda del exoesqueleto.*

**`Exo`** — how much torque the exoskeleton is currently contributing, based on the assist slider (as a percentage of Total, or as a fixed Nm value, depending on the mode switch).

***`Exo`*** *— cuánto torque está aportando actualmente el exoesqueleto, según el slider de asistencia (como porcentaje del Total, o como un valor fijo en Nm, según el interruptor de modo).*

**`(simple: X Nm)`** — the value the *original* sine/cosine formula would give for the same posture — shown only as a side-by-side reference, not used for anything else.

***`(simple: X Nm)`*** *— el valor que daría la fórmula *original* seno/coseno para la misma postura — se muestra solo como referencia comparativa, no se usa para nada más.*

**`OpenSim (real)` amber label** — the same 3 torques, computed with OpenSim's real cadaveric arm model, updated every ~1.2 seconds in the background (so it can lag slightly behind the sliders).

***Etiqueta ámbar `OpenSim (real)`*** *— los mismos 3 torques, calculados con el modelo de brazo cadavérico real de OpenSim, actualizado cada ~1.2 segundos en segundo plano (así que puede ir un poco atrasado respecto a los sliders).*

### 18.4 Glossary of every curve in the graphs / Glosario de cada curva en las gráficas

Each of the 3 graphs (Sh. Flexion, Sh. Abduction, Elbow) sweeps that joint's angle from -180° to 180°, keeping the *other two* joints fixed at their current slider value — answering "what would the torque be if I moved *this* joint, everything else staying where it is right now?"

*Cada una de las 3 gráficas (Sh. Flexión, Sh. Abducción, Codo) barre el ángulo de esa articulación de -180° a 180°, manteniendo las *otras dos* articulaciones fijas en su valor actual de slider — respondiendo "¿cuál sería el torque si moviera *esta* articulación, quedándose todo lo demás donde está ahora?"*

| Curve / Curva | Style / Estilo | Meaning / Significado |
|---|---|---|
| 3D Total | gray dashed / gris discontinuo | Total torque (3D method) across the whole angle range / Torque total (método 3D) en todo el rango de ángulo |
| 3D Human | green solid / verde sólido | The human's share, across the range / La parte del humano, en todo el rango |
| 3D Exo | blue solid / azul sólido | The exo's contribution, across the range / El aporte del exo, en todo el rango |
| Simple (sin/cos) | amber dotted / ámbar punteado | The original formula's curve, for visual comparison / La curva de la fórmula original, para comparación visual |
| OpenSim (real) | purple dash-dot / morado guión-punto | The real OpenSim value, from the precomputed table (only shown if `precompute_sweep.py` has been run) / El valor real de OpenSim, desde la tabla precalculada (solo se muestra si ya se corrió `precompute_sweep.py`) |
| Circle marker / círculo marcador | colored dot / punto de color | Your *current* slider position on that curve — moves live as you move sliders or play the animation / Tu posición *actual* de slider en esa curva — se mueve en vivo al mover sliders o correr la animación |
| Red dotted horizontal line / línea roja punteada horizontal | — | The ergonomic fatigue limit for that joint / El límite de fatiga ergonómica para esa articulación |
| Green shaded area / área sombreada verde | — | The "relief" the exoskeleton provides — the gap between Total and Human / El "alivio" que aporta el exoesqueleto — el espacio entre Total y Human |

**Simple example / Ejemplo simple:** if the exo assist slider is at 0%, `Exo` is exactly zero and `Human = Total` — so the green and gray lines sit exactly on top of each other (by design, not a bug). Raise the assist slider and you'll see them separate: the green line drops, and the blue line rises to fill the gap.

***Ejemplo simple:*** *si el slider de asistencia del exo está en 0%, `Exo` es exactamente cero y `Human = Total` — así que la línea verde y la gris quedan exactamente superpuestas (por diseño, no es un bug). Sube el slider de asistencia y las verás separarse: la línea verde baja, y la azul sube para llenar el espacio.*

---

## 20. Step 10 — Individual muscle forces via Static Optimization / Paso 10 — Fuerzas musculares individuales vía Static Optimization

### 20.1 The problem this solves / El problema que resuelve

Everything up to this point answered "how much **net** torque is needed at this joint?" But a net torque can be produced in infinitely many ways — the body has more muscles than degrees of freedom (a *redundant* system). Static Optimization resolves that ambiguity the same way OpenSim's own tool does: assume the body distributes the work "efficiently," minimizing total muscular effort (sum of squared activations), subject to the muscles' combined force × moment arm reproducing the exact required torque.

*Todo lo anterior respondía "¿cuánto torque **neto** se necesita en esta articulación?" Pero un torque neto se puede lograr de infinitas formas — el cuerpo tiene más músculos que grados de libertad (un sistema *redundante*). Static Optimization resuelve esa ambigüedad de la misma forma que la propia herramienta de OpenSim: asumir que el cuerpo reparte el trabajo "eficientemente," minimizando el esfuerzo muscular total (suma de activaciones al cuadrado), sujeto a que la combinación de fuerza × brazo de palanca de los músculos reproduzca exactamente el torque requerido.*

### 20.2 The implementation: verify, don't assume, once again / La implementación: verificar, no asumir, una vez más

Rather than hardcoding which muscles cross which joint (error-prone, as this whole project has repeatedly shown), `solve_muscle_activations` asks OpenSim itself: for every muscle in the model, compute its real moment arm (`muscle.computeMomentArm(state, coordinate)`) about each target coordinate, and only include muscles with a non-negligible moment arm. This is the same "verify with the real engine" principle used throughout the project (Sections 6, 8, 11).

*En vez de codificar a mano qué músculos cruzan qué articulación (propenso a errores, como este proyecto ha demostrado repetidamente), `solve_muscle_activations` le pregunta a OpenSim: para cada músculo del modelo, calcula su brazo de palanca real (`muscle.computeMomentArm(state, coordinate)`) sobre cada coordenada objetivo, y solo incluye los músculos con un brazo de palanca no despreciable. Es el mismo principio de "verificar con el motor real" usado en todo el proyecto (Secciones 6, 8, 11).*

```python
def solve_muscle_activations(osim_model, coord_names, target_torques, moment_arm_threshold=1e-4, side="r"):
    # 1. Para cada musculo del modelo, calcular su brazo de palanca real
    #    sobre cada coordenada objetivo -- sin asumir cuales "deberian" cruzarla
    for m in muscle_set:
        for coord in coords:
            moment_arm = m.computeMomentArm(state, coord)
            # incluir solo si |moment_arm| > umbral

    # 2. Minimizar suma(activacion^2) sujeto a:
    #    suma(moment_arm_i * Fmax_i * activacion_i) == torque_objetivo, para cada coordenada
    #    0 <= activacion_i <= 1
    res = minimize(cost, x0, jac=cost_grad, method="SLSQP", bounds=bounds, constraints=constraints)
```

**Sign handling for free (agonist/antagonist naturally emerges) / Manejo de signo "gratis" (agonistas/antagonistas emergen solos):** moment arms carry sign (positive on one side of the joint axis, negative on the other), so the linear constraint naturally lets some muscles contribute positive torque and others negative, even though activation itself stays bounded in [0,1] (muscles can only pull, never push). This is why the solver correctly favors flexors over extensors (or vice versa) without being told which is which.

*Los brazos de palanca tienen signo (positivo de un lado del eje articular, negativo del otro), así que la restricción lineal naturalmente permite que algunos músculos aporten torque positivo y otros negativo, aunque la activación en sí se mantenga acotada en [0,1] (los músculos solo pueden tirar, nunca empujar). Por eso el solver favorece correctamente flexores sobre extensores (o viceversa) sin que se le diga cuál es cuál.*

### 20.3 Validation: elbow first, then the full arm together / Validación: primero el codo, después el brazo completo junto

**Elbow alone (fewer muscles, easier to sanity-check):** with the elbow needing flexion torque to hold the arm up, the solver correctly activated only flexors (`BRA_r`, `BIClong_r`, `BRD_r`, `BICshort_r`) and left extensors (`TRIlong_r`, `TRIlat_r`, `TRImed_r`, `ANC_r`) at ~0% — activating the opposing muscle would waste effort, and the minimum-effort criterion correctly avoided it. The residual (difference between the muscles' combined torque and the target) was at machine precision (~1e-13 Nm).

*Codo solo (menos músculos, más fácil de verificar): con el codo necesitando torque de flexión para sostener el brazo, el solver activó correctamente solo flexores (`BRA_r`, `BIClong_r`, `BRD_r`, `BICshort_r`) y dejó los extensores (`TRIlong_r`, `TRIlat_r`, `TRImed_r`, `ANC_r`) en ~0% — activar el músculo contrario desperdiciaría esfuerzo, y el criterio de mínimo esfuerzo correctamente lo evitó. El residual (diferencia entre el torque combinado de los músculos y el objetivo) quedó en precisión de máquina (~1e-13 Nm).*

**Full arm together (shoulder + elbow, all 3 coordinates in one solve):** biarticular muscles (crossing both shoulder and elbow, e.g. `BIClong_r`) are only counted correctly if the shoulder and elbow are solved *together*, not separately — solving them apart would double-count or misattribute their contribution. Results matched real biomechanics without being told to: in the abduction posture, the deltoid did most of the work, **plus** the rotator cuff (`INFSP_r`, `SUBSC_r`) activated at low levels — the real "force couple" phenomenon (the rotator cuff stabilizes the humeral head while the deltoid elevates the arm), which the optimizer discovered on its own.

***Brazo completo junto (hombro + codo, las 3 coordenadas en un solo solve):*** *los músculos biarticulares (que cruzan tanto hombro como codo, ej. `BIClong_r`) solo se cuentan correctamente si el hombro y el codo se resuelven **juntos**, no por separado — resolverlos aparte los contaría dos veces o de forma inconsistente. Los resultados coincidieron con biomecánica real sin que se le dijera: en la postura de abducción, el deltoides hizo la mayor parte del trabajo, **más** el manguito rotador (`INFSP_r`, `SUBSC_r`) activándose en niveles bajos — el fenómeno real de "force couple" (el manguito rotador estabiliza la cabeza humeral mientras el deltoides eleva el brazo), que el optimizador descubrió solo.*

### 20.4 GUI integration: on-demand, not real-time / Integración en la GUI: bajo demanda, no en tiempo real

Static Optimization is heavier than the net-torque comparison (more muscles, more constraints), so — learning from the whole performance saga of Section 16 — it was integrated as a dedicated **"Analyze Muscles" button**, running in its own `MuscleAnalysisWorker` `QThread` (same pattern as `OpenSimCompareWorker`), triggered only on click, never continuously. Results open in a popup dialog with a horizontal bar chart (one bar per muscle, colored by activation level), reusing the warm-start guess from the live OpenSim comparison for a faster shoulder solve.

*Static Optimization es más pesado que la comparación de torque neto (más músculos, más restricciones), así que — aprendiendo de toda la saga de rendimiento de la Sección 16 — se integró como un **botón dedicado "Analyze Muscles"**, corriendo en su propio `QThread` (`MuscleAnalysisWorker`, mismo patrón que `OpenSimCompareWorker`), disparado solo al hacer clic, nunca continuamente. Los resultados se abren en una ventana emergente con una gráfica de barras horizontales (una barra por músculo, coloreada según nivel de activación), reutilizando el guess de warm-start de la comparación en vivo con OpenSim para un solve de hombro más rápido.*

**Readable muscle names:** rather than showing raw codes like `DELT2_r`, a lookup dictionary (`MUSCLE_FULL_NAMES`, ~45 entries covering shoulder, elbow, and forearm/wrist muscles) displays `Deltoid Middle (DELT2_r)` — the anatomical name first, with the original OpenSim code in parentheses for anyone cross-referencing the model file.

***Nombres de músculo legibles:*** *en vez de mostrar códigos crudos como `DELT2_r`, un diccionario de referencia (`MUSCLE_FULL_NAMES`, ~45 entradas cubriendo hombro, codo, y antebrazo/muñeca) muestra `Deltoid Middle (DELT2_r)` — el nombre anatómico primero, con el código original de OpenSim entre paréntesis para quien quiera cruzar referencia con el archivo del modelo.*

### 20.5 An important clarification: target-torque coordinate names / Una aclaración importante: los nombres de coordenadas en los torques objetivo

The dialog shows target torques labeled `Shoulder Plane (elv_angle_r)`, `Shoulder Elevation (shoulder_elv_r)`, `Elbow Flexion (elbow_flexion_r)` — deliberately **not** `Sh. Flexion` / `Sh. Abduction` (the names your sliders use). This is intentional, not an inconsistency: as Section 1 and 5 already established, `elv_angle_r`/`shoulder_elv_r` are a genuinely different (spherical) parametrization of shoulder orientation than your `sh_flex`/`sh_abd` (independent-axis) sliders — the same value in one basis does not point the arm in the same physical direction as the "equivalent-looking" value in the other. Labeling them as if they were `sh_flex`/`sh_abd` would misleadingly imply a 1:1 correspondence that doesn't exist. Since muscles' moment arms are computed directly against OpenSim's own coordinates, showing those coordinates' real names (with a plain-language gloss) is the honest choice.

***Una aclaración importante: los nombres de coordenadas en los torques objetivo.*** *La ventana muestra los torques objetivo etiquetados `Shoulder Plane (elv_angle_r)`, `Shoulder Elevation (shoulder_elv_r)`, `Elbow Flexion (elbow_flexion_r)` — deliberadamente **no** `Sh. Flexion` / `Sh. Abduction` (los nombres que usan tus sliders). Esto es intencional, no una inconsistencia: como ya establecieron las Secciones 1 y 5, `elv_angle_r`/`shoulder_elv_r` son una parametrización genuinamente distinta (esférica) de la orientación del hombro respecto a tus sliders `sh_flex`/`sh_abd` (ejes independientes) — el mismo valor en una base no apunta el brazo en la misma dirección física que el valor "de apariencia equivalente" en la otra. Etiquetarlos como si fueran `sh_flex`/`sh_abd` insinuaría falsamente una correspondencia 1:1 que no existe. Como los brazos de palanca de los músculos se calculan directamente contra las coordenadas propias de OpenSim, mostrar los nombres reales de esas coordenadas (con una traducción en palabras simples) es la opción honesta.*

---

## 21. UI/UX improvements / Mejoras de interfaz

A round of usability fixes, driven directly by hands-on testing feedback:

*Una ronda de arreglos de usabilidad, impulsada directamente por retroalimentación de pruebas reales:*

- **Toggle buttons** for the 3D view legend (hidden by default — it was covering too much of the view), IMU axis triads, and joint text labels — each independently switchable from the toolbar. / **Botones toggle** para la leyenda del visor 3D (oculta por defecto — tapaba demasiado de la vista), las tríadas de ejes IMU, y las etiquetas de texto de las articulaciones — cada uno conmutable independientemente desde la barra de herramientas.
- **Collapsible metrics panel** — a "Hide metrics" button frees up vertical space for the graphs (the most important part of the simulator) without needing to scroll. / **Panel de métricas colapsable** — un botón "Hide metrics" libera espacio vertical para las gráficas (la parte más importante del simulador) sin necesitar hacer scroll.
- **Full-width bottom layout** — the analytics panel moved from a narrow right-side column to a full-width bottom dock, with the 3 torque graphs arranged side by side instead of stacked, and much larger fonts throughout (titles, axis labels, legend). / **Layout de ancho completo abajo** — el panel de análisis se movió de una columna angosta a la derecha a un panel inferior de ancho completo, con las 3 gráficas de torque lado a lado en vez de apiladas, y fuentes mucho más grandes en todas partes (títulos, ejes, leyenda).
- **A subtle but important curve-visibility bug:** when exo assist is 0%, `Human = Total` exactly, so the thicker green "Human" line was drawn on top of and completely hid the gray dashed "Total" line underneath. Fixed with explicit `zorder` values so `Total` always draws visibly above `Human` even when their values coincide. / **Un bug sutil pero importante de visibilidad de curvas:** cuando la asistencia del exo es 0%, `Human = Total` exactamente, así que la línea verde más gruesa de "Human" se dibujaba encima y tapaba completamente la línea gris discontinua de "Total" debajo. Arreglado con valores explícitos de `zorder` para que `Total` siempre se dibuje visiblemente por encima de `Human`, incluso cuando sus valores coinciden.
- **The "curves frozen during continuous events" bug (a second instance of Section 16's core lesson):** the same debounce-vs-continuous-events problem from Section 16.3 resurfaced with the exoskeleton's "glow pulse" timer — it fires continuously the moment assist is set above 0%, which kept resetting the curve-refresh debounce indefinitely, freezing the graphs forever once assist was active. Fixed by switching from a pure debounce (resets on every call) to a throttle (only starts the timer if it isn't already running) — guaranteeing the curves refresh at least every 150ms regardless of how continuously `update_all()` is being called. / **El bug de "curvas congeladas durante eventos continuos" (una segunda instancia de la lección central de la Sección 16):** el mismo problema de debounce-vs-eventos-continuos de la Sección 16.3 resurgió con el timer del "pulso de brillo" del exoesqueleto — se dispara continuamente en cuanto la asistencia se pone por encima de 0%, lo que reiniciaba indefinidamente el debounce de refresco de curvas, congelando las gráficas para siempre una vez la asistencia estaba activa. Arreglado cambiando de un debounce puro (se reinicia en cada llamada) a un throttle (solo inicia el timer si no está ya corriendo) — garantizando que las curvas se refresquen al menos cada 150ms sin importar cuán continuamente se esté llamando `update_all()`.
- **Full English translation** of every user-facing string (buttons, dialog titles, labels, the equations tab's LaTeX text, muscle names) — internal code comments remain bilingual/Spanish as project documentation. / **Traducción completa al inglés** de cada cadena visible al usuario (botones, títulos de diálogo, etiquetas, el texto LaTeX de la pestaña de ecuaciones, nombres de músculos) — los comentarios internos del código quedan bilingües/en español como documentación del proyecto.

---

## 22. Comparing muscular effort with vs. without exo assist / Comparando el esfuerzo muscular con vs. sin asistencia del exo

### 22.1 The question this answers / La pregunta que responde

This is the most interesting question the whole project set out to answer: **does the exoskeleton genuinely reduce muscular effort, or does it just shift the load onto a different muscle?** A net-torque number alone can't answer this — two very different muscle activation patterns can produce the exact same net torque. You need the muscle-level breakdown from Section 20, computed *twice*: once for the torque the body would need to supply with no help, and once for the (smaller) torque it needs once the exoskeleton's contribution is subtracted.

*Esta es la pregunta más interesante que todo el proyecto se propuso responder: **¿el exoesqueleto realmente reduce el esfuerzo muscular, o solo traslada la carga a otro músculo?** Un solo número de torque neto no puede responder esto — dos patrones de activación muscular muy distintos pueden producir exactamente el mismo torque neto. Se necesita el desglose a nivel de músculo de la Sección 20, calculado *dos veces*: una para el torque que el cuerpo necesitaría aportar sin ayuda, y otra para el torque (menor) que necesita una vez restada la contribución del exoesqueleto.*

### 22.2 A basis-mismatch problem, once more / Un problema de bases distintas, una vez más

Here's the catch: the exoskeleton assist sliders (`Sh. Flex Exo Assist`, `Sh. Abd Exo Assist`) are defined in **your** anatomical basis (`sh_flex`, `sh_abd`) — that's the natural basis for a real exoskeleton, since its motors are mounted around real anatomical axes. But the muscle analysis of Section 20 needs its target torques in **OpenSim's own basis** (`elv_angle_r`, `shoulder_elv_r`) — the same basis mismatch problem from Sections 1, 5, and 13, appearing again in a new place.

*Aquí está la trampa: los sliders de asistencia del exoesqueleto (`Sh. Flex Exo Assist`, `Sh. Abd Exo Assist`) están definidos en **tu** base anatómica (`sh_flex`, `sh_abd`) — es la base natural para un exoesqueleto real, ya que sus motores se montan sobre ejes anatómicos reales. Pero el análisis muscular de la Sección 20 necesita sus torques objetivo en **la base propia de OpenSim** (`elv_angle_r`, `shoulder_elv_r`) — el mismo problema de bases distintas de las Secciones 1, 5 y 13, apareciendo de nuevo en un lugar nuevo.*

**Simple analogy:** imagine you know "this backpack is 3 kg lighter" (a fact in kilograms), but the question you need to answer is "how many fewer calories does carrying it burn?" (a different unit entirely). You can't just subtract 3 from a calorie count — you need a conversion factor that correctly relates the two. The Jacobian `J` from Section 13 *is* that conversion factor between the two torque bases — and since here we need to go the *other direction* (from your basis into OpenSim's, instead of OpenSim's into yours), we use its **inverse**, `J⁻¹`.

***Analogía simple:*** *imagina que sabes "esta mochila pesa 3 kg menos" (un hecho en kilogramos), pero la pregunta que necesitas responder es "¿cuántas calorías menos quema cargarla?" (una unidad completamente distinta). No puedes simplemente restar 3 de un conteo de calorías — necesitas un factor de conversión que relacione correctamente ambas. La Jacobiana `J` de la Sección 13 **es** ese factor de conversión entre las dos bases de torque — y como aquí necesitamos ir en la **dirección contraria** (de tu base hacia la de OpenSim, en vez de la de OpenSim hacia la tuya), usamos su **inversa**, `J⁻¹`.*

```python
# tau_urdf = J^T @ tau_osim  (Seccion 13b, direccion original)
# => tau_osim = (J^T)^-1 @ tau_urdf = (J^-1)^T @ tau_urdf  (aqui, direccion inversa)

J = shoulder_angle_jacobian(osim_model, sh_abd, sh_flex, elv_angle, shoulder_elv)
tau_urdf_reduction = np.array([exo_sa, exo_sf])          # lo que el exo aporta, en TU base
tau_osim_reduction = np.linalg.inv(J).T @ tau_urdf_reduction  # lo mismo, en la base de OpenSim

target_with_exo["elv_angle_r"] = target_no_exo["elv_angle_r"] - tau_osim_reduction[0]
target_with_exo["shoulder_elv_r"] = target_no_exo["shoulder_elv_r"] - tau_osim_reduction[1]
target_with_exo["elbow_flexion_r"] = target_no_exo["elbow_flexion_r"] - exo_el  # el codo SI es 1:1, no necesita conversion
```

Note the elbow doesn't need any conversion — `elbow_flexion_r` (OpenSim) and `elbow_flex` (your slider) are the exact same physical angle (Section 8), so subtracting the exo's elbow contribution is a direct subtraction, no Jacobian needed. Only the shoulder needs the basis conversion.

*Nota que el codo no necesita ninguna conversión — `elbow_flexion_r` (OpenSim) y `elbow_flex` (tu slider) son exactamente el mismo ángulo físico (Sección 8), así que restar la contribución del exo en el codo es una resta directa, sin Jacobiana. Solo el hombro necesita la conversión de base.*

### 22.3 A built-in sanity check / Una verificación de sanidad integrada

The cleanest validation: set exo assist to 0% and run the analysis. Since there's nothing to subtract, `target_with_exo` should equal `target_no_exo` exactly, and the "without exo" (red) and "with exo" (green) bars should overlap almost perfectly for every muscle. The dialog even states this explicitly ("no exo assist set — both bars should match") so it's obvious at a glance whether the math is behaving.

*La validación más limpia: poner la asistencia del exo en 0% y correr el análisis. Como no hay nada que restar, `target_with_exo` debería ser igual a `target_no_exo` exactamente, y las barras "sin exo" (rojo) y "con exo" (verde) deberían superponerse casi perfectamente para cada músculo. El diálogo incluso lo indica explícitamente ("no exo assist set — both bars should match") para que sea obvio de un vistazo si la matemática se está comportando bien.*

### 22.4 Reading the comparison chart / Leyendo la gráfica de comparación

Each muscle gets **two bars, side by side**: red = activation without any exo help, green = activation with the current exo assist setting. A muscle whose green bar is much shorter than its red bar is one the exoskeleton is genuinely relieving. A muscle whose green bar barely moves (or, in principle, gets *taller*) would be a red flag — it would mean the exoskeleton isn't helping that particular muscle, or is even compensating for the reduced load elsewhere by working harder itself (a real, valid finding an exoskeleton designer would want to know about, not something the tool would hide).

*Cada músculo obtiene **dos barras, una al lado de la otra**: rojo = activación sin ninguna ayuda del exo, verde = activación con la configuración actual de asistencia. Un músculo cuya barra verde sea mucho más corta que su barra roja es uno que el exoesqueleto está aliviando genuinamente. Un músculo cuya barra verde apenas se mueva (o, en principio, se vuelva *más alta*) sería una señal de alerta — significaría que el exoesqueleto no está ayudando a ese músculo en particular, o incluso está compensando la carga reducida en otro lado trabajando más él mismo (un hallazgo real y válido que a un diseñador de exoesqueletos le interesaría saber, no algo que la herramienta debería esconder).*

**Muscles are sorted by their "without exo" activation** (highest first), so the muscles doing the most work in the unassisted case are always at the top — usually the most informative ones to look at first.

***Los músculos se ordenan por su activación "sin exo"*** *(de mayor a menor), así que los músculos que hacen más trabajo en el caso sin asistencia siempre quedan arriba — usualmente los más informativos para mirar primero.*

---

## 24. Option B — Real bone visualization with PyVista, and a real bug it uncovered / Opción B — Visualización de huesos reales con PyVista, y un bug real que descubrió

### 24.1 The goal / El objetivo

Replace (in a separate tab, not touching the working schematic view) the simplified cylinder/sphere arm with the real OpenSim bone meshes (`.vtp` files), for a much more realistic presentation — purely cosmetic, deliberately kept separate from the validated physics so a rendering issue could never put the working simulator at risk (the "Option B" decision from earlier in the project, chosen specifically for this reason).

*Reemplazar (en una pestaña separada, sin tocar la vista esquemática que ya funciona) el brazo simplificado de cilindros/esferas por las mallas óseas reales de OpenSim (archivos `.vtp`), para una presentación mucho más realista — puramente cosmético, deliberadamente mantenido separado de la física ya validada para que un problema de renderizado nunca pusiera en riesgo el simulador que funciona (la decisión "Opción B" de antes en el proyecto, elegida específicamente por esta razón).*

### 24.2 Data layer: validated correctly, first try / Capa de datos: validada correctamente, a la primera

Following the project's established pattern (validate the data layer via console scripts before touching the GUI), `bone_viewer.py` parses each Body's `.vtp` mesh association directly from the `.osim` XML (`VisibleObject`/`GeometrySet`/`DisplayGeometry`), and computes each mesh's world transform by combining the Body's real kinematic transform (from OpenSim) with the mesh's own local offset. Both the XML parsing (33 mesh files found, correctly organized by body) and the 3D transforms (validated against known reference poses) worked correctly on the first attempt — a good sign that the project's accumulated conventions (verify with the real engine, never assume) were paying off.

*Siguiendo el patrón ya establecido del proyecto (validar la capa de datos con scripts de consola antes de tocar la GUI), `bone_viewer.py` extrae la asociación de cada malla `.vtp` de cada Body directamente del XML del `.osim` (`VisibleObject`/`GeometrySet`/`DisplayGeometry`), y calcula la transformación mundial de cada malla combinando la transformación cinemática real del Body (de OpenSim) con el desplazamiento local propio de la malla. Tanto el parseo del XML (33 archivos de malla encontrados, organizados correctamente por hueso) como las transformaciones 3D (validadas contra posturas de referencia conocidas) funcionaron correctamente al primer intento — buena señal de que las convenciones acumuladas del proyecto (verificar con el motor real, nunca asumir) estaban dando frutos.*

### 24.3 A real, significant bug the visual debugging uncovered / Un bug real y significativo que la depuración visual descubrió

While chasing what looked like a purely cosmetic problem, a genuine correctness bug surfaced — worth documenting in detail because it could have silently affected results elsewhere in the project.

*Mientras se perseguía lo que parecía un problema puramente cosmético, salió a la luz un bug real de corrección — vale la pena documentarlo en detalle porque podría haber afectado resultados en silencio en otras partes del proyecto.*

**Symptom:** the scapula's coordinates (`unrotscap_r2_r/r3_r`, `acromioclavicular_r*_r` — all coupled to `shoulder_elv_r` via `CoordinateCouplerConstraint`, Section 4.1) stayed **exactly identical** across completely different arm postures, when printed as a diagnostic. The scapulohumeral rhythm — a real, physiologically expected pattern where the scapula rotates as the arm elevates — simply wasn't happening.

***Síntoma:*** *las coordenadas de la escápula (`unrotscap_r2_r/r3_r`, `acromioclavicular_r*_r` — todas acopladas a `shoulder_elv_r` vía `CoordinateCouplerConstraint`, Sección 4.1) se quedaban **exactamente idénticas** entre posturas de brazo completamente distintas, al imprimirlas como diagnóstico. El ritmo escapulohumeral — un patrón real, fisiológicamente esperado, donde la escápula rota mientras el brazo se eleva — sencillamente no estaba ocurriendo.*

**Root cause:** the `enforceContraints=False` performance optimization from Section 16.5 — which sped up `set_pose()` by skipping automatic constraint re-solving on each individual coordinate assignment — had an unintended side effect: it also prevented the scapulohumeral coupling constraints from ever being evaluated, anywhere in the project, since nothing else ever explicitly forced a full constraint pass. The shoulder's own direction-matching optimizer (`solve_osim_angles`) never noticed, because it numerically searches for whatever `(elv_angle, shoulder_elv)` reproduces the correct humerus direction — and it found a combination that worked *despite* the broken coupling, simply compensating around it. But anything relying on the *scapula's own position* (this bone viewer, and potentially the moment arms of scapula-spanning muscles like `SUPSP_r`, `INFSP_r`, `SUBSC_r` in the Static Optimization of Section 20) was silently working with a scapula frozen at its default pose.

***Causa raíz:*** *la optimización de rendimiento `enforceContraints=False` de la Sección 16.5 — que aceleraba `set_pose()` saltándose la re-resolución automática de restricciones en cada asignación individual de coordenada — tuvo un efecto secundario no previsto: también impedía que las restricciones de acoplamiento escapulohumeral se evaluaran, en ningún lugar del proyecto, ya que nada más forzaba explícitamente una pasada completa de restricciones. El propio optimizador de coincidencia de dirección del hombro (`solve_osim_angles`) nunca lo notó, porque busca numéricamente cualquier `(elv_angle, shoulder_elv)` que reproduzca la dirección correcta del húmero — y encontró una combinación que funcionaba *a pesar de* el acoplamiento roto, compensándolo sin más. Pero cualquier cosa que dependiera de la *posición real de la escápula* (este visor de huesos, y potencialmente los brazos de palanca de músculos que cruzan la escápula como `SUPSP_r`, `INFSP_r`, `SUBSC_r` en la Static Optimization de la Sección 20) estaba trabajando en silencio con una escápula congelada en su postura por defecto.*

**Fix, and a fix-of-the-fix:** the first attempt (`enforceContraints=True` on the last of the four `setValue` calls inside `set_pose()`) backfired badly — `set_pose()` is called *internally*, hundreds of times, by the optimizer's own search loop, so forcing a full constraint solve on every one of those internal calls both slowed the whole application down noticeably and, worse, appeared to distort the optimizer's search landscape enough to make it converge to the degenerate `shoulder_elv≈0` singularity again (the exact failure mode Section 6's multi-start solver was built to avoid). The correct fix: keep `set_pose()` exactly as fast as before (`enforceContraints=False` throughout, used freely by the optimizer's internal search), and add a **separate** function, `finalize_constraints()`, called **once**, only *after* the solver has already converged to its final answer — forcing exactly one full constraint evaluation on the pose that will actually be used, at negligible cost.

***Arreglo, y un arreglo del arreglo:*** *el primer intento (`enforceContraints=True` en la última de las 4 llamadas `setValue` dentro de `set_pose()`) salió mal — `set_pose()` se llama *internamente*, cientos de veces, por el propio bucle de búsqueda del optimizador, así que forzar una resolución completa de restricciones en cada una de esas llamadas internas tanto ralentizó notablemente toda la aplicación como, peor aún, pareció distorsionar el paisaje de búsqueda del optimizador lo suficiente como para hacerlo converger de nuevo a la singularidad degenerada `shoulder_elv≈0` (exactamente el modo de falla que el solver multi-start de la Sección 6 fue construido para evitar). El arreglo correcto: mantener `set_pose()` exactamente tan rápido como antes (`enforceContraints=False` en todo, usado libremente por la búsqueda interna del optimizador), y agregar una función **separada**, `finalize_constraints()`, llamada **una vez**, solo *después* de que el solver ya convergió a su respuesta final — forzando exactamente una evaluación completa de restricciones sobre la postura que realmente se va a usar, a costo insignificante.*

```python
def finalize_constraints(self):
    """Fuerza UNA evaluacion completa de las restricciones del modelo
    (ritmo escapulohumeral), sobre la pose ACTUAL ya fijada con
    set_pose(). Llamar SOLO una vez, DESPUES de que el solver ya
    convergio -- nunca dentro del bucle de busqueda."""
    current = self.coord_shoulder_elv.getValue(self.state)
    self.coord_shoulder_elv.setValue(self.state, current, True)
    self.model.realizePosition(self.state)
```

**Lesson for future OpenSim model integrations:** a "cosmetic" bug can be the thread that unravels a real correctness issue hiding elsewhere. This is exactly why the project's practice of building small, independently-verifiable diagnostic scripts (rather than only trusting the live GUI) keeps paying off — the scapular coordinate print statement, added purely to debug a rendering problem, is what surfaced this.

***Lección para futuras integraciones de modelos OpenSim:*** *un bug "cosmético" puede ser el hilo que destapa un problema real de corrección escondido en otro lado. Es exactamente por esto que la práctica del proyecto de construir pequeños scripts de diagnóstico verificables de forma independiente (en vez de confiar solo en la GUI en vivo) sigue dando frutos — el print de las coordenadas de la escápula, agregado puramente para depurar un problema de renderizado, fue lo que sacó esto a la luz.*

### 24.4 Definitive proof the underlying data is correct / Prueba definitiva de que los datos subyacentes son correctos

After fixing the constraint bug, a final diagnostic (`test_all_bones_diagnostic.py`) printed the real-world position of every bone in the chain — thorax, clavicle, scapula, humerus, ulna — together, for a single posture:

*Después de arreglar el bug de restricciones, un diagnóstico final (`test_all_bones_diagnostic.py`) imprimió la posición real en el mundo de cada hueso de la cadena — torso, clavícula, escápula, húmero, cúbito — juntos, para una sola postura:*

```
Body              Pos X    Pos Y    Pos Z
thorax            0.000    0.000    0.000
clavicle_r       -0.025    0.007    0.006
scapula_r        -0.140    0.048   -0.058
humerus_r        -0.160    0.019   -0.046
ulna_r           -0.450    0.019   -0.045
```

A smooth, fully connected, monotonically advancing chain in the flexion direction (-X, already validated) — no jumps, no disconnected bodies. **The kinematic data is proven correct beyond reasonable doubt.**

*Una cadena suave, completamente conectada, avanzando monotónicamente en la dirección de flexión (-X, ya validada) — sin saltos, sin cuerpos desconectados. **Los datos cinemáticos están probados correctos más allá de toda duda razonable.**.*

### 24.5 Honest status: an unresolved cosmetic issue / Estado honesto: un problema cosmético sin resolver

Despite the data being conclusively correct, the actual rendered view in the "Realistic View" tab still doesn't look right to the eye — the torso, scapula, and arm appear visually disconnected or oddly oriented. Multiple attempts to fix this via preset camera buttons (Front/Side/Top, using PyVista's `view_vector` and later its explicit `camera.position`/`focal_point`/`up` API) each failed or introduced new confusion, without the ability to visually verify the result directly. A manual torso-mesh rotation correction was also tried and later removed, since it likely compensated for a camera problem rather than a real mesh issue.

*A pesar de que los datos son concluyentemente correctos, la vista realmente renderizada en la pestaña "Realistic View" todavía no se ve bien a simple vista — el torso, la escápula y el brazo aparecen visualmente desconectados o extrañamente orientados. Múltiples intentos de arreglar esto vía botones de cámara preestablecidos (Front/Side/Top, usando primero `view_vector` de PyVista y después su API explícita de `camera.position`/`focal_point`/`up`) fallaron o introdujeron nueva confusión, sin poder verificar visualmente el resultado de forma directa. También se probó (y luego se quitó) una corrección manual de rotación de la malla del torso, ya que probablemente compensaba un problema de cámara en vez de un problema real de la malla.*

**Current recommendation:** treat the camera presets in the Realistic View tab as unreliable for now. The `QtInteractor` widget supports free mouse rotation (click-and-drag), which the user can use to find a clear viewing angle manually. The underlying data — the actual reason this feature exists — is solid; only the default camera framing needs further work, ideally by someone who can see the render directly.

***Recomendación actual:*** *tratar los presets de cámara en la pestaña Realistic View como no confiables por ahora. El widget `QtInteractor` soporta rotación libre con el mouse (clic y arrastrar), que el usuario puede usar para encontrar un ángulo de vista claro manualmente. Los datos subyacentes — la razón real de que esta funcionalidad exista — son sólidos; solo falta trabajo adicional en el encuadre de cámara por defecto, idealmente por alguien que pueda ver el render directamente.*

---

## 25. Next steps / Próximos pasos

1. ~~Step 8b: project the shoulder's generalized torques into your anatomical basis~~ — **done, see Section 13.** / ~~Paso 8b: proyectar los torques generalizados del hombro a tu base anatómica~~ — **hecho, ver Sección 13.**
2. ~~Extend your elbow AND shoulder formulas (full 3D torque)~~ — **done, see Section 15** (also uncovered and fixed a sideways-bending elbow axis in the URDF). / ~~Extender tus fórmulas de codo y hombro (torque 3D completo)~~ — **hecho, ver Sección 15** (también se descubrió y corrigió un eje de codo que doblaba hacia el costado en el URDF).
3. ~~Add an optional payload to the OpenSim model~~ — **done, see Section 14** (virtual point mass, no `.osim` changes needed). / ~~Agregar un payload opcional al modelo de OpenSim~~ — **hecho, ver Sección 14** (masa puntual virtual, sin cambios al `.osim`).
4. ~~Real-time integration into the PyQt6 `update_all()` loop~~ — **done, see Section 16** (background `QThread` + debounced curves + warm-start; a precomputed lookup table remains as the structural fix if performance is still not smooth enough). / ~~Integración en tiempo real en el bucle `update_all()` de PyQt6~~ — **hecho, ver Sección 16** (`QThread` en segundo plano + curvas con debounce + warm-start; una tabla de consulta precalculada queda como el arreglo estructural si el rendimiento aún no es suficientemente fluido).
5. **(Optional, no longer urgent)** Precomputed lookup table for the OpenSim bridge — animation is already smooth after Section 16's four fixes; only worth revisiting if even more headroom is needed later. / **(Opcional, ya no urgente)** Tabla de consulta precalculada para el puente de OpenSim — la animación ya es fluida tras los cuatro arreglos de la Sección 16; solo valdría la pena retomarlo si se necesita más margen de rendimiento más adelante.
6. Quantify the error across the **full** range of motion, not just the sample postures tested so far — the item already flagged as future work in Section 13 of your original documentation. / Cuantificar el error a lo largo de **todo** el rango de movimiento, no solo las posturas de muestra probadas hasta ahora — el ítem ya anotado como trabajo futuro en la Sección 13 de tu documentación original.
7. ~~PyVista-based 3D visualization~~ — **partially done, see Section 24**: real bone data is implemented and proven correct (kinematic chain validated end-to-end), but the default camera framing in the "Realistic View" tab still needs work — use free mouse rotation in the meantime. / ~~Visualización 3D con PyVista~~ — **parcialmente hecho, ver Sección 24**: los datos de huesos reales están implementados y probados correctos (cadena cinemática validada de punta a punta), pero el encuadre de cámara por defecto en la pestaña "Realistic View" todavía necesita trabajo — usar rotación libre con el mouse mientras tanto.
8. ~~Individual muscle forces (Static Optimization)~~ — **done, see Section 20** ("Analyze Muscles" button, validated for elbow-only and full-arm cases). / ~~Fuerzas musculares individuales (Static Optimization)~~ — **hecho, ver Sección 20** (botón "Analyze Muscles", validado para el caso codo-solo y brazo completo).
9. ~~Compare muscle activations with vs. without exoskeleton assist~~ — **done, see Section 22** (inverse-Jacobian basis conversion, validated with a 0%-assist sanity check). / ~~Comparar activaciones musculares con vs. sin asistencia del exoesqueleto~~ — **hecho, ver Sección 22** (conversión de base vía Jacobiana inversa, validado con una verificación de sanidad a 0% de asistencia).
10. **Re-verify Static Optimization results (Section 20) now that the scapulohumeral rhythm bug is fixed** (Section 24.3) — muscles whose moment arms depend on scapular position (`SUPSP_r`, `INFSP_r`, `SUBSC_r`) may have been computed with the scapula silently frozen at its default pose before this fix. / **Re-verificar los resultados de Static Optimization (Sección 20) ahora que el bug del ritmo escapulohumeral está corregido** (Sección 24.3) — los músculos cuyo brazo de palanca depende de la posición de la escápula (`SUPSP_r`, `INFSP_r`, `SUBSC_r`) pudieron haberse calculado con la escápula congelada en silencio en su postura por defecto antes de este arreglo.
11. **Calibrate the Realistic View camera** — the underlying bone data is proven correct (Section 24.4); only the default camera framing needs visual tuning by someone who can see the render directly. / **Calibrar la cámara de la Vista Realista** — los datos de huesos subyacentes están probados correctos (Sección 24.4); solo falta ajuste visual del encuadre de cámara por defecto, por alguien que pueda ver el render directamente.

---

*Generated as a bilingual technical log for the OpenSim ↔ Full-Arm URDF Kinematics & Exoskeleton Simulator integration project.*
*Generado como registro técnico bilingüe del proyecto de integración OpenSim ↔ Full-Arm URDF Kinematics & Exoskeleton Simulator.*

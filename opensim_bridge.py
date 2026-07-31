"""
opensim_bridge.py
==================
Puente cinematico entre el simulador URDF (shoulder_abd / shoulder_flex /
shoulder_rot / elbow_flex) y el modelo OpenSim MoBL-ARMS bimanual
(elv_angle_r / shoulder_elv_r / shoulder_rot_r / elbow_flexion_r).

ESTRATEGIA (Paso 6):
--------------------
En vez de derivar a mano las matrices de rotacion de OpenSim (los ejes reales
del .osim no son ejes puros X/Y/Z, son vectores medidos con pequenios errores
de alineacion cadaverica), dejamos que el propio motor de OpenSim calcule su
cinematica directa real (esto respeta automaticamente la restriccion de
acoplamiento shoulder1_r2_r = -elv_angle_r). Solo comparamos el VECTOR DE
DIRECCION real hombro->codo (humero) entre los dos modelos, y resolvemos
numericamente (elv_angle_r, shoulder_elv_r) para que OpenSim reproduzca la
misma direccion que tu cadena URDF.

CONVENCION DE EJES (ver Seccion 5 de la documentacion del simulador):
  Simulador URDF   : X = medio-lateral, Y = antero-posterior, Z = vertical
  OpenSim (thorax) : X = antero-posterior (+ = adelante),
                     Y = vertical (+ = arriba),
                     Z = medio-lateral (+ = derecha)

Mapeo de un vector v_urdf=(x,y,z) -> v_osim=(x',y',z'):
  x' = y   (antero-posterior)
  y' = z   (vertical)
  z' = x   (medio-lateral)

shoulder_rot (torsion sobre el eje longitudinal del humero) NO afecta la
direccion del hueso en ninguno de los dos modelos (es un giro sobre su propio
eje), asi que no participa en el matching de direccion; se mapea aparte
(Paso 7, requiere calibracion empirica de signo).
"""

import math
import numpy as np

try:
    import opensim as osim
except ImportError:
    osim = None  # Permite importar este modulo solo para las funciones URDF/matematicas


# --------------------------------------------------------------------------
# 1. Remapeo de convencion de ejes URDF -> OpenSim
# --------------------------------------------------------------------------
def urdf_to_osim_vec(v):
    """Convierte un vector (x,y,z) en convencion URDF (X=ML,Y=AP,Z=vert)
    a la convencion real de este modelo OpenSim (calibrada empiricamente
    por inspeccion visual en el visualizador de Simbody, ver Paso 6e):

        osim_X = -urdf_Y   ("adelante" es -X en este modelo, no +X)
        osim_Y =  urdf_Z   (vertical, sin cambio)
        osim_Z =  urdf_X   (lateral, sin cambio)

    Confirmado contra 3 posturas de referencia (reposo, abduccion pura,
    flexion pura) y contra observacion visual directa del modelo.
    """
    x, y, z = v
    return np.array([-y, z, x], dtype=float)


# --------------------------------------------------------------------------
# 2. Cinematica directa del lado URDF (independiente de OpenSim)
#    Reutiliza la misma logica que urdf_arm_simulator_EN_Final.py
# --------------------------------------------------------------------------
def _axis_angle_matrix(axis, theta):
    axis = np.array(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(theta), math.sin(theta)
    C = 1 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def urdf_humerus_direction(sh_abd, sh_flex):
    """Vector unitario (en convencion URDF) desde el hombro hacia el codo,
    dado shoulder_abd y shoulder_flex (en radianes). shoulder_rot no afecta
    esta direccion (es una torsion sobre el propio eje del hueso).

    Reproduce exactamente la cadena del URDF:
      torso -> shoulder_abd (eje -Y) -> shoulder_flex (eje +X) -> upper_arm
    con el hueso apuntando en -Z en su propio marco de reposo.
    """
    R_abd = _axis_angle_matrix([0, -1, 0], sh_abd)
    R_flex = _axis_angle_matrix([1, 0, 0], sh_flex)
    R_total = R_abd @ R_flex
    v0 = np.array([0.0, 0.0, -1.0])  # el hueso apunta -Z en reposo
    v = R_total @ v0
    return v / np.linalg.norm(v)


def urdf_elbow_axis_direction(sh_abd, sh_flex, sh_rot):
    """Vector unitario (convencion URDF) del eje de flexion del codo, visto
    desde el marco del torso, dados los 3 angulos del hombro. A diferencia
    de urdf_humerus_direction, aqui SI participa shoulder_rot (es justo el
    eje que gira con la torsion del humero -- por eso sirve para calibrarla).
    """
    R_abd = _axis_angle_matrix([0, -1, 0], sh_abd)
    R_flex = _axis_angle_matrix([1, 0, 0], sh_flex)
    R_rot = _axis_angle_matrix([0, 0, 1], sh_rot)
    R_total = R_abd @ R_flex @ R_rot
    axis_local = np.array([0.0, 1.0, 0.0])  # eje del elbow_flex en tu URDF
    v = R_total @ axis_local
    return v / np.linalg.norm(v)


# --------------------------------------------------------------------------
# 3. Cinematica directa del lado OpenSim (usa el motor real de OpenSim)
# --------------------------------------------------------------------------
class OpenSimArmModel:
    """Envoltura delgada sobre el modelo MoBL-ARMS para consultas de
    cinematica directa reutilizables durante el matching numerico."""

    def __init__(self, osim_path, side="r"):
        if osim is None:
            raise ImportError(
                "El paquete 'opensim' no esta instalado en este entorno de Python. "
                "Corre este script dentro del entorno conda 'exoarm_osim'."
            )
        self.side = side
        self.model = osim.Model(osim_path)
        self.state = self.model.initSystem()

        self.humerus_body = self.model.getBodySet().get(f"humerus_{side}")
        self.ulna_body = self.model.getBodySet().get(f"ulna_{side}")

        self.coord_elv_angle = self.model.getCoordinateSet().get(f"elv_angle_{side}")
        self.coord_shoulder_elv = self.model.getCoordinateSet().get(f"shoulder_elv_{side}")
        self.coord_shoulder_rot = self.model.getCoordinateSet().get(f"shoulder_rot_{side}")
        self.coord_elbow_flex = self.model.getCoordinateSet().get(f"elbow_flexion_{side}")

        # Eje de flexion del codo, en el marco LOCAL del humero (leido
        # directamente del .osim: TransformAxis 'rotation1' del joint
        # elbow_r/elbow_l). Se usa para calibrar shoulder_rot (Paso 7).
        axis = np.array([0.04940001, 0.03660001, 0.99810825])
        self._elbow_axis_local = axis / np.linalg.norm(axis)

    def set_pose(self, elv_angle, shoulder_elv, shoulder_rot=0.0, elbow_flex=None):
        # enforceContraints=False: rapido, usado tanto internamente por el
        # optimizador (cientos de llamadas por busqueda, Seccion 16.5) como
        # por cualquier llamada externa normal. NO evalua el ritmo
        # escapulohumeral (ver finalize_constraints() para cuando eso
        # importa, ej. antes de leer la posicion de la escapula para
        # renderizado o de musculos que dependen de ella).
        self.coord_elv_angle.setValue(self.state, float(elv_angle), False)
        self.coord_shoulder_elv.setValue(self.state, float(shoulder_elv), False)
        self.coord_shoulder_rot.setValue(self.state, float(shoulder_rot), False)
        if elbow_flex is not None:
            self.coord_elbow_flex.setValue(self.state, float(elbow_flex), False)
        self.model.realizePosition(self.state)

    def finalize_constraints(self):
        """Fuerza UNA evaluacion completa de las restricciones del modelo
        (ritmo escapulohumeral: unrotscap_r2_r/r3_r, acromioclavicular_r*_r,
        sternoclavicular_r*_r -- todas acopladas a shoulder_elv_r, Seccion
        4.1 del tutorial), sobre la pose ACTUAL ya fijada con set_pose().

        Llamar SOLO una vez, DESPUES de que el solver (solve_osim_angles,
        etc.) ya convergio a la pose final -- nunca dentro del bucle de
        busqueda del optimizador (eso fue un bug real: activar esto en
        cada llamada interna de set_pose distorsionaba la busqueda y la
        volvia mucho mas lenta, Seccion 16.5/24 del tutorial). Necesario
        antes de leer la posicion de la escapula (visor de huesos) o
        brazos de palanca de musculos que dependen de ella.
        """
        current = self.coord_shoulder_elv.getValue(self.state)
        self.coord_shoulder_elv.setValue(self.state, current, True)
        self.model.realizePosition(self.state)

    def humerus_direction(self):
        """Vector unitario (en convencion OpenSim, thorax) desde el origen
        del humero hacia el origen del cubito (hombro -> codo real)."""
        ground = self.model.getGround()
        p_shoulder = self.humerus_body.findStationLocationInAnotherFrame(
            self.state, osim.Vec3(0, 0, 0), ground
        )
        p_elbow = self.ulna_body.findStationLocationInAnotherFrame(
            self.state, osim.Vec3(0, 0, 0), ground
        )
        v = np.array([p_elbow.get(i) - p_shoulder.get(i) for i in range(3)])
        return v / np.linalg.norm(v)

    def elbow_axis_direction(self):
        """Vector unitario (convencion OpenSim) del eje de flexion del codo,
        transformado del marco local del humero al marco del suelo, dada la
        pose actual (incluye el efecto real de shoulder_rot_r)."""
        ground = self.model.getGround()
        p0 = self.humerus_body.findStationLocationInAnotherFrame(
            self.state, osim.Vec3(0, 0, 0), ground
        )
        p1 = self.humerus_body.findStationLocationInAnotherFrame(
            self.state, osim.Vec3(*self._elbow_axis_local), ground
        )
        v = np.array([p1.get(i) - p0.get(i) for i in range(3)])
        return v / np.linalg.norm(v)


def solve_osim_angles(osim_model: OpenSimArmModel, sh_abd, sh_flex,
                       n_starts=8, verbose=False):
    """Dado shoulder_abd/shoulder_flex (radianes) de tu simulador URDF,
    encuentra (elv_angle_r, shoulder_elv_r) tal que la direccion real
    hombro->codo del modelo OpenSim coincide con la de tu URDF.

    Usa una estimacion analitica de shoulder_elv (el angulo de elevacion
    real medido desde la posicion de reposo) mas 'n_starts' arranques
    distintos de elv_angle repartidos en su rango valido, combinados con
    dos ramas de shoulder_elv (la estimacion directa y su complementaria
    180-estimacion), para evitar quedar atrapado en minimos locales o en
    la singularidad geometrica de shoulder_elv ~ 0.

    Con verbose=True imprime cada arranque probado (util solo para
    depuracion); por defecto solo se calcula en silencio.

    Devuelve: (elv_angle, shoulder_elv, error_residual_en_grados)
    """
    from scipy.optimize import minimize

    v0 = np.array([0.0, 0.0, -1.0])
    v_urdf = urdf_humerus_direction(sh_abd, sh_flex)
    v_target = urdf_to_osim_vec(v_urdf)

    # Estimacion analitica: shoulder_elv es, por definicion, el angulo de
    # elevacion medido desde la posicion de reposo (brazo colgando) -> es
    # exactamente el angulo entre v_urdf y v0.
    shoulder_elv_estimate = math.acos(np.clip(np.dot(v_urdf, v0), -1.0, 1.0))

    def cost(x):
        elv_angle, shoulder_elv = x
        osim_model.set_pose(elv_angle, shoulder_elv)
        v = osim_model.humerus_direction()
        return np.sum((v - v_target) ** 2)

    lo = [osim_model.coord_elv_angle.getRangeMin(), osim_model.coord_shoulder_elv.getRangeMin()]
    hi = [osim_model.coord_elv_angle.getRangeMax(), osim_model.coord_shoulder_elv.getRangeMax()]
    bounds = list(zip(lo, hi))

    shoulder_elv_branches = [
        np.clip(shoulder_elv_estimate, *bounds[1]),
        np.clip(math.pi - shoulder_elv_estimate, *bounds[1]),
    ]
    elv_angle_candidates = np.linspace(bounds[0][0], bounds[0][1], n_starts)

    best_x, best_cost = None, np.inf
    for shoulder_elv_guess in shoulder_elv_branches:
        for elv_angle_guess in elv_angle_candidates:
            x0 = [elv_angle_guess, shoulder_elv_guess]
            res = minimize(cost, x0, bounds=bounds, method="L-BFGS-B",
                            options={"maxiter": 50})
            if verbose:
                print(f"    elv0={math.degrees(elv_angle_guess):6.1f} "
                      f"shelv0={math.degrees(shoulder_elv_guess):6.1f}  "
                      f"-> costo={res.fun:.2e}", flush=True)
            if res.fun < best_cost:
                best_cost, best_x = res.fun, res.x

    elv_angle, shoulder_elv = best_x
    osim_model.set_pose(elv_angle, shoulder_elv)
    v_final = osim_model.humerus_direction()
    error_deg = math.degrees(math.acos(np.clip(np.dot(v_final, v_target), -1.0, 1.0)))
    return elv_angle, shoulder_elv, error_deg


# --------------------------------------------------------------------------
# 5. Solver 1D: calibrar shoulder_rot dado que (elv_angle, shoulder_elv) ya
#    se resolvieron (Paso 7)
# --------------------------------------------------------------------------
def solve_shoulder_rot(osim_model: OpenSimArmModel, sh_abd, sh_flex, sh_rot,
                        elv_angle, shoulder_elv):
    """Dado un shoulder_rot (radianes) de tu URDF y el (elv_angle,
    shoulder_elv) ya resueltos para la misma pose, encuentra shoulder_rot_r
    tal que el eje de flexion del codo en OpenSim apunte en la misma
    direccion real que en tu URDF.

    Devuelve: (shoulder_rot_r, error_residual_en_grados)
    """
    from scipy.optimize import minimize_scalar

    v_target = urdf_to_osim_vec(urdf_elbow_axis_direction(sh_abd, sh_flex, sh_rot))

    def cost(shoulder_rot_r):
        osim_model.set_pose(elv_angle, shoulder_elv, shoulder_rot=shoulder_rot_r)
        v = osim_model.elbow_axis_direction()
        return float(np.sum((v - v_target) ** 2))

    lo = osim_model.coord_shoulder_rot.getRangeMin()
    hi = osim_model.coord_shoulder_rot.getRangeMax()
    res = minimize_scalar(cost, bounds=(lo, hi), method="bounded")
    shoulder_rot_r = res.x

    osim_model.set_pose(elv_angle, shoulder_elv, shoulder_rot=shoulder_rot_r)
    v_final = osim_model.elbow_axis_direction()
    error_deg = math.degrees(math.acos(np.clip(np.dot(v_final, v_target), -1.0, 1.0)))
    return shoulder_rot_r, error_deg


# --------------------------------------------------------------------------
# 6. Torque gravitacional via Dinamica Inversa real de OpenSim (Paso 8)
# --------------------------------------------------------------------------
def _payload_ground_position(model, state, f_len):
    """Posicion (en el marco del suelo) de un payload situado exactamente
    a distancia 'f_len' del codo, siguiendo la direccion REAL codo->mano
    que calcula OpenSim en la postura actual -- replica exactamente la
    convencion de tu URDF (payload a F_LEN del codo, en la punta del
    antebrazo), en vez de usar una ubicacion arbitraria como el centro de
    masa de 'hand_r'. Esto aisla cualquier diferencia restante a la
    distribucion antropometrica real, no a un brazo de palanca mal
    alineado entre los dos modelos.
    """
    ground = model.getGround()
    ulna = model.getBodySet().get("ulna_r")
    hand = model.getBodySet().get("hand_r")

    p_elbow = ulna.findStationLocationInAnotherFrame(state, osim.Vec3(0, 0, 0), ground)
    p_hand = hand.findStationLocationInAnotherFrame(state, osim.Vec3(0, 0, 0), ground)
    elbow = np.array([p_elbow.get(i) for i in range(3)])
    hand_pos = np.array([p_hand.get(i) for i in range(3)])

    direction = hand_pos - elbow
    norm = np.linalg.norm(direction)
    if norm < 1e-9:
        direction = np.array([0.0, 0.0, -1.0])
    else:
        direction = direction / norm

    return elbow + f_len * direction


def _total_gravitational_pe(model, state, payload_mass=0.0, payload_f_len=0.27):
    """Energia potencial gravitacional total del modelo (suma de
    masa*g*altura de cada Body), calculada a mano con la misma tecnica
    (findStationLocationInAnotherFrame) ya validada en humerus_direction().
    Evita depender de metodos a nivel de 'System' (calcPotentialEnergy)
    que resultaron no estar expuestos de forma utilizable en esta version
    de los bindings de Python.

    payload_mass (kg), opcional: agrega una masa puntual VIRTUAL colocada
    exactamente a 'payload_f_len' metros del codo, siguiendo la direccion
    real codo->mano (ver _payload_ground_position) -- replica tu
    convencion de payload en la punta del antebrazo, sin modificar el
    archivo .osim ni el modelo en si.
    """
    g_vec = model.getGravity()
    g = np.array([g_vec.get(0), g_vec.get(1), g_vec.get(2)])
    ground = model.getGround()
    body_set = model.getBodySet()

    total = 0.0
    for i in range(body_set.getSize()):
        body = body_set.get(i)
        m = body.getMass()
        if m <= 0:
            continue  # cuerpos ficticios sin masa (ej. 'thorax'), no aportan
        com_local = body.get_mass_center()
        p = body.findStationLocationInAnotherFrame(state, com_local, ground)
        r = np.array([p.get(0), p.get(1), p.get(2)])
        total += -m * float(np.dot(g, r))

    if payload_mass > 0:
        r = _payload_ground_position(model, state, payload_f_len)
        total += -payload_mass * float(np.dot(g, r))

    return total


def gravity_only_generalized_forces(osim_model: OpenSimArmModel, coord_names=None, eps=1e-4,
                                     payload_mass=0.0, payload_f_len=0.27):
    """Calcula el torque generalizado (Nm) necesario para sostener la
    postura ACTUAL del modelo (la que ya se fijo con set_pose) unicamente
    contra la gravedad.

    NOTA DE DISEÑO: se intento primero usar 'InverseDynamicsSolver' + el
    indice de coordenada via 'getMobilizerQIndex()' (devuelve el indice
    LOCAL dentro del mobilizador, no el global -- bug detectado), y luego
    'MultibodySystem.calcPotentialEnergy()' (no expuesto de forma usable
    en estos bindings). En su lugar, se calcula la energia potencial
    gravitacional a mano sumando masa*g*altura de cada Body
    (_total_gravitational_pe), y se deriva por diferencias finitas
    centrales: tau_i = +dU/dq_i (el torque que los musculos/motor deben
    aplicar para SOSTENER la postura es el opuesto al que la gravedad
    ejerce por si sola, que seria -dU/dq_i -- esta es la misma convencion
    de signo que usa tu formula seno/coseno). Esto es la definicion fisica
    torque de sostenimiento estatico contra gravedad, y no depende de
    ningun indexado ni metodo interno de Simbody.

    Devuelve un dict {nombre_coordenada: torque_generalizado_Nm}.

    NOTA IMPORTANTE (sin cambios): para el codo (elbow_flexion_r) esto es
    directamente comparable con tu torque_elbow. Para el hombro,
    elv_angle_r/shoulder_elv_r NO estan en la misma base que tu
    shoulder_flex/shoulder_abd -- comparar esos requiere proyectar con la
    matriz Jacobiana de la conversion de angulos (Paso 8b).
    """
    model = osim_model.model
    state = osim_model.state

    if coord_names is None:
        coord_set = model.getCoordinateSet()
        coord_names = [coord_set.get(i).getName() for i in range(coord_set.getSize())]

    result = {}
    for name in coord_names:
        coord = model.getCoordinateSet().get(name)
        q0 = coord.getValue(state)

        coord.setValue(state, q0 + eps, False)
        model.realizePosition(state)
        u_plus = _total_gravitational_pe(model, state, payload_mass, payload_f_len)

        coord.setValue(state, q0 - eps, False)
        model.realizePosition(state)
        u_minus = _total_gravitational_pe(model, state, payload_mass, payload_f_len)

        coord.setValue(state, q0, False)  # restaurar el valor original
        model.realizePosition(state)

        result[name] = (u_plus - u_minus) / (2 * eps)

    return result


# --------------------------------------------------------------------------
# 7. Proyeccion de torques de hombro a tu base anatomica (Paso 8b)
# --------------------------------------------------------------------------
def _solve_osim_angles_local(osim_model: OpenSimArmModel, sh_abd, sh_flex,
                              elv_guess, shoulder_elv_guess):
    """Refinamiento LOCAL (sin multi-start) de (elv_angle, shoulder_elv),
    anclado a una solucion base ya conocida. Se usa para las perturbaciones
    pequeñas de la Jacobiana numerica, para evitar que el solver salte a
    una rama de solucion distinta (la ambiguedad de branches que ya vimos
    en el Paso 6) -- con una perturbacion pequeña, la solucion correcta
    esta muy cerca de la base, asi que un refinamiento local es suficiente
    y mucho mas rapido que repetir el multi-start completo.
    """
    from scipy.optimize import minimize

    v_target = urdf_to_osim_vec(urdf_humerus_direction(sh_abd, sh_flex))

    def cost(x):
        osim_model.set_pose(x[0], x[1])
        v = osim_model.humerus_direction()
        return np.sum((v - v_target) ** 2)

    bounds = [
        (osim_model.coord_elv_angle.getRangeMin(), osim_model.coord_elv_angle.getRangeMax()),
        (osim_model.coord_shoulder_elv.getRangeMin(), osim_model.coord_shoulder_elv.getRangeMax()),
    ]
    res = minimize(cost, [elv_guess, shoulder_elv_guess], bounds=bounds, method="L-BFGS-B",
                    options={"maxiter": 3, "ftol": 1e-8})
    return res.x[0], res.x[1]


def solve_osim_angles_warmstart(osim_model: OpenSimArmModel, sh_abd, sh_flex,
                                 elv_guess, shoulder_elv_guess):
    """Version 'warm-start' de solve_osim_angles: un UNICO refinamiento
    local (sin multi-start), partiendo de la solucion de la postura
    anterior. Muchisimo mas rapida que solve_osim_angles (32 arranques),
    pensada para uso interactivo en tiempo real donde la postura cambia
    gradualmente (sliders) entre una actualizacion y la siguiente -- la
    solucion nueva casi siempre esta muy cerca de la anterior, asi que un
    refinamiento local basta.

    ADVERTENCIA: si la postura cambia MUCHO de golpe entre llamadas (salto
    grande, no gradual), esta version puede quedar atrapada en un minimo
    local peor que el optimo global -- en ese caso, usar solve_osim_angles
    (multi-start) para 're-anclar' la solucion.

    Devuelve: (elv_angle, shoulder_elv, error_residual_en_grados)
    """
    elv_angle, shoulder_elv = _solve_osim_angles_local(
        osim_model, sh_abd, sh_flex, elv_guess, shoulder_elv_guess
    )
    v_target = urdf_to_osim_vec(urdf_humerus_direction(sh_abd, sh_flex))
    v_final = osim_model.humerus_direction()
    error_deg = math.degrees(math.acos(np.clip(np.dot(v_final, v_target), -1.0, 1.0)))
    return elv_angle, shoulder_elv, error_deg



def shoulder_angle_jacobian(osim_model: OpenSimArmModel, sh_abd, sh_flex,
                             elv_angle, shoulder_elv, eps=1e-4):
    """Jacobiana numerica 2x2 J = d(elv_angle, shoulder_elv) / d(sh_abd, sh_flex),
    evaluada en la pose actual, por diferencias finitas centrales usando el
    refinamiento local (_solve_osim_angles_local) anclado a la solucion base
    ya conocida, para mantener la continuidad de rama.
    """
    def solve(abd, flex):
        e, s = _solve_osim_angles_local(osim_model, abd, flex, elv_angle, shoulder_elv)
        return np.array([e, s])

    d_dabd = (solve(sh_abd + eps, sh_flex) - solve(sh_abd - eps, sh_flex)) / (2 * eps)
    d_dflex = (solve(sh_abd, sh_flex + eps) - solve(sh_abd, sh_flex - eps)) / (2 * eps)

    J = np.column_stack([d_dabd, d_dflex])  # J[:,0]=d(elv,she)/d(abd), J[:,1]=d(elv,she)/d(flex)
    return J


def project_shoulder_torque_to_urdf(osim_model: OpenSimArmModel, sh_abd, sh_flex,
                                     elv_angle, shoulder_elv, tau_elv_angle, tau_shoulder_elv):
    """Proyecta los torques generalizados de OpenSim (base elv_angle/
    shoulder_elv) a tu base anatomica (shoulder_abd/shoulder_flex), via
    tau_urdf = J^T @ tau_osim -- la relacion estandar de trabajo virtual
    entre dos parametrizaciones distintas de la MISMA rotacion fisica.

    Devuelve: (tau_sh_abd, tau_sh_flex) en Nm.
    """
    J = shoulder_angle_jacobian(osim_model, sh_abd, sh_flex, elv_angle, shoulder_elv)
    tau_osim = np.array([tau_elv_angle, tau_shoulder_elv])
    tau_urdf = J.T @ tau_osim
    return float(tau_urdf[0]), float(tau_urdf[1])


# --------------------------------------------------------------------------
# 8. Fuerzas musculares individuales via Static Optimization (Opcion 3)
# --------------------------------------------------------------------------
def solve_muscle_activations(osim_model: OpenSimArmModel, coord_names, target_torques,
                              moment_arm_threshold=1e-4, side="r"):
    """Reparte un torque neto requerido (ya calculado, ej. con
    gravity_only_generalized_forces) entre los musculos individuales que
    cruzan esas articulaciones, usando el mismo criterio que la
    herramienta 'Static Optimization' de OpenSim: minimizar la suma de
    activaciones al cuadrado, sujeto a que la suma de (fuerza_musculo x
    brazo_de_palanca) reproduzca exactamente el torque requerido en cada
    coordenada.

    NO se asume de antemano que musculos cruzan cada articulacion -- se
    deja que OpenSim calcule el brazo de palanca real
    (muscle.computeMomentArm) de CADA musculo del modelo, y solo se
    incluyen los que tienen un brazo de palanca no-nulo (mismo principio
    de "verificar, no asumir" usado en todo este proyecto).

    Parametros:
        coord_names: lista de nombres de coordenadas, ej. ['elbow_flexion_r']
        target_torques: dict {nombre_coordenada: torque_Nm} -- el torque de
            SOSTENIMIENTO requerido (misma convencion de signo que
            gravity_only_generalized_forces / project_shoulder_torque_to_urdf)
        moment_arm_threshold: brazo de palanca minimo (m) para considerar
            que un musculo "cruza" esa articulacion
        side: 'r' o 'l' -- filtra solo musculos de ese lado

    Devuelve: dict {nombre_musculo: {activation, force_N, moment_arms_m,
    max_isometric_force_N}}, mas una entrada '_meta' con info del solver.
    """
    from scipy.optimize import minimize

    model = osim_model.model
    state = osim_model.state
    model.realizePosition(state)

    muscle_set = model.getMuscles()
    coords = {name: model.getCoordinateSet().get(name) for name in coord_names}

    relevant = []
    moment_arms = {}
    fmax = {}
    suffix = f"_{side}"
    for i in range(muscle_set.getSize()):
        m = muscle_set.get(i)
        name = m.getName()
        if not name.endswith(suffix):
            continue
        arms = {}
        has_arm = False
        for cname, coord in coords.items():
            ma = m.computeMomentArm(state, coord)
            arms[cname] = ma
            if abs(ma) > moment_arm_threshold:
                has_arm = True
        if has_arm:
            relevant.append(name)
            moment_arms[name] = arms
            fmax[name] = m.getMaxIsometricForce()

    result = {}
    n = len(relevant)
    if n == 0:
        result["_meta"] = {"success": False, "message": "Ningun musculo relevante encontrado."}
        return result

    coord_list = list(coord_names)
    A = np.zeros((len(coord_list), n))
    for j, name in enumerate(relevant):
        for i, cname in enumerate(coord_list):
            A[i, j] = moment_arms[name][cname] * fmax[name]
    b = np.array([target_torques[c] for c in coord_list])

    def cost(a):
        return float(np.sum(a ** 2))

    def cost_grad(a):
        return 2.0 * a

    constraints = [
        {"type": "eq", "fun": (lambda a, i=i: A[i, :] @ a - b[i])}
        for i in range(len(coord_list))
    ]
    bounds = [(0.0, 1.0)] * n
    x0 = np.full(n, 0.1)

    res = minimize(cost, x0, jac=cost_grad, method="SLSQP", bounds=bounds,
                    constraints=constraints, options={"maxiter": 200, "ftol": 1e-10})

    for j, name in enumerate(relevant):
        a = float(res.x[j])
        result[name] = {
            "activation": a,
            "force_N": a * fmax[name],
            "moment_arms_m": moment_arms[name],
            "max_isometric_force_N": fmax[name],
        }

    residual = A @ res.x - b
    result["_meta"] = {
        "success": bool(res.success),
        "message": res.message,
        "residual_Nm": residual.tolist(),
        "n_muscles": n,
    }
    return result

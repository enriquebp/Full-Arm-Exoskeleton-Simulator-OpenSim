"""
urdf_kinematics_preview.py
============================
Previsualizador completo de cinemática URDF con:
- Estilo UI Dark CAD con fondo negro/gris oscuro.
- Leyenda limpia: Sin etiquetas de "Upper Arm" ni "Forearm".
- Leyenda ubicada dinámicamente por encima del sector de Azim / Elev / Zoom.
- Control Panel ajustado para visualización óptima de las 4 pestañas (incluyendo Ecuaciones).
- Barra de herramientas superior (QToolBar) para accesos rápidos.
- Paneles colapsables (Acordeón) para los sensores IMU.
- Toggle Switch moderno para unidades de Exoesqueleto (% / Nm).
- Seguimiento de trayectoria 3D en tiempo real.
- Gráficas de Torque vs Ángulo con curvas de Torque Humano, Torque Exo y Torque Neto.
- Imagen de referencia visual en la pestaña Kinematics.
- Pestaña de Ecuaciones con tipografía ampliada (más grande) y la figura posicionada exactamente debajo del bloque superior.
"""

import sys
import os
import math
import xml.etree.ElementTree as ET

import numpy as np

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, 
    QGroupBox, QLabel, QSlider, QDoubleSpinBox, QSizePolicy, QScrollArea, 
    QPushButton, QComboBox, QToolBar, QDockWidget, QTabWidget, QToolButton, QCheckBox,
    QDialog
)
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt6.QtGui import QPainter, QColor, QPixmap, QKeySequence, QShortcut

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from torque_3d import compute_full_3d_torques  # Paso 1: torque 3D completo (r x F)

try:
    import pyvista as pv
    from pyvistaqt import QtInteractor
    PYVISTA_AVAILABLE = True
except ImportError:
    PYVISTA_AVAILABLE = False


# --------------------------------------------------------------------------
# Utilidades de transformaciones homogéneas y Cuaterniones
# --------------------------------------------------------------------------
def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def matrix_to_quaternion(R):
    tr = np.trace(R)
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return np.array([w, x, y, z])


def axis_angle_matrix(axis, theta):
    axis = np.array(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-9:
        return np.eye(3)
    axis = axis / n
    x, y, z = axis
    c, s = math.cos(theta), math.sin(theta)
    C = 1 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def make_transform(xyz, rpy_or_R):
    T = np.eye(4)
    if isinstance(rpy_or_R, tuple) or (isinstance(rpy_or_R, np.ndarray) and rpy_or_R.shape == (3,)):
        T[:3, :3] = rpy_to_matrix(*rpy_or_R)
    else:
        T[:3, :3] = rpy_or_R
    T[:3, 3] = xyz
    return T


# --------------------------------------------------------------------------
# Parser de URDF Base
# --------------------------------------------------------------------------
def parse_xyz(elem, tag="origin", attr="xyz"):
    o = elem.find(tag)
    if o is None or o.get(attr) is None:
        return np.zeros(3)
    return np.array([float(v) for v in o.get(attr).split()])


def parse_rpy(elem, tag="origin"):
    o = elem.find(tag)
    if o is None or o.get("rpy") is None:
        return np.zeros(3)
    return np.array([float(v) for v in o.get("rpy").split()])


def parse_geometry(visual_elem):
    geom = visual_elem.find("geometry")
    if geom is None:
        return None
    box = geom.find("box")
    if box is not None:
        return {"type": "box", "size": [float(v) for v in box.get("size").split()]}
    cyl = geom.find("cylinder")
    if cyl is not None:
        return {"type": "cylinder", "radius": float(cyl.get("radius")), "length": float(cyl.get("length"))}
    sph = geom.find("sphere")
    if sph is not None:
        return {"type": "sphere", "radius": float(sph.get("radius"))}
    return None


def parse_urdf(path):
    tree = ET.parse(path)
    root = tree.getroot()

    links = {}
    for link in root.findall("link"):
        name = link.get("name")
        visual = link.find("visual")
        geometry = parse_geometry(visual) if visual is not None else None
        vis_xyz = parse_xyz(visual) if visual is not None else np.zeros(3)
        vis_rpy = parse_rpy(visual) if visual is not None else np.zeros(3)
        links[name] = {"geometry": geometry, "vis_xyz": vis_xyz, "vis_rpy": vis_rpy}

    joints = []
    for j in root.findall("joint"):
        jtype = j.get("type")
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        origin_xyz = parse_xyz(j)
        origin_rpy = parse_rpy(j)
        axis_elem = j.find("axis")
        axis = np.array([float(v) for v in axis_elem.get("xyz").split()]) if axis_elem is not None else np.array([1.0, 0, 0])
        
        limit_elem = j.find("limit")
        lower, upper = 0.0, 0.0
        if limit_elem is not None:
            lower_str = limit_elem.get("lower")
            upper_str = limit_elem.get("upper")
            if lower_str is not None:
                lower = float(lower_str)
            if upper_str is not None:
                upper = float(upper_str)

        joints.append({
            "name": j.get("name"), "type": jtype, "parent": parent, "child": child,
            "origin_xyz": origin_xyz, "origin_rpy": origin_rpy, "axis": axis,
            "lower": lower, "upper": upper,
        })

    child_links = {j["child"] for j in joints}
    root_candidates = [name for name in links if name not in child_links]
    root_link = root_candidates[0] if root_candidates else next(iter(links))

    return links, joints, root_link


# --------------------------------------------------------------------------
# Componentes UI Personalizados
# --------------------------------------------------------------------------
class ToggleSwitch(QCheckBox):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(48, 22)
        self.setCursor(Qt.CursorShape.PointingHandCursor)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        bg_color = QColor("#1f6feb") if self.isChecked() else QColor("#262626")
        handle_color = QColor("#ffffff")

        p.setBrush(bg_color)
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRoundedRect(0, 0, self.width(), self.height(), 11, 11)

        x_pos = 28 if self.isChecked() else 3
        p.setBrush(handle_color)
        p.drawEllipse(x_pos, 3, 16, 16)


class CollapsiblePanel(QWidget):
    def __init__(self, title="", parent=None):
        super().__init__(parent)
        
        self.toggle_button = QToolButton()
        self.toggle_button.setText(f"▶  {title}")
        self.toggle_button.setCheckable(True)
        self.toggle_button.setChecked(False)
        self.toggle_button.setStyleSheet("""
            QToolButton {
                border: 1px solid #262626;
                background-color: #121212;
                color: #58a6ff;
                font-weight: bold;
                text-align: left;
                padding: 6px;
                border-radius: 4px;
                font-size: 11px;
            }
            QToolButton:hover {
                background-color: #1f1f1f;
                color: #ffffff;
            }
            QToolButton:checked {
                background-color: #1f6feb;
                color: #ffffff;
            }
        """)
        self.toggle_button.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self.toggle_button.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.toggle_button.clicked.connect(self.on_toggle)

        self.content_area = QWidget()
        self.content_area.setVisible(False)
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.setSpacing(2)
        layout.addWidget(self.toggle_button)
        layout.addWidget(self.content_area)

    def on_toggle(self, checked):
        arrow = "▼" if checked else "▶"
        title_text = self.toggle_button.text()[3:]
        self.toggle_button.setText(f"{arrow}  {title_text}")
        self.content_area.setVisible(checked)

    def setContentLayout(self, layout):
        self.content_area.setLayout(layout)


class LabeledSlider(QWidget):
    def __init__(self, label, minimum, maximum, value, suffix=" °", decimals=1, accent_color="#58a6ff", parent=None):
        super().__init__(parent)
        self.decimals = decimals
        self.scale = 10 ** decimals

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 1, 0, 1)

        top = QHBoxLayout()
        top.setContentsMargins(0, 0, 0, 0)
        self.label = QLabel(label)
        self.label.setStyleSheet(f"color:{accent_color}; font-weight:700; font-size:11px; background: transparent;")
        top.addWidget(self.label)
        top.addStretch()
        self.spin = QDoubleSpinBox()
        self.spin.setDecimals(decimals)
        self.spin.setMinimum(minimum)
        self.spin.setMaximum(maximum)
        self.spin.setSingleStep(1.0 if decimals == 1 else 0.1)
        self.spin.setValue(value)
        self.spin.setSuffix(suffix)
        self.spin.setFixedWidth(80)
        self.spin.setStyleSheet(f"background-color: #080808; color: {accent_color}; border: 1px solid #262626; border-radius: 4px; font-size: 11px; font-weight: bold;")
        top.addWidget(self.spin)
        layout.addLayout(top)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setMinimum(int(minimum * self.scale))
        self.slider.setMaximum(int(maximum * self.scale))
        self.slider.setValue(int(value * self.scale))
        self.slider.setStyleSheet(f"""
            QSlider::groove:horizontal {{ border: 1px solid #262626; height: 4px; background: #080808; border-radius: 2px; }}
            QSlider::sub-page:horizontal {{ background: {accent_color}; border-radius: 2px; }}
            QSlider::handle:horizontal {{ background: #f0f6fc; border: 1px solid {accent_color}; width: 12px; margin: -4px 0; border-radius: 6px; }}
        """)
        layout.addWidget(self.slider)

        self.slider.valueChanged.connect(self._slider_changed)
        self.spin.valueChanged.connect(self._spin_changed)

    def _slider_changed(self, v):
        self.spin.blockSignals(True)
        self.spin.setValue(v / self.scale)
        self.spin.blockSignals(False)

    def _spin_changed(self, v):
        self.slider.blockSignals(True)
        self.slider.setValue(int(v * self.scale))
        self.slider.blockSignals(False)

    def value(self):
        return self.spin.value()

    def setValue(self, val):
        self.spin.setValue(val)


# --------------------------------------------------------------------------
# Triada de ejes (X/Y/Z) para IMU
# --------------------------------------------------------------------------
def draw_axis_triad(ax, position, R, length=0.06, label=None, label_color="#c9d1d9",
                     linewidth=2.4, add_legend_labels=False):
    axis_specs = [
        (R[:, 0], "#e5484d", "IMU X-axis"),
        (R[:, 1], "#2ea043", "IMU Y-axis"),
        (R[:, 2], "#58a6ff", "IMU Z-axis"),
    ]
    for direction, color, axis_label in axis_specs:
        d = direction * length
        ax.quiver(position[0], position[1], position[2],
                  d[0], d[1], d[2],
                  color=color, linewidth=linewidth, arrow_length_ratio=0.35,
                  label=axis_label if add_legend_labels else None)
    if label:
        ax.text(position[0], position[1], position[2] + length * 1.3, label,
                color=label_color, fontsize=8, fontweight="bold", ha="center")


# --------------------------------------------------------------------------
# Canvas 3D Interactivo
# --------------------------------------------------------------------------
class InteractiveCanvas3D(FigureCanvasQTAgg):
    def __init__(self, fig, main_window):
        super().__init__(fig)
        self.main_window = main_window
        self.press_pos = None

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        current_zoom = self.main_window.slider_zoom.value()
        if delta > 0:
            new_zoom = min(3.0, current_zoom * 1.1)
        else:
            new_zoom = max(0.1, current_zoom / 1.1)
        self.main_window.slider_zoom.setValue(new_zoom)
        event.accept()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.press_pos = event.pos()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.MouseButton.LeftButton and self.press_pos is not None:
            dx = event.pos().x() - self.press_pos.x()
            dy = event.pos().y() - self.press_pos.y()
            self.press_pos = event.pos()
            
            azim = self.main_window.slider_azim.value() - dx * 0.5
            elev = self.main_window.slider_elev.value() + dy * 0.5
            
            azim = max(-180.0, min(180.0, azim))
            elev = max(-90.0, min(90.0, elev))
            
            self.main_window.updating_cam = True
            self.main_window.slider_azim.setValue(azim)
            self.main_window.slider_elev.setValue(elev)
            self.main_window.updating_cam = False
            self.main_window.update_all()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.press_pos = None
        super().mouseReleaseEvent(event)


# --------------------------------------------------------------------------
# Ventana Principal
# --------------------------------------------------------------------------
class OpenSimCompareWorker(QThread):
    """Corre el bridge de OpenSim en un HILO SEPARADO. Usa 'warm-start'
    (solve_osim_angles_warmstart): un solo refinamiento local partiendo de
    la solucion (elv_angle, shoulder_elv) de la actualizacion ANTERIOR, en
    vez del multi-start completo (32 arranques) -- mucho mas rapido para
    uso interactivo, donde la postura cambia gradualmente entre una
    llamada y la siguiente.

    NOTA sobre el GIL: mover esto a un QThread reduce pero NO elimina por
    completo el bloqueo de la UI, porque el interprete de Python (CPython)
    solo permite que un hilo a la vez ejecute bytecode -- si las llamadas
    de OpenSim (C++ via SWIG) no liberan el GIL durante calculos largos,
    el hilo principal se sigue viendo afectado mientras el worker corre.
    Reducir el costo del calculo en si (este warm-start) es el arreglo
    mas directo; si aun asi se siente lento, el siguiente paso seria
    mover el calculo a un PROCESO separado (multiprocessing), que si evita
    el GIL por completo.
    """
    result_ready = pyqtSignal(str, float, float)

    def __init__(self, osim_arm, sh_abd, sh_flex, elbow_flex, f_len, p_mass,
                 elv_guess, shoulder_elv_guess, parent=None):
        super().__init__(parent)
        self.osim_arm = osim_arm
        self.sh_abd = sh_abd
        self.sh_flex = sh_flex
        self.elbow_flex = elbow_flex
        self.f_len = f_len
        self.p_mass = p_mass
        self.elv_guess = elv_guess
        self.shoulder_elv_guess = shoulder_elv_guess

    def run(self):
        elv_angle, shoulder_elv = self.elv_guess, self.shoulder_elv_guess
        try:
            from opensim_bridge import (
                solve_osim_angles_warmstart, gravity_only_generalized_forces,
                project_shoulder_torque_to_urdf,
            )
            elv_angle, shoulder_elv, err_deg = solve_osim_angles_warmstart(
                self.osim_arm, self.sh_abd, self.sh_flex, self.elv_guess, self.shoulder_elv_guess
            )
            self.osim_arm.set_pose(elv_angle, shoulder_elv, shoulder_rot=0.0, elbow_flex=self.elbow_flex)

            forces = gravity_only_generalized_forces(
                self.osim_arm,
                coord_names=["elv_angle_r", "shoulder_elv_r", "elbow_flexion_r"],
                payload_mass=self.p_mass, payload_f_len=self.f_len,
            )
            tau_sh_abd_osim, tau_sh_flex_osim = project_shoulder_torque_to_urdf(
                self.osim_arm, self.sh_abd, self.sh_flex, elv_angle, shoulder_elv,
                forces["elv_angle_r"], forces["shoulder_elv_r"],
            )
            tau_elbow_osim = forces["elbow_flexion_r"]

            text = (f"<b>OpenSim (real, background):</b> "
                    f"Sh.Flex: {tau_sh_flex_osim:.2f} Nm | "
                    f"Sh.Abd: {tau_sh_abd_osim:.2f} Nm | "
                    f"Elbow: {tau_elbow_osim:.2f} Nm "
                    f"<font color='#6e7681'>(shoulder fit: {err_deg:.3f}°)</font>")
        except Exception as e:
            text = f"<b>OpenSim (real):</b> error: {e}"
        self.result_ready.emit(text, elv_angle, shoulder_elv)


class MuscleAnalysisWorker(QThread):
    """Corre Static Optimization (reparto de fuerzas musculares, Opcion 3)
    en un HILO SEPARADO para la postura ACTUAL, disparado por el boton
    'Analizar Musculos' -- no corre en tiempo real (es mas pesado que el
    torque neto), solo bajo demanda cuando el usuario lo pide.

    Calcula DOS escenarios: sin asistencia del exo (torque objetivo =
    torque total real de OpenSim) y con asistencia (torque objetivo =
    total menos lo que aporta el exo). La asistencia esta definida en la
    base de los sliders (sh_flex/sh_abd/elbow) -- para restarla
    correctamente de los objetivos en la base de OpenSim
    (elv_angle_r/shoulder_elv_r), se usa la Jacobiana INVERSA (la misma
    del Paso 8b, usada al reves): tau_osim = (J^-1)^T @ tau_urdf.
    """
    result_ready = pyqtSignal(dict, dict, dict, dict, float, float)
    # (result_no_exo, result_with_exo, target_no_exo, target_with_exo, elv_angle, shoulder_elv)
    error_ready = pyqtSignal(str)

    def __init__(self, osim_arm, sh_abd, sh_flex, elbow_flex, p_mass,
                 raw_sf_exo, raw_sa_exo, raw_el_exo, is_percent,
                 elv_guess, shoulder_elv_guess, parent=None):
        super().__init__(parent)
        self.osim_arm = osim_arm
        self.sh_abd = sh_abd
        self.sh_flex = sh_flex
        self.elbow_flex = elbow_flex
        self.p_mass = p_mass
        self.raw_sf_exo = raw_sf_exo
        self.raw_sa_exo = raw_sa_exo
        self.raw_el_exo = raw_el_exo
        self.is_percent = is_percent
        self.elv_guess = elv_guess
        self.shoulder_elv_guess = shoulder_elv_guess

    def run(self):
        try:
            import numpy as np
            from opensim_bridge import (
                solve_osim_angles_warmstart, gravity_only_generalized_forces,
                solve_muscle_activations, project_shoulder_torque_to_urdf,
                shoulder_angle_jacobian,
            )
            coords = ["elv_angle_r", "shoulder_elv_r", "elbow_flexion_r"]

            elv_angle, shoulder_elv, _ = solve_osim_angles_warmstart(
                self.osim_arm, self.sh_abd, self.sh_flex, self.elv_guess, self.shoulder_elv_guess
            )
            self.osim_arm.set_pose(elv_angle, shoulder_elv, shoulder_rot=0.0, elbow_flex=self.elbow_flex)

            forces = gravity_only_generalized_forces(
                self.osim_arm, coord_names=coords, payload_mass=self.p_mass,
            )
            target_no_exo = {c: forces[c] for c in coords}

            # --- Cuanto aporta el exo, en la base de OpenSim ---
            tau_sh_abd_osim, tau_sh_flex_osim = project_shoulder_torque_to_urdf(
                self.osim_arm, self.sh_abd, self.sh_flex, elv_angle, shoulder_elv,
                forces["elv_angle_r"], forces["shoulder_elv_r"],
            )
            if self.is_percent:
                exo_sf = tau_sh_flex_osim * (self.raw_sf_exo / 100.0)
                exo_sa = tau_sh_abd_osim * (self.raw_sa_exo / 100.0)
                exo_el = target_no_exo["elbow_flexion_r"] * (self.raw_el_exo / 100.0)
            else:
                exo_sf = self.raw_sf_exo
                exo_sa = self.raw_sa_exo
                exo_el = self.raw_el_exo

            J = shoulder_angle_jacobian(self.osim_arm, self.sh_abd, self.sh_flex, elv_angle, shoulder_elv)
            tau_urdf_reduction = np.array([exo_sa, exo_sf])  # orden (abd, flex), igual que project_shoulder_torque_to_urdf
            tau_osim_reduction = np.linalg.inv(J).T @ tau_urdf_reduction

            target_with_exo = dict(target_no_exo)
            target_with_exo["elv_angle_r"] = target_no_exo["elv_angle_r"] - float(tau_osim_reduction[0])
            target_with_exo["shoulder_elv_r"] = target_no_exo["shoulder_elv_r"] - float(tau_osim_reduction[1])
            target_with_exo["elbow_flexion_r"] = target_no_exo["elbow_flexion_r"] - exo_el

            result_no_exo = solve_muscle_activations(self.osim_arm, coord_names=coords, target_torques=target_no_exo)
            self.osim_arm.set_pose(elv_angle, shoulder_elv, shoulder_rot=0.0, elbow_flex=self.elbow_flex)
            result_with_exo = solve_muscle_activations(self.osim_arm, coord_names=coords, target_torques=target_with_exo)

            self.result_ready.emit(result_no_exo, result_with_exo, target_no_exo, target_with_exo, elv_angle, shoulder_elv)
        except Exception as e:
            import traceback
            self.error_ready.emit(f"{e}\n{traceback.format_exc()}")


MUSCLE_FULL_NAMES = {
    # Shoulder
    "DELT1": "Deltoid Anterior", "DELT2": "Deltoid Middle", "DELT3": "Deltoid Posterior",
    "SUPSP": "Supraspinatus", "INFSP": "Infraspinatus", "SUBSC": "Subscapularis",
    "TMIN": "Teres Minor", "TMAJ": "Teres Major",
    "PECM1": "Pectoralis Major (clavicular)", "PECM2": "Pectoralis Major (sternal)", "PECM3": "Pectoralis Major (ribs)",
    "LAT1": "Latissimus Dorsi (thoracic)", "LAT2": "Latissimus Dorsi (lumbar)", "LAT3": "Latissimus Dorsi (iliac)",
    "CORB": "Coracobrachialis",
    # Elbow
    "TRIlong": "Triceps (long head)", "TRIlat": "Triceps (lateral head)", "TRImed": "Triceps (medial head)",
    "ANC": "Anconeus", "SUP": "Supinator",
    "BIClong": "Biceps (long head)", "BICshort": "Biceps (short head)",
    "BRA": "Brachialis", "BRD": "Brachioradialis",
    # Forearm / wrist
    "ECRL": "Extensor Carpi Radialis Longus", "ECRB": "Extensor Carpi Radialis Brevis", "ECU": "Extensor Carpi Ulnaris",
    "FCR": "Flexor Carpi Radialis", "FCU": "Flexor Carpi Ulnaris", "PL": "Palmaris Longus",
    "PT": "Pronator Teres", "PQ": "Pronator Quadratus",
    "FDSL": "Flexor Digitorum Superficialis (little)", "FDSR": "Flexor Digitorum Superficialis (ring)",
    "FDSM": "Flexor Digitorum Superficialis (middle)", "FDSI": "Flexor Digitorum Superficialis (index)",
    "FDPL": "Flexor Digitorum Profundus (little)", "FDPR": "Flexor Digitorum Profundus (ring)",
    "FDPM": "Flexor Digitorum Profundus (middle)", "FDPI": "Flexor Digitorum Profundus (index)",
    "EDCL": "Extensor Digitorum Communis (little)", "EDCR": "Extensor Digitorum Communis (ring)",
    "EDCM": "Extensor Digitorum Communis (middle)", "EDCI": "Extensor Digitorum Communis (index)",
    "EDM": "Extensor Digiti Minimi", "EIP": "Extensor Indicis Proprius",
    "EPL": "Extensor Pollicis Longus", "EPB": "Extensor Pollicis Brevis",
    "FPL": "Flexor Pollicis Longus", "APL": "Abductor Pollicis Longus",
}


def muscle_display_name(muscle_name):
    """'DELT2_r' -> 'Deltoid Middle (DELT2_r)'; if the name isn't in the
    dictionary, the original code is shown as-is."""
    base = muscle_name.rsplit("_", 1)[0]
    full = MUSCLE_FULL_NAMES.get(base)
    return f"{full} ({muscle_name})" if full else muscle_name


COORD_DISPLAY_NAMES = {
    "elv_angle_r": "Shoulder Plane",
    "shoulder_elv_r": "Shoulder Elevation",
    "shoulder_rot_r": "Shoulder Rotation",
    "elbow_flexion_r": "Elbow Flexion",
}


def coord_display_name(coord_name):
    """'elv_angle_r' -> 'Shoulder Plane (elv_angle_r)'."""
    friendly = COORD_DISPLAY_NAMES.get(coord_name)
    return f"{friendly} ({coord_name})" if friendly else coord_name


class BoneMeshLoadWorker(QThread):
    """Carga los archivos .vtp de malla (I/O + parseo, en un hilo separado
    para no trabar la UI la primera vez que se abre la vista realista).
    La creacion de actores de PyVista en si ocurre en el hilo principal
    (VTK/Qt no son thread-safe para eso)."""
    meshes_ready = pyqtSignal(list)  # [(body, file, pv.PolyData, scale, local_transform), ...]
    error_ready = pyqtSignal(str)

    def __init__(self, osim_path, body_meshes, side="r", parent=None):
        super().__init__(parent)
        self.osim_path = osim_path
        self.body_meshes = body_meshes
        self.side = side

    def run(self):
        try:
            import pyvista as pv
            from bone_viewer import mesh_local_transform_matrix
            geometry_dir = os.path.join(os.path.dirname(os.path.abspath(self.osim_path)), "Geometry")
            suffix = f"_{self.side}"
            cache = {}
            loaded = []
            for body, meshes in self.body_meshes.items():
                if not (body.endswith(suffix) or body == "thorax"):
                    continue
                for m in meshes:
                    fpath = os.path.join(geometry_dir, m["file"])
                    if m["file"] not in cache:
                        cache[m["file"]] = pv.read(fpath)
                    local_T = mesh_local_transform_matrix(m)
                    loaded.append((body, m["file"], cache[m["file"]], m["scale"], local_T))
            self.meshes_ready.emit(loaded)
        except Exception as e:
            import traceback
            self.error_ready.emit(f"{e}\n{traceback.format_exc()}")


class BoneTransformWorker(QThread):
    """Resuelve la pose actual (warm-start) y calcula la transformacion
    mundial de cada Body (sin recalcular mallas), en un hilo separado."""
    transforms_ready = pyqtSignal(dict, float, float)  # ({body: 4x4 np.array}, elv_angle, shoulder_elv)
    error_ready = pyqtSignal(str)

    def __init__(self, osim_arm, sh_abd, sh_flex, elv_guess, shoulder_elv_guess, side="r", parent=None):
        super().__init__(parent)
        self.osim_arm = osim_arm
        self.sh_abd = sh_abd
        self.sh_flex = sh_flex
        self.elv_guess = elv_guess
        self.shoulder_elv_guess = shoulder_elv_guess
        self.side = side

    def run(self):
        try:
            import numpy as np
            from opensim_bridge import solve_osim_angles

            # IMPORTANTE: se usa la version ROBUSTA (multi-start, 32
            # arranques), no la version 'warmstart' (un solo refinamiento
            # local). La vista de huesos se actualiza bajo demanda (no en
            # cada frame), asi que podemos permitirnos unos segundos extra
            # a cambio de evitar quedar atrapados en un minimo local
            # incorrecto (el mismo problema documentado en la Seccion 6
            # del tutorial) -- eso fue lo que causaba que sh_flex se viera
            # como abduccion: el solver rapido convergia a una postura
            # distinta a la solicitada.
            elv_angle, shoulder_elv, err_deg = solve_osim_angles(
                self.osim_arm, self.sh_abd, self.sh_flex
            )
            if err_deg > 1.0:
                print(f"[Bone viewer] Advertencia: ajuste de hombro alto ({err_deg:.2f} deg)")
            self.osim_arm.set_pose(elv_angle, shoulder_elv, shoulder_rot=0.0)
            # Evaluar el ritmo escapulohumeral UNA vez, ya con la pose final
            # (no durante la busqueda del solver -- ver opensim_bridge.py
            # finalize_constraints() para el porque).
            self.osim_arm.finalize_constraints()

            model = self.osim_arm.model
            state = self.osim_arm.state

            # DIAGNOSTICO: valores reales de las coordenadas de la escapula,
            # para confirmar si el ritmo escapulohumeral (CoordinateCouplerConstraint,
            # Seccion 4.1 del tutorial) se esta aplicando correctamente al
            # cambiar elv_angle_r/shoulder_elv_r, o si quedan "pegadas".
            try:
                cset = model.getCoordinateSet()
                scap_coords = ["unrotscap_r2_r", "unrotscap_r3_r", "acromioclavicular_r1_r",
                               "acromioclavicular_r2_r", "acromioclavicular_r3_r"]
                vals = [math.degrees(cset.get(c).getValue(state)) for c in scap_coords]
                print(f"[Bone viewer] elv={math.degrees(elv_angle):.1f} she={math.degrees(shoulder_elv):.1f} | "
                      f"escapula: " + ", ".join(f"{c}={v:.1f}" for c, v in zip(scap_coords, vals)))
            except Exception as e:
                print(f"[Bone viewer] No se pudo leer coordenadas de escapula: {e}")
            suffix = f"_{self.side}"
            body_transforms = {}
            body_set = model.getBodySet()
            for i in range(body_set.getSize()):
                body = body_set.get(i)
                name = body.getName()
                if not (name.endswith(suffix) or name == "thorax"):
                    continue
                T = body.getTransformInGround(state)
                R = T.R()
                p = T.p()
                M = np.eye(4)
                for r in range(3):
                    for c in range(3):
                        M[r, c] = R.get(r, c)
                    M[r, 3] = p.get(r)
                body_transforms[name] = M

            self.transforms_ready.emit(body_transforms, elv_angle, shoulder_elv)
        except Exception as e:
            import traceback
            self.error_ready.emit(f"{e}\n{traceback.format_exc()}")


class MainWindow(QMainWindow):
    def __init__(self, urdf_path):
        super().__init__()
        self.setWindowTitle(f"CAD Kinematics & Exoskeleton Simulator — {os.path.basename(urdf_path)} | Dr.-Ing. Enrique Bances")

        # Tamaño inicial calculado a partir del área disponible de la pantalla
        # (no un valor fijo como 1920x1000), y ventana centrada. Esto evita
        # que en pantallas más pequeñas la ventana se abra más grande que el
        # escritorio, empujando la barra de título fuera del área visible.
        screen = QApplication.primaryScreen()
        avail = screen.availableGeometry() if screen else None
        if avail is not None:
            target_w = min(1920, int(avail.width() * 0.92))
            target_h = min(1000, int(avail.height() * 0.90))
            self.resize(target_w, target_h)
            self.move(avail.x() + (avail.width() - target_w) // 2,
                      avail.y() + (avail.height() - target_h) // 2)
        else:
            self.resize(1280, 800)
        
        self.setDockNestingEnabled(True)

        self.setStyleSheet("""
            QMainWindow { background-color: #080808; }
            QWidget { background-color: #080808; color: #c9d1d9; }
            QGroupBox {
                color: #f0f6fc; font-weight: bold; border: 1px solid #262626;
                border-radius: 6px; margin-top: 6px; padding: 6px; background-color: #121212;
                font-size: 12px;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; color: #58a6ff; background-color: #121212; }
            QLabel { color: #c9d1d9; font-size: 11px; background: transparent; }
            QPushButton {
                background-color: #121212; color: #c9d1d9; border: 1px solid #262626;
                border-radius: 4px; padding: 5px 10px; font-weight: bold; font-size: 11px;
            }
            QPushButton:hover { background-color: #1f1f1f; color: #ffffff; border-color: #444444; }
            QPushButton:pressed { background-color: #1f6feb; border-color: #58a6ff; }
            QDockWidget {
                color: #f0f6fc; font-weight: bold; font-size: 12px;
                background-color: #080808;
            }
            QDockWidget::title {
                background: #121212; padding: 6px; border: 1px solid #262626; border-radius: 4px;
            }
            QTabWidget::pane { border: 1px solid #262626; background: #080808; border-radius: 4px; }
            QTabBar::tab { background: #121212; color: #8b949e; padding: 6px 8px; font-weight: bold; font-size: 11px; border: 1px solid #262626; }
            QTabBar::tab:selected { background: #1f6feb; color: #ffffff; border-color: #58a6ff; }
            QComboBox { background-color: #121212; color: #c9d1d9; border: 1px solid #262626; border-radius: 4px; padding: 2px 5px; font-size: 11px; }
            QScrollArea { border: none; background-color: #080808; }
        """)

        self.base_links, self.base_joints, self.root_link = parse_urdf(urdf_path)
        self.hand_trail = []

        # ================= BARRA DE HERRAMIENTAS SUPERIOR (TOOLBAR) =================
        toolbar = QToolBar("Main Controls")
        toolbar.setMovable(False)
        toolbar.setStyleSheet("QToolBar { background: #121212; border-bottom: 1px solid #262626; padding: 4px; spacing: 8px; }")
        self.addToolBar(toolbar)

        self.btn_reset_angles = QPushButton("🔄 Reset Joint Angles")
        self.btn_reset_angles.clicked.connect(self.reset_angles)
        toolbar.addWidget(self.btn_reset_angles)

        toolbar.addSeparator()

        self.btn_anim_play = QPushButton("▶ Play Animation")
        self.btn_anim_play.setCheckable(True)
        self.btn_anim_play.setStyleSheet("background-color: #238636; color: #ffffff; border: none; font-weight: bold;")
        self.btn_anim_play.clicked.connect(self.toggle_animation)
        toolbar.addWidget(self.btn_anim_play)

        self.btn_anim_reset = QPushButton("⏹ Stop")
        self.btn_anim_reset.clicked.connect(self.stop_animation)
        toolbar.addWidget(self.btn_anim_reset)

        self.combo_anim_mode = QComboBox()
        self.combo_anim_mode.addItems(["Both (Shoulder + Elbow)", "Shoulder only (Flex+Abd)", "Elbow only"])
        toolbar.addWidget(QLabel(" Mode:"))
        toolbar.addWidget(self.combo_anim_mode)

        toolbar.addSeparator()

        self.btn_top = QPushButton(" Top")
        self.btn_side = QPushButton(" Side")
        self.btn_front = QPushButton(" Front")
        self.btn_def_view = QPushButton(" Default View")
        for btn in (self.btn_top, self.btn_side, self.btn_front, self.btn_def_view):
            toolbar.addWidget(btn)

        toolbar.addSeparator()

        self.btn_clear_trail = QPushButton("🧹 Clear Trail")
        self.btn_clear_trail.setStyleSheet("color: #d29922;")
        self.btn_clear_trail.clicked.connect(self.clear_hand_trail)
        toolbar.addWidget(self.btn_clear_trail)

        # Toggle para mostrar/ocultar la leyenda del visor 3D -- por
        # defecto OCULTA, ya que tapaba buena parte de la vista.
        self.show_3d_legend = False
        self.btn_toggle_3d_legend = QPushButton("🏷️ Legend: Off")
        self.btn_toggle_3d_legend.setCheckable(True)
        self.btn_toggle_3d_legend.setChecked(False)
        self.btn_toggle_3d_legend.setStyleSheet("color: #8b949e;")
        self.btn_toggle_3d_legend.clicked.connect(self._toggle_3d_legend)
        toolbar.addWidget(self.btn_toggle_3d_legend)

        # Toggle para mostrar/ocultar las triadas de ejes IMU (X/Y/Z en
        # torso/brazo/antebrazo) -- por defecto VISIBLES (comportamiento
        # previo sin cambios), solo se agrega la posibilidad de ocultarlas.
        self.show_imu_axes = True
        self.btn_toggle_imu_axes = QPushButton("📡 IMU Axes: On")
        self.btn_toggle_imu_axes.setCheckable(True)
        self.btn_toggle_imu_axes.setChecked(True)
        self.btn_toggle_imu_axes.setStyleSheet("color: #8b949e;")
        self.btn_toggle_imu_axes.clicked.connect(self._toggle_imu_axes)
        toolbar.addWidget(self.btn_toggle_imu_axes)

        # Toggle para mostrar/ocultar las etiquetas de texto junto a cada
        # articulacion (ej. "Shoulder", "Elbow") -- por defecto VISIBLES.
        self.show_joint_labels = True
        self.btn_toggle_joint_labels = QPushButton("🔤 Joint Labels: On")
        self.btn_toggle_joint_labels.setCheckable(True)
        self.btn_toggle_joint_labels.setChecked(True)
        self.btn_toggle_joint_labels.setStyleSheet("color: #8b949e;")
        self.btn_toggle_joint_labels.clicked.connect(self._toggle_joint_labels)
        toolbar.addWidget(self.btn_toggle_joint_labels)

        toolbar.addSeparator()

        # Boton para correr Static Optimization (Opcion 3: fuerzas
        # musculares individuales) BAJO DEMANDA -- no en tiempo real, es
        # mas pesado. Corre en un QThread separado (MuscleAnalysisWorker).
        self.btn_analyze_muscles = QPushButton("🔬 Analyze Muscles")
        self.btn_analyze_muscles.setStyleSheet("color: #bc8cff; font-weight: bold;")
        self.btn_analyze_muscles.clicked.connect(self.run_muscle_analysis)
        toolbar.addWidget(self.btn_analyze_muscles)
        self.muscle_worker = None

        # ---- Controles de ventana propios (por si el gestor de ventanas no
        # muestra la barra de título nativa: minimizar / maximizar / pantalla
        # completa / cerrar quedan siempre accesibles desde aquí) ----
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        toolbar.addWidget(spacer)
        toolbar.addSeparator()

        self.btn_win_min = QPushButton("🗕")
        self.btn_win_min.setToolTip("Minimize")
        self.btn_win_min.setFixedWidth(34)
        self.btn_win_min.clicked.connect(self.showMinimized)
        toolbar.addWidget(self.btn_win_min)

        self.btn_win_max = QPushButton("🗖")
        self.btn_win_max.setToolTip("Maximize / Restore")
        self.btn_win_max.setFixedWidth(34)
        self.btn_win_max.clicked.connect(self.toggle_maximize_restore)
        toolbar.addWidget(self.btn_win_max)

        self.btn_win_fullscreen = QPushButton("⛶")
        self.btn_win_fullscreen.setToolTip("Full Screen (F11)")
        self.btn_win_fullscreen.setCheckable(True)
        self.btn_win_fullscreen.setFixedWidth(34)
        self.btn_win_fullscreen.clicked.connect(self.toggle_fullscreen)
        toolbar.addWidget(self.btn_win_fullscreen)

        self.btn_win_close = QPushButton("✕")
        self.btn_win_close.setToolTip("Close (Ctrl+Q)")
        self.btn_win_close.setFixedWidth(34)
        self.btn_win_close.setStyleSheet(
            "QPushButton { background-color: #3b1214; color: #f85149; border: 1px solid #5a1a1d; font-weight: bold; }"
            "QPushButton:hover { background-color: #f85149; color: #ffffff; }"
        )
        self.btn_win_close.clicked.connect(self.close)
        toolbar.addWidget(self.btn_win_close)

        QShortcut(QKeySequence("F11"), self, activated=self.toggle_fullscreen)
        QShortcut(QKeySequence("Ctrl+Q"), self, activated=self.close)
        QShortcut(QKeySequence("Escape"), self, activated=self.exit_fullscreen_only)

        # ================= PANEL IZQUIERDO =================
        left_dock = QDockWidget(" Control & Configuration Panel", self)
        left_dock.setAllowedAreas(Qt.DockWidgetArea.LeftDockWidgetArea | Qt.DockWidgetArea.RightDockWidgetArea)
        left_dock.setMinimumWidth(380)

        tab_left = QTabWidget()
        tab_left.setMinimumWidth(360)

        # --- TAB 1: Kinematics & Animation ---
        tab_kinematics = QWidget()
        layout_tab_kin = QVBoxLayout(tab_kinematics)
        
        box_joints = QGroupBox("Joint Angles (Rad / Deg)")
        box_joints_layout = QVBoxLayout(box_joints)
        self.sliders = {}
        joint_colors = {"shoulder_flex": "#f85149", "shoulder_abd": "#58a6ff", "elbow_flex": "#3fb950"}

        for j in self.base_joints:
            if j["type"] not in ("revolute", "continuous"):
                continue
            lo_deg = math.degrees(j["lower"])
            hi_deg = math.degrees(j["upper"])
            default_deg = max(lo_deg, min(hi_deg, 0.0))
            color = joint_colors.get(j["name"], "#58a6ff")
            s = LabeledSlider(j["name"], lo_deg, hi_deg, default_deg, accent_color=color)
            s.slider.valueChanged.connect(self.update_all)
            s.spin.valueChanged.connect(self.update_all)
            box_joints_layout.addWidget(s)
            self.sliders[j["name"]] = s

        layout_tab_kin.addWidget(box_joints)

        box_anim_cfg = QGroupBox("Animation Parameters")
        box_anim_cfg_layout = QVBoxLayout(box_anim_cfg)
        self.slider_anim_speed = LabeledSlider("Anim Speed", 0.1, 3.0, 2.0, suffix="x", decimals=2, accent_color="#d29922")
        box_anim_cfg_layout.addWidget(self.slider_anim_speed)
        self.lbl_anim_status = QLabel("Status: Stopped")
        self.lbl_anim_status.setStyleSheet("color: #8b949e; font-style: italic;")
        box_anim_cfg_layout.addWidget(self.lbl_anim_status)
        layout_tab_kin.addWidget(box_anim_cfg)

        base_dir = os.path.dirname(os.path.abspath(__file__))
        assets_dir = os.path.join(base_dir, "assets")
        
        img_path = None
        for ext in [".png", ".jpg", ".jpeg"]:
            possible_path = os.path.join(assets_dir, f"arm_reference{ext}")
            if os.path.isfile(possible_path):
                img_path = possible_path
                break

        if img_path:
            box_img = QGroupBox("IMU Placement Reference")
            box_img_layout = QVBoxLayout(box_img)
            box_img_layout.setContentsMargins(4, 4, 4, 4)
            
            lbl_img = QLabel()
            pixmap = QPixmap(img_path)
            if not pixmap.isNull():
                # Imagen panoramica (~1.75:1) con texto pequeno (theta, X/Y/Z):
                # se muestra a todo el ancho del panel izquierdo para que sea
                # legible, en vez de un recuadro chico con espacio desperdiciado
                # al costado (por eso el layout es vertical, no horizontal).
                lbl_img.setPixmap(pixmap.scaled(340, 260, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
                lbl_img.setAlignment(Qt.AlignmentFlag.AlignCenter)
                box_img_layout.addWidget(lbl_img)
                layout_tab_kin.addWidget(box_img)

        layout_tab_kin.addStretch()

        # --- TAB 2: Physical Parameters & Exo ---
        tab_physics = QWidget()
        layout_tab_phys = QVBoxLayout(tab_physics)

        box_unit = QGroupBox("Exoskeleton Mode")
        unit_layout = QHBoxLayout(box_unit)
        unit_layout.addWidget(QLabel("Mode (% / Nm):"))
        self.switch_exo_unit = ToggleSwitch()
        self.switch_exo_unit.stateChanged.connect(self.on_exo_unit_changed)
        self.lbl_unit_mode = QLabel("<b>Percentage (%)</b>")
        self.lbl_unit_mode.setStyleSheet("color: #58a6ff;")
        unit_layout.addWidget(self.switch_exo_unit)
        unit_layout.addWidget(self.lbl_unit_mode)
        unit_layout.addStretch()
        layout_tab_phys.addWidget(box_unit)

        box_physics_sliders = QGroupBox("Arm Geometry & Exoskeleton Assistance")
        box_physics_layout = QVBoxLayout(box_physics_sliders)

        self.phys_sliders = {
            # Payload primero (el mas usado/ajustado con frecuencia),
            # despues asistencia del exo, despues masas/longitudes.
            "payload_mass": LabeledSlider("Payload Mass", 0.0, 10.0, 3.0, suffix=" kg", decimals=1, accent_color="#d29922"),
            "shoulder_flex_exo": LabeledSlider("Sh. Flex Exo Assist", 0.0, 100.0, 0.0, suffix=" %", decimals=1, accent_color="#f85149"),
            "shoulder_abd_exo": LabeledSlider("Sh. Abd Exo Assist", 0.0, 100.0, 0.0, suffix=" %", decimals=1, accent_color="#58a6ff"),
            "elbow_exo": LabeledSlider("Elbow Exo Assist", 0.0, 100.0, 0.0, suffix=" %", decimals=1, accent_color="#3fb950"),
            "upper_arm_len": LabeledSlider("Upper Arm Len", 0.2, 0.5, 0.3, suffix=" m", decimals=2, accent_color="#8b949e"),
            "forearm_len": LabeledSlider("Forearm Len", 0.15, 0.4, 0.27, suffix=" m", decimals=2, accent_color="#8b949e"),
            "upper_arm_mass": LabeledSlider("Upper Arm Mass", 0.5, 5.0, 2.0, suffix=" kg", decimals=1, accent_color="#8b949e"),
            "forearm_mass": LabeledSlider("Forearm Mass", 0.5, 4.0, 1.5, suffix=" kg", decimals=1, accent_color="#8b949e"),
        }
        for key, s in self.phys_sliders.items():
            s.slider.valueChanged.connect(self.update_all)
            s.spin.valueChanged.connect(self.update_all)
            box_physics_layout.addWidget(s)

        layout_tab_phys.addWidget(box_physics_sliders)
        layout_tab_phys.addStretch()

        # --- TAB 3: IMU Sensors ---
        tab_imus = QWidget()
        layout_tab_imus = QVBoxLayout(tab_imus)
        self.imu_sliders = {}

        for imu_name in ["IMU_Torso", "IMU_UpperArm", "IMU_Forearm"]:
            panel = CollapsiblePanel(imu_name)
            panel_layout = QVBoxLayout()
            
            s_roll = LabeledSlider(f"Roll", -180.0, 180.0, 0.0, suffix=" °", decimals=1, accent_color="#8b949e")
            s_pitch = LabeledSlider(f"Pitch", -180.0, 180.0, 0.0, suffix=" °", decimals=1, accent_color="#8b949e")
            s_yaw = LabeledSlider(f"Yaw", -180.0, 180.0, 0.0, suffix=" °", decimals=1, accent_color="#8b949e")
            
            for s in (s_roll, s_pitch, s_yaw):
                s.slider.valueChanged.connect(self.update_all)
                s.spin.valueChanged.connect(self.update_all)
                panel_layout.addWidget(s)

            panel.setContentLayout(panel_layout)
            self.imu_sliders[imu_name] = {"roll": s_roll, "pitch": s_pitch, "yaw": s_yaw}
            layout_tab_imus.addWidget(panel)

        layout_tab_imus.addStretch()

        # --- TAB 4: Mathematical Equations (LaTeX Matplotlib con fuente más grande y figura debajo exacta) ---
        tab_equations = QWidget()
        layout_tab_eq = QVBoxLayout(tab_equations)
        layout_tab_eq.setContentsMargins(6, 6, 6, 6)
        layout_tab_eq.setSpacing(6)
        
        box_eq = QGroupBox("Mathematical Foundations (IMUs, Quaternions & Torques DOF)")
        box_eq_layout = QVBoxLayout(box_eq)
        box_eq_layout.setContentsMargins(4, 4, 4, 4)
        
        self.fig_eq = Figure(figsize=(5.0, 8.2), facecolor="#121212")
        self.canvas_eq = FigureCanvasQTAgg(self.fig_eq)
        box_eq_layout.addWidget(self.canvas_eq)
        
        layout_tab_eq.addWidget(box_eq)

        tab_left.addTab(tab_kinematics, "🤖 Kinematics")
        tab_left.addTab(tab_physics, "⚙️ Physics & Exo")
        tab_left.addTab(tab_imus, "📡 IMU Sensors")
        tab_left.addTab(tab_equations, "📐 Equations")

        scroll_left = QScrollArea()
        scroll_left.setWidgetResizable(True)
        scroll_left.setWidget(tab_left)

        left_dock.setWidget(scroll_left)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, left_dock)

        # ================= CENTER PANEL (3D VISOR CENTRAL) =================
        center_container = QWidget()
        center_layout = QVBoxLayout(center_container)
        center_layout.setContentsMargins(4, 4, 4, 4)

        center_box = QGroupBox("3D CAD Visor & Trajectory Trail (Mouse Orbiting & Scroll Zoom)")
        box_layout = QVBoxLayout(center_box)
        box_layout.setContentsMargins(4, 4, 4, 4)

        self.fig = Figure(figsize=(6, 4.2), facecolor="#000000")
        self.canvas = InteractiveCanvas3D(self.fig, self)
        self.canvas.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.ax = self.fig.add_subplot(111, projection="3d")
        
        box_layout.addWidget(self.canvas)

        cam_layout = QHBoxLayout()
        self.slider_azim = LabeledSlider("Azim", -180.0, 180.0, 120.0, suffix="°", decimals=0, accent_color="#8b949e")
        self.slider_elev = LabeledSlider("Elev", -90.0, 90.0, 15.0, suffix="°", decimals=0, accent_color="#8b949e")
        self.slider_zoom = LabeledSlider("Zoom", 0.1, 3.0, 1.0, suffix="x", decimals=2, accent_color="#8b949e")
        for s in (self.slider_azim, self.slider_elev, self.slider_zoom):
            s.slider.valueChanged.connect(self.on_camera_slider_changed)
            s.spin.valueChanged.connect(self.on_camera_slider_changed)
            cam_layout.addWidget(s)
        box_layout.addLayout(cam_layout)

        # --- Pestana "Realistic View" (Opcion B, ver tutorial Seccion 24):
        # huesos reales de OpenSim, cargados/actualizados BAJO DEMANDA (no
        # en tiempo real -- ver la saga de rendimiento de la Seccion 16),
        # sin tocar ni arriesgar la vista esquematica actual.
        realistic_tab = QWidget()
        realistic_layout = QVBoxLayout(realistic_tab)

        self.pv_plotter = None
        self.pv_actors = {}
        self.bone_meshes_loaded = False
        self.bone_load_worker = None
        self.bone_transform_worker = None
        self.last_body_transforms = {}
        self.torso_fix_deg = 0.0

        if PYVISTA_AVAILABLE:
            self.pv_interactor = QtInteractor(realistic_tab)
            self.pv_interactor.set_background("#0d1117")
            realistic_layout.addWidget(self.pv_interactor)
            self.pv_plotter = self.pv_interactor
        else:
            lbl_no_pv = QLabel("PyVista is not installed. Run: pip install pyvista pyvistaqt")
            lbl_no_pv.setStyleSheet("color: #f85149; padding: 20px;")
            realistic_layout.addWidget(lbl_no_pv)

        bone_btn_layout = QHBoxLayout()
        self.btn_load_bones = QPushButton("🦴 Load Skeleton")
        self.btn_load_bones.setStyleSheet("color: #d29922; font-weight: bold;")
        self.btn_load_bones.clicked.connect(self.load_or_update_skeleton)
        self.btn_load_bones.setEnabled(PYVISTA_AVAILABLE)
        bone_btn_layout.addWidget(self.btn_load_bones)

        # Correccion de rotacion SOLO para la malla del torso (thorax.vtp)
        # -- el brazo ya esta validado y correcto (ver test_flex_vs_abd.py);
        # el torso parece estar mal orientado en el propio archivo .vtp o
        # en como se interpreta su joint base. En vez de seguir adivinando
        # el valor correcto sin poder verlo, se deja como control visual
        # ajustable: rota hasta que el torso se vea alineado con el brazo
        # y la flecha roja de referencia.
        bone_btn_layout.addWidget(QLabel("Torso fix (Y°):"))
        self.spin_torso_fix = QDoubleSpinBox()
        self.spin_torso_fix.setRange(-180.0, 180.0)
        self.spin_torso_fix.setSingleStep(15.0)
        self.spin_torso_fix.setValue(0.0)
        self.spin_torso_fix.valueChanged.connect(self._on_torso_fix_changed)
        bone_btn_layout.addWidget(self.spin_torso_fix)

        self.chk_show_scapula = QCheckBox("Show scapula/clavicle")
        self.chk_show_scapula.setChecked(True)
        self.chk_show_scapula.stateChanged.connect(self._on_toggle_scapula_visibility)
        bone_btn_layout.addWidget(self.chk_show_scapula)

        self.lbl_bone_status = QLabel("Not loaded yet. Click 'Load Skeleton' to load the real OpenSim bone meshes.")
        self.lbl_bone_status.setStyleSheet("color: #8b949e; padding: 4px;")
        bone_btn_layout.addWidget(self.lbl_bone_status)
        realistic_layout.addLayout(bone_btn_layout)

        center_tabs = QTabWidget()
        center_tabs.addTab(center_box, "📐 Schematic View")
        center_tabs.addTab(realistic_tab, "🦴 Realistic View (OpenSim)")
        center_layout.addWidget(center_tabs)
        self.setCentralWidget(center_container)

        # ================= PANEL INFERIOR (ancho completo) =================
        # Movido de la derecha (columna angosta) a abajo (ancho completo de
        # la ventana) porque las graficas de torque son la parte mas
        # importante de la simulacion y necesitan mucho mas espacio /
        # fuentes mas grandes para ser legibles.
        right_dock = QDockWidget(" Real-Time Analytics & Fatigue Monitor", self)
        right_dock.setAllowedAreas(Qt.DockWidgetArea.BottomDockWidgetArea | Qt.DockWidgetArea.RightDockWidgetArea)
        right_dock.setMinimumHeight(420)

        right_container = QWidget()
        right_layout = QVBoxLayout(right_container)
        right_layout.setContentsMargins(2, 2, 2, 2)

        box_metrics = QGroupBox("Joint Status & Ergonomic Fatigue Alerts")
        box_metrics_layout = QVBoxLayout(box_metrics)

        # Boton para colapsar/expandir el panel de metricas -- libera
        # espacio vertical para las graficas (que son lo mas importante),
        # reduciendo la necesidad de scroll.
        self.btn_toggle_metrics = QPushButton("▼ Hide metrics (more space for graphs)")
        self.btn_toggle_metrics.setCheckable(True)
        self.btn_toggle_metrics.setChecked(False)
        self.btn_toggle_metrics.setStyleSheet("color: #8b949e; text-align: left; padding: 4px;")
        self.btn_toggle_metrics.clicked.connect(self._toggle_metrics_panel)

        self.lbl_angles = QLabel("<b>Angles:</b> <font color='#f85149'>Sh_Flex: 0.0°</font> | <font color='#58a6ff'>Sh_Abd: 0.0°</font> | <font color='#3fb950'>Elbow: 0.0°</font>")
        
        self.lbl_t_sh_flex = QLabel("<b>Sh. Flexion:</b> Total: 0.00 Nm | Human: 0.00 Nm | Exo: 0.00 Nm")
        self.lbl_t_sh_abd = QLabel("<b>Sh. Abduction:</b> Total: 0.00 Nm | Human: 0.00 Nm | Exo: 0.00 Nm")
        self.lbl_t_elbow = QLabel("<b>Elbow:</b> Total: 0.00 Nm | Human: 0.00 Nm | Exo: 0.00 Nm")
        self.lbl_osim_compare = QLabel("<b>OpenSim (real):</b> loading model...")
        self.lbl_osim_compare.setStyleSheet("color: #d29922; font-size: 11px; background: #1a1400; padding: 4px; border-radius: 4px; border: 1px solid #4d3c00;")

        for lbl in (self.lbl_angles,):
            lbl.setStyleSheet("color: #58a6ff; font-size: 11px; background: #080808; padding: 4px; border-radius: 4px; border: 1px solid #262626;")

        self.metrics_content_widget = QWidget()
        metrics_content_layout = QVBoxLayout(self.metrics_content_widget)
        metrics_content_layout.setContentsMargins(0, 0, 0, 0)
        metrics_content_layout.addWidget(self.lbl_angles)
        metrics_content_layout.addWidget(self.lbl_t_sh_flex)
        metrics_content_layout.addWidget(self.lbl_t_sh_abd)
        metrics_content_layout.addWidget(self.lbl_t_elbow)
        metrics_content_layout.addWidget(self.lbl_osim_compare)

        box_metrics_layout.addWidget(self.btn_toggle_metrics)
        box_metrics_layout.addWidget(self.metrics_content_widget)

        box_plot = QGroupBox("Torque vs Angle Curves & Human Relief Shading")
        box_plot_layout = QVBoxLayout(box_plot)

        # Figura mucho mas ancha (para el layout horizontal a todo el ancho)
        # y los 3 subplots ahora van LADO A LADO (131/132/133) en vez de
        # apilados, ya que el panel es ancho y bajo, no angosto y alto.
        self.fig_torque = Figure(figsize=(18, 5.2), facecolor="#121212")
        self.canvas_torque = FigureCanvasQTAgg(self.fig_torque)
        self.canvas_torque.setMinimumHeight(380)

        self.ax_sh_flex = self.fig_torque.add_subplot(131)
        self.ax_sh_abd = self.fig_torque.add_subplot(132)
        self.ax_elbow = self.fig_torque.add_subplot(133)
        box_plot_layout.addWidget(self.canvas_torque)
        
        right_layout.addWidget(box_metrics)
        right_layout.addWidget(box_plot)

        scroll_right = QScrollArea()
        scroll_right.setWidgetResizable(True)
        scroll_right.setWidget(right_container)

        right_dock.setWidget(scroll_right)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, right_dock)

        self.btn_top.clicked.connect(lambda: self.set_view(90, -90))
        self.btn_top.clicked.connect(lambda: self._set_pv_camera("top"))
        self.btn_side.clicked.connect(lambda: self.set_view(0, 0))
        self.btn_side.clicked.connect(lambda: self._set_pv_camera("side"))
        self.btn_front.clicked.connect(lambda: self.set_view(0, 90))
        self.btn_front.clicked.connect(lambda: self._set_pv_camera("front"))
        self.btn_def_view.clicked.connect(lambda: self.set_view(15, 120))
        self.btn_def_view.clicked.connect(lambda: self._set_pv_camera("iso"))

        self.updating_cam = False

        self.anim_timer = QTimer(self)
        self.anim_timer.timeout.connect(self._animation_step)
        self.anim_t = 0.0

        # Timer dedicado para que el halo/brillo de asistencia "lata" en
        # cuanto se activa el exoesqueleto, sin necesitar que el usuario
        # presione Play en la animación de las articulaciones.
        self.exo_pulse_timer = QTimer(self)
        self.exo_pulse_timer.timeout.connect(self._exo_pulse_step)

        # --- Comparación con OpenSim (Paso 2, opción "espaciada") ---
        # El bridge de OpenSim (solve_osim_angles + proyección Jacobiana)
        # es demasiado lento para correr cada frame de animación (~30 fps)
        # sin trabar la UI, así que corre en un timer SEPARADO y más lento
        # (cada 750 ms), no en update_all(). Carga el modelo una sola vez.
        self.osim_arm = None
        self.osim_worker = None
        # Guess inicial para el warm-start (Paso 2): se actualiza con el
        # resultado de cada llamada, para que la siguiente arranque desde
        # ahi (mucho mas rapido que el multi-start completo).
        self.osim_elv_guess = 0.0
        self.osim_shoulder_elv_guess = 0.5
        try:
            from opensim_bridge import OpenSimArmModel
            osim_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      "MoBL_ARMS_bimanual_6_2_21.osim")
            self._osim_path_for_bones = osim_path
            self.osim_arm = OpenSimArmModel(osim_path, side="r")
            self.lbl_osim_compare.setText("<b>OpenSim (real):</b> model loaded, waiting for first update...")
        except Exception as e:
            self.lbl_osim_compare.setText(f"<b>OpenSim (real):</b> not available ({e})")

        self.osim_timer = QTimer(self)
        self.osim_timer.timeout.connect(self.trigger_opensim_comparison)
        self.osim_timer.start(1200)  # cada 1.2 s, no cada frame

        # Debounce para las curvas de Torque vs Angulo (caras, 300 evals):
        # solo se recalculan 150 ms despues del ULTIMO movimiento de slider.
        self._pending_curve_args = None
        self.curve_debounce_timer = QTimer(self)
        self.curve_debounce_timer.setSingleShot(True)
        self.curve_debounce_timer.timeout.connect(self._refresh_torque_curves_deferred)

        self.init_torque_plots()
        self.init_equations_view()
        self.update_all()

    def _toggle_metrics_panel(self):
        collapsed = self.btn_toggle_metrics.isChecked()
        self.metrics_content_widget.setVisible(not collapsed)
        self.btn_toggle_metrics.setText(
            "▶ Show metrics" if collapsed else "▼ Hide metrics (more space for graphs)"
        )

    def _toggle_imu_axes(self):
        self.show_imu_axes = self.btn_toggle_imu_axes.isChecked()
        self.btn_toggle_imu_axes.setText(f"📡 IMU Axes: {'On' if self.show_imu_axes else 'Off'}")
        self.update_all()

    def _toggle_joint_labels(self):
        self.show_joint_labels = self.btn_toggle_joint_labels.isChecked()
        self.btn_toggle_joint_labels.setText(f"🔤 Joint Labels: {'On' if self.show_joint_labels else 'Off'}")
        self.update_all()

    def _toggle_3d_legend(self):
        self.show_3d_legend = self.btn_toggle_3d_legend.isChecked()
        self.btn_toggle_3d_legend.setText("🏷️ Legend: On" if self.show_3d_legend else "🏷️ Legend: Off")
        self.update_all()

    def clear_hand_trail(self):
        self.hand_trail.clear()
        self.update_all()

    def toggle_maximize_restore(self):
        if self.isMaximized():
            self.showNormal()
            self.btn_win_max.setText("🗖")
        else:
            self.showMaximized()
            self.btn_win_max.setText("🗗")

    def toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
            self.btn_win_fullscreen.setChecked(False)
        else:
            self.showFullScreen()
            self.btn_win_fullscreen.setChecked(True)

    def exit_fullscreen_only(self):
        # Escape solo sale de pantalla completa; no cierra la ventana.
        if self.isFullScreen():
            self.showNormal()
            self.btn_win_fullscreen.setChecked(False)

    def init_equations_view(self):
        self.fig_eq.clear()
        ax_eq = self.fig_eq.add_subplot(111)
        ax_eq.set_facecolor("#121212")
        ax_eq.axis('off')
        
        # Equations with larger LaTeX font (fontsize=9.5) and full torque set for all DOF
        eq_text = (
            r"$\mathbf{1.\ \text{IMU\ Kinematics\ and\ Quaternions}}$" "\n\n"
            r"Rotation Matrix to Quaternion $(w, x, y, z)$:" "\n"
            r"$w = \frac{1}{2}\sqrt{1 + R_{xx} + R_{yy} + R_{zz}}$" "\n"
            r"$x = \frac{R_{zy} - R_{yz}}{4w}, \ y = \frac{R_{xz} - R_{zx}}{4w}$" "\n\n"
            r"Relative Rotation (Child/Parent):" "\n"
            r"$R_{\text{rel}} = R_{\text{parent}}^T \cdot R_{\text{child}}$" "\n\n"
            r"$\mathbf{2.\ \text{Joint\ Angles}}$" "\n\n"
            r"Elbow:" "\n"
            r"$\theta_{\text{el}} = \text{atan2}(R_{\text{rel},(2,1)}, R_{\text{rel},(2,2)})$" "\n\n"
            r"Shoulder (Flex/Abd from $R_{\text{torso}}^T R_{\text{upper}}$):" "\n"
            r"$\theta_{\text{sf}} = \arcsin(-R_{\text{rel},(0,2)})$" "\n"
            r"$\theta_{\text{sa}} = \text{atan2}(R_{\text{rel},(1,2)}, R_{\text{rel},(2,2)})$" "\n\n"
            r"$\mathbf{3.\ \text{Torque\ Calculation\ (Full\ DOF)}}$" "\n\n"
            r"$\tau_{\text{elbow}} = \left( m_f \frac{L_f}{2} + m_p L_f \right) g \sin(\theta_{\text{el}})$" "\n\n"
            r"$\tau_{\text{sh\_flex}} = \left[ m_u \frac{L_u}{2} + (m_f + m_p) L_u \right] g \sin(\theta_{\text{sf}}) + \tau_{\text{elbow}}$" "\n\n"
            r"$\tau_{\text{sh\_abd}} = \left[ m_u \frac{L_u}{2} + (m_f + m_p) L_u \right] g \cos(\theta_{\text{sf}}) \sin(\theta_{\text{sa}})$" "\n\n"
            r"$\tau_{\text{human}} = \tau_{\text{total}} - \tau_{\text{exo}}$"
        )
        
             
        
        
        ax_eq.text(0.02, 0.98, eq_text, color="#c9d1d9", fontsize=9.5, 
                   verticalalignment='top', horizontalalignment='left',
                   transform=ax_eq.transAxes, family='sans-serif')
        
        self.fig_eq.subplots_adjust(left=0.01, right=0.99, top=0.99, bottom=0.01)
        self.canvas_eq.draw()

    def init_torque_plots(self):
        angles_range = np.linspace(-180, 180, 100)
        angles_rad = np.radians(angles_range)

        g = 9.81
        u_len = self.phys_sliders["upper_arm_len"].value()
        f_len = self.phys_sliders["forearm_len"].value()
        u_mass = self.phys_sliders["upper_arm_mass"].value()
        f_mass = self.phys_sliders["forearm_mass"].value()
        p_mass = self.phys_sliders["payload_mass"].value()

        curve_elbow = (f_mass * g * (f_len / 2.0) + p_mass * g * f_len) * np.sin(angles_rad)
        curve_sf = (u_mass * g * (u_len / 2.0) + (f_mass + p_mass) * g * u_len) * np.sin(angles_rad) + curve_elbow
        curve_sa = (u_mass * g * (u_len / 2.0) + (f_mass + p_mass) * g * u_len) * np.cos(0.0) * np.sin(angles_rad)

        bg_plot = "#121212"

        # Shoulder Flexion
        self.ax_sh_flex.set_facecolor(bg_plot)
        self.line_sf_tot, = self.ax_sh_flex.plot(angles_range, curve_sf, color='#8b949e', linestyle='--', linewidth=2, label='3D Total', zorder=4)
        self.line_sf_hum, = self.ax_sh_flex.plot(angles_range, curve_sf, color='#3fb950', linestyle='-', linewidth=2.5, label='3D Human', zorder=2)
        self.line_sf_exo, = self.ax_sh_flex.plot(angles_range, np.zeros_like(angles_range), color='#58a6ff', linestyle='-', linewidth=2.5, label='3D Exo', zorder=3)
        self.line_sf_simple, = self.ax_sh_flex.plot(angles_range, curve_sf, color='#d29922', linestyle=':', linewidth=2.2, label='Simple (sin/cos)', zorder=5)
        self.line_sf_osim, = self.ax_sh_flex.plot(angles_range, np.full_like(angles_range, np.nan), color='#bc8cff', linestyle='-.', linewidth=2.2, label='OpenSim (real)', zorder=6)
        self.fill_sf = self.ax_sh_flex.fill_between(angles_range, curve_sf, curve_sf, color='#3fb950', alpha=0.25)
        self.line_sf_fatigue = self.ax_sh_flex.axhline(y=40.0, color='#f85149', linestyle=':', linewidth=1.5, alpha=0.9)
        self.marker_sf, = self.ax_sh_flex.plot([0], [0], marker='o', markersize=14, color='#f85149', markeredgecolor='#ffffff', markeredgewidth=2, zorder=10)
        self.ax_sh_flex.set_title('Sh. Flexion (fatigue limit: 40 Nm)', color='#f85149', fontsize=14, fontweight='bold')
        self.ax_sh_flex.set_xlabel('Angle (°)', color='#c9d1d9', fontsize=12)
        self.ax_sh_flex.set_ylabel('Torque (Nm)', color='#c9d1d9', fontsize=12)
        self.ax_sh_flex.tick_params(colors='#c9d1d9', labelsize=11)
        self.ax_sh_flex.grid(True, color='#333333', linestyle='--', alpha=0.5)

        # Shoulder Abduction
        self.ax_sh_abd.set_facecolor(bg_plot)
        self.line_sa_tot, = self.ax_sh_abd.plot(angles_range, curve_sa, color='#8b949e', linestyle='--', linewidth=2, label='3D Total', zorder=4)
        self.line_sa_hum, = self.ax_sh_abd.plot(angles_range, curve_sa, color='#3fb950', linestyle='-', linewidth=2.5, label='3D Human', zorder=2)
        self.line_sa_exo, = self.ax_sh_abd.plot(angles_range, np.zeros_like(angles_range), color='#58a6ff', linestyle='-', linewidth=2.5, label='3D Exo', zorder=3)
        self.line_sa_simple, = self.ax_sh_abd.plot(angles_range, curve_sa, color='#d29922', linestyle=':', linewidth=2.2, label='Simple (sin/cos)', zorder=5)
        self.line_sa_osim, = self.ax_sh_abd.plot(angles_range, np.full_like(angles_range, np.nan), color='#bc8cff', linestyle='-.', linewidth=2.2, label='OpenSim (real)', zorder=6)
        self.fill_sa = self.ax_sh_abd.fill_between(angles_range, curve_sa, curve_sa, color='#3fb950', alpha=0.25)
        self.ax_sh_abd.axhline(y=30.0, color='#f85149', linestyle=':', linewidth=1.5, alpha=0.9)
        self.marker_sa, = self.ax_sh_abd.plot([0], [0], marker='o', markersize=14, color='#58a6ff', markeredgecolor='#ffffff', markeredgewidth=2, zorder=10)
        self.ax_sh_abd.set_title('Sh. Abduction (fatigue limit: 30 Nm)', color='#58a6ff', fontsize=14, fontweight='bold')
        self.ax_sh_abd.set_xlabel('Angle (°)', color='#c9d1d9', fontsize=12)
        self.ax_sh_abd.set_ylabel('Torque (Nm)', color='#c9d1d9', fontsize=12)
        self.ax_sh_abd.tick_params(colors='#c9d1d9', labelsize=11)
        self.ax_sh_abd.grid(True, color='#333333', linestyle='--', alpha=0.5)

        # Elbow
        self.ax_elbow.set_facecolor(bg_plot)
        self.line_el_tot, = self.ax_elbow.plot(angles_range, curve_elbow, color='#8b949e', linestyle='--', linewidth=2, label='3D Total', zorder=4)
        self.line_el_hum, = self.ax_elbow.plot(angles_range, curve_elbow, color='#3fb950', linestyle='-', linewidth=2.5, label='3D Human', zorder=2)
        self.line_el_exo, = self.ax_elbow.plot(angles_range, np.zeros_like(angles_range), color='#58a6ff', linestyle='-', linewidth=2.5, label='3D Exo', zorder=3)
        self.line_el_simple, = self.ax_elbow.plot(angles_range, curve_elbow, color='#d29922', linestyle=':', linewidth=2.2, label='Simple (sin/cos)', zorder=5)
        self.line_el_osim, = self.ax_elbow.plot(angles_range, np.full_like(angles_range, np.nan), color='#bc8cff', linestyle='-.', linewidth=2.2, label='OpenSim (real)', zorder=6)
        self.fill_el = self.ax_elbow.fill_between(angles_range, curve_elbow, curve_elbow, color='#3fb950', alpha=0.25)
        self.ax_elbow.axhline(y=25.0, color='#f85149', linestyle=':', linewidth=1.5, alpha=0.9)
        self.marker_el, = self.ax_elbow.plot([0], [0], marker='o', markersize=14, color='#3fb950', markeredgecolor='#ffffff', markeredgewidth=2, zorder=10)
        self.ax_elbow.set_title('Elbow (fatigue limit: 25 Nm)', color='#3fb950', fontsize=14, fontweight='bold')
        self.ax_elbow.set_xlabel('Angle (°)', color='#c9d1d9', fontsize=12)
        self.ax_elbow.set_ylabel('Torque (Nm)', color='#c9d1d9', fontsize=12)
        self.ax_elbow.tick_params(colors='#c9d1d9', labelsize=11)
        self.ax_elbow.grid(True, color='#333333', linestyle='--', alpha=0.5)

        # UNA sola leyenda compartida para toda la figura (en vez de 3
        # leyendas repetidas y diminutas) -- mas clara, y con espacio
        # reservado abajo via tight_layout(rect=...) para que no se solape.
        handles = [self.line_sf_tot, self.line_sf_hum, self.line_sf_exo,
                   self.line_sf_simple, self.line_sf_osim]
        labels = ['3D Total (authoritative)', '3D Human', '3D Exo',
                  'Simple (fórmula sin/cos)', 'OpenSim (real, si hay tabla)']
        self.fig_torque.legend(handles, labels, loc='lower center', ncol=5,
                                fontsize=11, facecolor='#121212', edgecolor='#333333',
                                labelcolor='#e6edf3', bbox_to_anchor=(0.5, -0.02))

        self.fig_torque.tight_layout(rect=[0, 0.08, 1, 1])

        # Tabla precalculada OpenSim (Opcion 1) -- se carga si existe el
        # archivo generado por precompute_sweep.py; si no existe, las
        # curvas OpenSim del grafico simplemente no se muestran (NaN).
        self.osim_table = None
        try:
            table_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "torque_comparison_table.npz")
            if os.path.exists(table_path):
                self.osim_table = np.load(table_path)
        except Exception as e:
            print(f"[Tabla OpenSim] No se pudo cargar: {e}")

    def on_exo_unit_changed(self, state):
        is_torque = (state == Qt.CheckState.Checked.value or state == 2)
        if is_torque:
            self.lbl_unit_mode.setText("<b>Torque (Nm)</b>")
            self.lbl_unit_mode.setStyleSheet("color: #3fb950;")
        else:
            self.lbl_unit_mode.setText("<b>Percentage (%)</b>")
            self.lbl_unit_mode.setStyleSheet("color: #58a6ff;")

        for key in ("shoulder_flex_exo", "shoulder_abd_exo", "elbow_exo"):
            s = self.phys_sliders[key]
            if not is_torque:
                s.spin.setMaximum(100.0)
                s.slider.setMaximum(int(100.0 * s.scale))
                s.spin.setSuffix(" %")
            else:
                s.spin.setMaximum(50.0)
                s.slider.setMaximum(int(50.0 * s.scale))
                s.spin.setSuffix(" Nm")
        self.update_all()

    def set_view(self, elev, azim):
        self.updating_cam = True
        self.slider_elev.setValue(elev)
        self.slider_azim.setValue(azim)
        self.slider_zoom.setValue(1.0)
        self.updating_cam = False
        self.update_all()

    def on_camera_slider_changed(self):
        if not self.updating_cam:
            self.update_all()

    def reset_angles(self):
        for name, s in self.sliders.items():
            lo_deg = math.degrees(next(j["lower"] for j in self.base_joints if j["name"] == name))
            hi_deg = math.degrees(next(j["upper"] for j in self.base_joints if j["name"] == name))
            default_deg = max(lo_deg, min(hi_deg, 0.0))
            s.setValue(default_deg)
        self.clear_hand_trail()
        self.update_all()

    ANIM_FREQ_HZ = {"shoulder_flex": 0.070, "shoulder_abd": 0.051, "elbow_flex": 0.093}
    ANIM_PHASE = {"shoulder_flex": 0.0, "shoulder_abd": 1.3, "elbow_flex": 2.6}
    ANIM_INTERVAL_MS = 33

    def toggle_animation(self):
        if self.btn_anim_play.isChecked():
            self.btn_anim_play.setText("⏸ Pause Animation")
            self.btn_anim_play.setStyleSheet("background-color: #d29922; color: #080808; border: none; font-weight: bold;")
            self.lbl_anim_status.setText("Status: Playing…")
            self.anim_timer.start(self.ANIM_INTERVAL_MS)
        else:
            self.btn_anim_play.setText("▶ Play Animation")
            self.btn_anim_play.setStyleSheet("background-color: #238636; color: #ffffff; border: none; font-weight: bold;")
            self.lbl_anim_status.setText("Status: Paused")
            self.anim_timer.stop()

    def stop_animation(self):
        self.anim_timer.stop()
        self.btn_anim_play.setChecked(False)
        self.btn_anim_play.setText("▶ Play Animation")
        self.btn_anim_play.setStyleSheet("background-color: #238636; color: #ffffff; border: none; font-weight: bold;")
        self.lbl_anim_status.setText("Status: Stopped")
        self.anim_t = 0.0
        self.reset_angles()

    def _animation_step(self):
        speed = self.slider_anim_speed.value()
        dt = self.ANIM_INTERVAL_MS / 1000.0
        self.anim_t += dt * speed

        mode = self.combo_anim_mode.currentText()
        active_joints = {
            "Both (Shoulder + Elbow)": ("shoulder_flex", "shoulder_abd", "elbow_flex"),
            "Shoulder only (Flex+Abd)": ("shoulder_flex", "shoulder_abd"),
            "Elbow only": ("elbow_flex",),
        }[mode]

        for name in active_joints:
            if name not in self.sliders:
                continue
            j = next(jj for jj in self.base_joints if jj["name"] == name)
            lo_deg, hi_deg = math.degrees(j["lower"]), math.degrees(j["upper"])
            mid, amp = (lo_deg + hi_deg) / 2.0, (hi_deg - lo_deg) / 2.0
            freq, phase0 = self.ANIM_FREQ_HZ[name], self.ANIM_PHASE[name]
            target_deg = mid + amp * math.sin(2 * math.pi * freq * self.anim_t + phase0)

            widget = self.sliders[name]
            widget.slider.blockSignals(True)
            widget.spin.blockSignals(True)
            widget.spin.setValue(target_deg)
            widget.slider.setValue(int(target_deg * widget.scale))
            widget.slider.blockSignals(False)
            widget.spin.blockSignals(False)

        self.update_all()

    def _exo_pulse_step(self):
        # Solo avanza la fase del pulso y redibuja; no toca los angulos de
        # las articulaciones (a diferencia de _animation_step).
        self.anim_t += self.ANIM_INTERVAL_MS / 1000.0
        self.update_all()

    def get_dynamic_urdf_structure(self):
        links = dict(self.base_links)
        joints = [dict(j) for j in self.base_joints]
        u_len = self.phys_sliders["upper_arm_len"].value()
        f_len = self.phys_sliders["forearm_len"].value()
        p_mass = self.phys_sliders["payload_mass"].value()

        for j in joints:
            if j["name"] == "elbow_flex":
                j["origin_xyz"] = np.array([0.0, 0.0, -u_len])
        
        for name, data in links.items():
            if name == "upper_arm" and data["geometry"]:
                data["geometry"]["length"] = u_len
                links[name]["vis_xyz"] = np.array([0.0, 0.0, -u_len / 2.0])
            elif name == "forearm" and data["geometry"]:
                data["geometry"]["length"] = f_len
                links[name]["vis_xyz"] = np.array([0.0, 0.0, -f_len / 2.0])
            elif name == "hand_payload" and data["geometry"]:
                data["geometry"]["radius"] = 0.025 + p_mass * 0.010

        return links, joints

    def trigger_opensim_comparison(self):
        """Lanza el calculo de comparacion con OpenSim en un QThread
        separado (ver OpenSimCompareWorker), para no bloquear la UI.
        Se llama desde self.osim_timer cada 750 ms.
        """
        if self.osim_arm is None:
            return
        # Si el calculo anterior todavia esta corriendo, no lanzar otro
        # encima -- evita amontonar hilos si el solver tarda mas de 750 ms.
        if self.osim_worker is not None and self.osim_worker.isRunning():
            return

        sh_abd = math.radians(self.sliders["shoulder_abd"].value()) if "shoulder_abd" in self.sliders else 0.0
        sh_flex = math.radians(self.sliders["shoulder_flex"].value()) if "shoulder_flex" in self.sliders else 0.0
        elbow_flex = math.radians(self.sliders["elbow_flex"].value()) if "elbow_flex" in self.sliders else 0.0
        f_len = self.phys_sliders["forearm_len"].value()
        p_mass = self.phys_sliders["payload_mass"].value()

        self.osim_worker = OpenSimCompareWorker(
            self.osim_arm, sh_abd, sh_flex, elbow_flex, f_len, p_mass,
            self.osim_elv_guess, self.osim_shoulder_elv_guess, parent=self
        )
        self.osim_worker.result_ready.connect(self._on_osim_result)
        self.osim_worker.start()

    def _on_osim_result(self, text, elv_angle, shoulder_elv):
        """Recibe el resultado del OpenSimCompareWorker: actualiza la
        etiqueta y guarda (elv_angle, shoulder_elv) como el guess de
        warm-start para la PROXIMA llamada (ver solve_osim_angles_warmstart
        en opensim_bridge.py)."""
        self.lbl_osim_compare.setText(text)
        self.osim_elv_guess = elv_angle
        self.osim_shoulder_elv_guess = shoulder_elv

    def _set_pv_camera(self, kind):
        """Reposiciona la camara del visor PyVista para que coincida con
        los botones Top/Side/Front/Default View existentes -- usa la
        convencion de ejes REAL de OpenSim ya calibrada visualmente en el
        Paso 7 del tutorial: 'adelante' es -X, 'lateral' es Z, Y es vertical.

        NOTA: se usa la API explicita de camara (position/focal_point/up)
        en vez de 'view_vector', porque view_vector dio resultados
        inconsistentes con lo esperado en pruebas anteriores -- con
        position/focal_point no hay ambiguedad de interpretacion posible.
        """
        if not PYVISTA_AVAILABLE or self.pv_plotter is None:
            return
        try:
            self.pv_plotter.reset_camera()
            focal = np.array(self.pv_plotter.camera.focal_point)
            bounds = self.pv_plotter.bounds
            diag = math.sqrt(sum((bounds[2 * i + 1] - bounds[2 * i]) ** 2 for i in range(3)))
            dist = max(diag * 1.3, 0.5)

            # Direccion CAMARA->FOCAL (hacia donde mira la camara), en
            # convencion OpenSim (X=?, Y=vertical, Z=lateral; adelante=-X)
            if kind == "top":
                cam_dir = np.array([0.001, -1, 0])   # mira hacia abajo
                up = np.array([-1, 0, 0])            # 'adelante' arriba en pantalla
            elif kind == "side":
                cam_dir = np.array([0, 0, -1])        # mira a lo largo de Z (lateral)
                up = np.array([0, 1, 0])
            elif kind == "front":
                cam_dir = np.array([1, 0, 0])         # mira a lo largo de X (adelante)
                up = np.array([0, 1, 0])
            else:  # iso / default
                cam_dir = np.array([1, -0.6, -1])
                up = np.array([0, 1, 0])

            cam_dir = cam_dir / np.linalg.norm(cam_dir)
            position = focal - cam_dir * dist  # la camara se ubica "detras" de la direccion de vista

            self.pv_plotter.camera.position = tuple(position)
            self.pv_plotter.camera.focal_point = tuple(focal)
            self.pv_plotter.camera.up = tuple(up)
            self.pv_plotter.render()
        except Exception as e:
            print(f"[PyVista camera] {e}")

    def load_or_update_skeleton(self):
        """Boton 'Load Skeleton': la primera vez, carga las mallas .vtp
        (mas lento, una sola vez). Las siguientes veces, solo actualiza la
        postura de los huesos ya cargados (mas rapido). Todo bajo demanda,
        nunca en tiempo real (ver tutorial Seccion 24)."""
        if not PYVISTA_AVAILABLE or self.osim_arm is None:
            return

        self.btn_load_bones.setEnabled(False)

        if not self.bone_meshes_loaded:
            self.lbl_bone_status.setText("Loading bone meshes (.vtp files)...")
            from bone_viewer import parse_body_meshes
            try:
                body_meshes = parse_body_meshes(self._osim_path_for_bones)
            except Exception as e:
                self.lbl_bone_status.setText(f"Error parsing .osim for meshes: {e}")
                self.btn_load_bones.setEnabled(True)
                return

            self.bone_load_worker = BoneMeshLoadWorker(self._osim_path_for_bones, body_meshes, side="r", parent=self)
            self.bone_load_worker.meshes_ready.connect(self._on_bone_meshes_ready)
            self.bone_load_worker.error_ready.connect(self._on_bone_load_error)
            self.bone_load_worker.start()
        else:
            self.lbl_bone_status.setText("Updating pose...")
            self._start_bone_transform_update()

    def _on_bone_load_error(self, error_text):
        self.btn_load_bones.setEnabled(True)
        self.lbl_bone_status.setText("Error loading meshes (see console).")
        print(f"[Bone mesh loading] Error:\n{error_text}")

    def _on_bone_meshes_ready(self, mesh_list):
        for body, file, polydata, scale, local_transform in mesh_list:
            actor = self.pv_plotter.add_mesh(polydata, color="#e8dcc8", smooth_shading=True,
                                              specular=0.3, name=f"{body}_{file}")
            try:
                actor.scale = scale
            except Exception:
                pass
            self.pv_actors.setdefault(body, []).append({"actor": actor, "local_transform": local_transform})

        self.pv_plotter.reset_camera()
        self._set_pv_camera("front")

        # Flecha de referencia INEQUIVOCA: apunta hacia "adelante" (-X en
        # este modelo, confirmado matematicamente con test_flex_vs_abd.py)
        # -- para no depender de interpretar visualmente hacia donde "mira"
        # la malla del torso (dificil de juzgar sin cabeza/pelvis de
        # referencia). Si el brazo en sh_flex=90 se alinea con esta flecha,
        # los datos y el render son correctos.
        try:
            arrow = pv.Arrow(start=(0, 0, 0), direction=(-1, 0, 0), scale=0.35)
            self.pv_plotter.add_mesh(arrow, color="red", name="anterior_ref_arrow")
            self.pv_plotter.add_point_labels(
                [(-0.4, 0.02, 0)], ["ANTERIOR (-X)"], text_color="red",
                font_size=14, shape=None, always_visible=True,
            )
        except Exception as e:
            print(f"[Bone viewer] No se pudo agregar la flecha de referencia: {e}")

        self.bone_meshes_loaded = True
        self.lbl_bone_status.setText(f"Loaded {len(mesh_list)} bone meshes. Updating pose...")
        self._start_bone_transform_update()

    def _start_bone_transform_update(self):
        if self.osim_arm is None:
            self.btn_load_bones.setEnabled(True)
            return
        sh_abd = math.radians(self.sliders["shoulder_abd"].value()) if "shoulder_abd" in self.sliders else 0.0
        sh_flex = math.radians(self.sliders["shoulder_flex"].value()) if "shoulder_flex" in self.sliders else 0.0

        self.bone_transform_worker = BoneTransformWorker(
            self.osim_arm, sh_abd, sh_flex, self.osim_elv_guess, self.osim_shoulder_elv_guess,
            side="r", parent=self
        )
        self.bone_transform_worker.transforms_ready.connect(self._on_bone_transforms_ready)
        self.bone_transform_worker.error_ready.connect(self._on_bone_transform_error)
        self.bone_transform_worker.start()

    def _on_bone_transform_error(self, error_text):
        self.btn_load_bones.setEnabled(True)
        self.lbl_bone_status.setText("Error updating pose (see console).")
        print(f"[Bone transform update] Error:\n{error_text}")

    def _on_bone_transforms_ready(self, body_transforms, elv_angle, shoulder_elv):
        self.btn_load_bones.setEnabled(True)
        self.osim_elv_guess = elv_angle
        self.osim_shoulder_elv_guess = shoulder_elv
        self.last_body_transforms = body_transforms

        self._apply_bone_transforms()
        self.lbl_bone_status.setText(
            f"Skeleton pose updated ({len(self.pv_actors)} bodies). Click again after moving sliders."
        )

    def _apply_bone_transforms(self):
        """Aplica self.last_body_transforms a los actores de PyVista,
        agregando la correccion manual de rotacion del torso
        (self.torso_fix_deg) solo a las mallas del body 'thorax'. Separado
        de _on_bone_transforms_ready para poder reaplicar instantaneamente
        cuando el usuario mueve el control 'Torso fix', sin recalcular la
        pose completa en OpenSim."""
        if not self.last_body_transforms or self.pv_plotter is None:
            return
        fix_rad = math.radians(self.torso_fix_deg)
        cos_f, sin_f = math.cos(fix_rad), math.sin(fix_rad)
        R_fix = np.array([[cos_f, 0, sin_f], [0, 1, 0], [-sin_f, 0, cos_f]])  # rotacion extra sobre Y

        for body_name, mesh_infos in self.pv_actors.items():
            T_body = self.last_body_transforms.get(body_name)
            if T_body is None:
                continue
            for info in mesh_infos:
                T_world = T_body @ info["local_transform"]
                if body_name == "thorax":
                    T_world = T_world.copy()
                    T_world[:3, :3] = T_world[:3, :3] @ R_fix
                info["actor"].user_matrix = T_world

        self.pv_plotter.render()

    def _on_toggle_scapula_visibility(self, state):
        visible = (state == Qt.CheckState.Checked.value or state == 2)
        for body_name in ("scapula_r", "clavicle_r"):
            for info in self.pv_actors.get(body_name, []):
                try:
                    info["actor"].visibility = visible
                except Exception:
                    pass
        if self.pv_plotter is not None:
            self.pv_plotter.render()

    def _on_torso_fix_changed(self, value):
        self.torso_fix_deg = value
        self._apply_bone_transforms()

    def run_muscle_analysis(self):
        """Lanza Static Optimization (Opcion 3) para la postura ACTUAL, en
        un QThread separado -- no bloquea la UI aunque tarde varios
        segundos. Muestra el resultado en una ventana emergente con
        grafica de barras (con vs. sin asistencia del exo) al terminar."""
        if self.osim_arm is None:
            return
        if self.muscle_worker is not None and self.muscle_worker.isRunning():
            return  # ya hay un analisis corriendo

        sh_abd = math.radians(self.sliders["shoulder_abd"].value()) if "shoulder_abd" in self.sliders else 0.0
        sh_flex = math.radians(self.sliders["shoulder_flex"].value()) if "shoulder_flex" in self.sliders else 0.0
        elbow_flex = math.radians(self.sliders["elbow_flex"].value()) if "elbow_flex" in self.sliders else 0.0
        p_mass = self.phys_sliders["payload_mass"].value()
        raw_sf_exo = self.phys_sliders["shoulder_flex_exo"].value()
        raw_sa_exo = self.phys_sliders["shoulder_abd_exo"].value()
        raw_el_exo = self.phys_sliders["elbow_exo"].value()
        is_percent = not self.switch_exo_unit.isChecked()

        self.btn_analyze_muscles.setEnabled(False)
        self.btn_analyze_muscles.setText("🔬 Analyzing...")

        self.muscle_worker = MuscleAnalysisWorker(
            self.osim_arm, sh_abd, sh_flex, elbow_flex, p_mass,
            raw_sf_exo, raw_sa_exo, raw_el_exo, is_percent,
            self.osim_elv_guess, self.osim_shoulder_elv_guess, parent=self
        )
        self.muscle_worker.result_ready.connect(self._on_muscle_analysis_result)
        self.muscle_worker.error_ready.connect(self._on_muscle_analysis_error)
        self.muscle_worker.start()

    def _on_muscle_analysis_error(self, error_text):
        self.btn_analyze_muscles.setEnabled(True)
        self.btn_analyze_muscles.setText("🔬 Analyze Muscles")
        print(f"[Muscle analysis] Error:\n{error_text}")

    def _on_muscle_analysis_result(self, result_no_exo, result_with_exo, target_no_exo, target_with_exo, elv_angle, shoulder_elv):
        self.btn_analyze_muscles.setEnabled(True)
        self.btn_analyze_muscles.setText("🔬 Analyze Muscles")
        self.osim_elv_guess = elv_angle
        self.osim_shoulder_elv_guess = shoulder_elv

        meta_no = result_no_exo.pop("_meta", {})
        meta_with = result_with_exo.pop("_meta", {})

        # Union de musculos relevantes en CUALQUIERA de los 2 escenarios,
        # para que la comparacion sea justa (un musculo puede aparecer en
        # uno y no en el otro si su activacion cambia mucho).
        all_names = set(n for n, info in result_no_exo.items() if info["activation"] > 0.005)
        all_names |= set(n for n, info in result_with_exo.items() if info["activation"] > 0.005)
        # Ordenar por la activacion SIN exo (de mayor a menor), para ver primero los que mas cambian
        muscles = sorted(all_names, key=lambda n: -result_no_exo.get(n, {}).get("activation", 0))

        exo_active = (self.phys_sliders["shoulder_flex_exo"].value() > 0
                      or self.phys_sliders["shoulder_abd_exo"].value() > 0
                      or self.phys_sliders["elbow_exo"].value() > 0)

        dlg = QDialog(self)
        dlg.setWindowTitle("Muscle Force Analysis (Static Optimization) — With vs. Without Exo Assist")
        dlg.resize(1100, max(460, 40 + 32 * max(len(muscles), 1)))
        layout = QVBoxLayout(dlg)

        info_lbl = QLabel(
            f"<b>Without exo — target torques:</b> "
            f"{coord_display_name('elv_angle_r')}={target_no_exo.get('elv_angle_r', 0):.2f} Nm | "
            f"{coord_display_name('shoulder_elv_r')}={target_no_exo.get('shoulder_elv_r', 0):.2f} Nm | "
            f"{coord_display_name('elbow_flexion_r')}={target_no_exo.get('elbow_flexion_r', 0):.2f} Nm<br>"
            f"<b>With exo — target torques:</b> "
            f"{coord_display_name('elv_angle_r')}={target_with_exo.get('elv_angle_r', 0):.2f} Nm | "
            f"{coord_display_name('shoulder_elv_r')}={target_with_exo.get('shoulder_elv_r', 0):.2f} Nm | "
            f"{coord_display_name('elbow_flexion_r')}={target_with_exo.get('elbow_flexion_r', 0):.2f} Nm "
            f"<font color='#8b949e'>({'no exo assist set — both bars should match' if not exo_active else 'exo assist active'})</font>"
        )
        info_lbl.setStyleSheet("color: #c9d1d9; padding: 6px;")
        info_lbl.setWordWrap(True)
        layout.addWidget(info_lbl)

        fig = Figure(figsize=(10.6, max(4, 0.36 * max(len(muscles), 1))), facecolor="#121212")
        canvas = FigureCanvasQTAgg(fig)
        ax = fig.add_subplot(111)
        ax.set_facecolor("#121212")

        if muscles:
            names = [muscle_display_name(n) for n in muscles]
            acts_no = [result_no_exo.get(n, {}).get("activation", 0.0) * 100 for n in muscles]
            acts_with = [result_with_exo.get(n, {}).get("activation", 0.0) * 100 for n in muscles]

            y = np.arange(len(muscles))
            bar_h = 0.38
            ax.barh(y + bar_h / 2, acts_no, height=bar_h, color="#f85149", label="Without exo assist")
            ax.barh(y - bar_h / 2, acts_with, height=bar_h, color="#3fb950", label="With exo assist")
            ax.set_yticks(y)
            ax.set_yticklabels(names)
            ax.set_xlabel("Activation (%)", color="#c9d1d9", fontsize=11)
            ax.tick_params(colors="#c9d1d9", labelsize=9)
            ax.grid(True, axis="x", color="#333333", linestyle="--", alpha=0.5)
            ax.legend(loc="lower right", fontsize=10, facecolor="#121212",
                      edgecolor="#333333", labelcolor="#e6edf3")
            ax.invert_yaxis()
        else:
            ax.text(0.5, 0.5, "No muscle with relevant activation", color="#8b949e",
                    ha="center", va="center", transform=ax.transAxes)
        fig.tight_layout(rect=[0.0, 0.0, 1.0, 1.0])
        fig.subplots_adjust(left=0.38)
        layout.addWidget(canvas)

        btn_close = QPushButton("Close")
        btn_close.clicked.connect(dlg.accept)
        layout.addWidget(btn_close)

        dlg.setStyleSheet("background-color: #0d1117;")
        dlg.exec()

    def update_all(self):
        angles = {name: math.radians(s.value()) for name, s in self.sliders.items()}
        links, joints = self.get_dynamic_urdf_structure()
        transforms = self.compute_world_transforms(self.root_link, joints, angles)
        
        R_torso = transforms.get("torso", np.eye(4))[:3, :3]
        R_upper = transforms.get("upper_arm", np.eye(4))[:3, :3]
        R_fore = transforms.get("forearm", np.eye(4))[:3, :3]

        if "IMU_Torso" in self.imu_sliders:
            torso_rpy = [math.radians(self.imu_sliders["IMU_Torso"][k].value()) for k in ("roll", "pitch", "yaw")]
            R_torso = R_torso @ rpy_to_matrix(*torso_rpy)
        if "IMU_UpperArm" in self.imu_sliders:
            upper_rpy = [math.radians(self.imu_sliders["IMU_UpperArm"][k].value()) for k in ("roll", "pitch", "yaw")]
            R_upper = R_upper @ rpy_to_matrix(*upper_rpy)
        if "IMU_Forearm" in self.imu_sliders:
            fore_rpy = [math.radians(self.imu_sliders["IMU_Forearm"][k].value()) for k in ("roll", "pitch", "yaw")]
            R_fore = R_fore @ rpy_to_matrix(*fore_rpy)

        self.R_torso, self.R_upper, self.R_fore = R_torso, R_upper, R_fore

        g = 9.81
        u_len = self.phys_sliders["upper_arm_len"].value()
        f_len = self.phys_sliders["forearm_len"].value()
        u_mass = self.phys_sliders["upper_arm_mass"].value()
        f_mass = self.phys_sliders["forearm_mass"].value()
        p_mass = self.phys_sliders["payload_mass"].value()

        sh_flex_deg = self.sliders.get("shoulder_flex").value() if "shoulder_flex" in self.sliders else 0.0
        sh_abd_deg = self.sliders.get("shoulder_abd").value() if "shoulder_abd" in self.sliders else 0.0
        elbow_deg = self.sliders.get("elbow_flex").value() if "elbow_flex" in self.sliders else 0.0

        sh_flex = angles.get("shoulder_flex", 0.0)
        sh_abd = angles.get("shoulder_abd", 0.0)
        elbow_flex = angles.get("elbow_flex", 0.0)

        torque_elbow_simple = (f_mass * g * (f_len / 2.0) + p_mass * g * f_len) * math.sin(elbow_flex)
        torque_sh_flex_simple = (u_mass * g * (u_len / 2.0) + (f_mass + p_mass) * g * u_len) * math.sin(sh_flex) + torque_elbow_simple
        torque_sh_abd_simple = (u_mass * g * (u_len / 2.0) + (f_mass + p_mass) * g * u_len) * math.cos(sh_flex) * math.sin(sh_abd)

        # --- Paso 1: torque 3D completo (r x F), reemplaza la aproximacion
        # seno/coseno de un solo eje como valor AUTORITATIVO (el que se usa
        # para asistencia de exo, alertas, etc.) -- ver torque_3d.py y la
        # Seccion 16 del tutorial para el porque y la validacion. La formula
        # simple se conserva arriba solo para mostrarla como referencia.
        torques_3d = compute_full_3d_torques(transforms, joints, u_len, f_len, u_mass, f_mass, p_mass, g)
        torque_sh_flex = torques_3d["sh_flex"]
        torque_sh_abd = torques_3d["sh_abd"]
        torque_elbow = torques_3d["elbow"]

        raw_sf_exo = self.phys_sliders["shoulder_flex_exo"].value()
        raw_sa_exo = self.phys_sliders["shoulder_abd_exo"].value()
        raw_el_exo = self.phys_sliders["elbow_exo"].value()
        is_percent = not self.switch_exo_unit.isChecked()

        # El pulso del halo de asistencia necesita que self.anim_t avance.
        # Si la animación de articulaciones ya está corriendo, ese timer ya
        # lo hace (evitamos duplicar); si no, usamos el timer dedicado.
        any_assist_active = (raw_sf_exo > 0 or raw_sa_exo > 0 or raw_el_exo > 0)
        if any_assist_active and not self.anim_timer.isActive():
            if not self.exo_pulse_timer.isActive():
                self.exo_pulse_timer.start(self.ANIM_INTERVAL_MS)
        else:
            if self.exo_pulse_timer.isActive():
                self.exo_pulse_timer.stop()

        if is_percent:
            exo_sf = torque_sh_flex * (raw_sf_exo / 100.0)
            exo_sa = torque_sh_abd * (raw_sa_exo / 100.0)
            exo_el = torque_elbow * (raw_el_exo / 100.0)
        else:
            exo_sf = raw_sf_exo
            exo_sa = raw_sa_exo
            exo_el = raw_el_exo

        hum_sf = torque_sh_flex - exo_sf
        hum_sa = torque_sh_abd - exo_sa
        hum_el = torque_elbow - exo_el

        self.lbl_angles.setText(f"<b>Angles:</b> <font color='#f85149'>Sh_Flex: {sh_flex_deg:.1f}°</font> | <font color='#58a6ff'>Sh_Abd: {sh_abd_deg:.1f}°</font> | <font color='#3fb950'>Elbow: {elbow_deg:.1f}°</font>")

        sf_alert = abs(hum_sf) > 40.0
        sf_bg = "#2d1618" if sf_alert else "#080808"
        sf_border = "#f85149" if sf_alert else "#3fb950"
        self.lbl_t_sh_flex.setStyleSheet(f"font-size: 11px; font-weight: bold; background: {sf_bg}; padding: 5px; border-radius: 4px; border: 1px solid #262626; border-left: 4px solid {sf_border};")
        self.lbl_t_sh_flex.setText(f"<b>Sh. Flexion:</b> <font color='#8b949e'>Total: {torque_sh_flex:.2f} Nm</font> | <font color='{sf_border}'>Human: {hum_sf:.2f} Nm</font> | <font color='#58a6ff'>Exo: {exo_sf:.2f} Nm</font> | <font color='#6e7681'>(simple: {torque_sh_flex_simple:.2f} Nm)</font>")

        sa_alert = abs(hum_sa) > 30.0
        sa_bg = "#2d1618" if sa_alert else "#080808"
        sa_border = "#f85149" if sa_alert else "#3fb950"
        self.lbl_t_sh_abd.setStyleSheet(f"font-size: 11px; font-weight: bold; background: {sa_bg}; padding: 5px; border-radius: 4px; border: 1px solid #262626; border-left: 4px solid {sa_border};")
        self.lbl_t_sh_abd.setText(f"<b>Sh. Abduction:</b> <font color='#8b949e'>Total: {torque_sh_abd:.2f} Nm</font> | <font color='{sa_border}'>Human: {hum_sa:.2f} Nm</font> | <font color='#58a6ff'>Exo: {exo_sa:.2f} Nm</font> | <font color='#6e7681'>(simple: {torque_sh_abd_simple:.2f} Nm)</font>")

        el_alert = abs(hum_el) > 25.0
        el_bg = "#2d1618" if el_alert else "#080808"
        el_border = "#f85149" if el_alert else "#3fb950"
        self.lbl_t_elbow.setStyleSheet(f"font-size: 11px; font-weight: bold; background: {el_bg}; padding: 5px; border-radius: 4px; border: 1px solid #262626; border-left: 4px solid {el_border};")
        self.lbl_t_elbow.setText(f"<b>Elbow:</b> <font color='#8b949e'>Total: {torque_elbow:.2f} Nm</font> | <font color='{el_border}'>Human: {hum_el:.2f} Nm</font> | <font color='#58a6ff'>Exo: {exo_el:.2f} Nm</font> | <font color='#6e7681'>(simple: {torque_elbow_simple:.2f} Nm)</font>")

        # El MARCADOR (posicion actual) se mueve SIEMPRE, en cada frame --
        # incluida la animacion automatica -- porque es barato (un solo
        # punto, sin barrido de cinematica). Se separa a proposito del
        # debounce de las curvas (que SI son caras): durante la animacion
        # continua, el debounce nunca "hace pausa", asi que si el marcador
        # dependiera de el, nunca se moveria mientras la animacion corre.
        self.marker_sf.set_data([sh_flex_deg], [torque_sh_flex])
        self.marker_sa.set_data([sh_abd_deg], [torque_sh_abd])
        self.marker_el.set_data([elbow_deg], [torque_elbow])
        self.canvas_torque.draw_idle()

        # La vista 3D se redibuja SIEMPRE, de inmediato (es la animacion
        # principal, tiene que sentirse fluida al mover sliders).
        self._draw(transforms, links)

        # Las curvas de "Torque vs Angulo" son caras (300 evaluaciones de
        # cinematica completa) -- en vez de recalcularlas en cada evento de
        # slider (lo que congelaba la UI), se guardan los datos necesarios
        # y se programa un redibujado DIFERIDO que solo se ejecuta 150 ms
        # despues del ULTIMO movimiento (debounce). Si el usuario sigue
        # moviendo el slider, el timer se reinicia una y otra vez y el
        # calculo pesado nunca llega a correr hasta que hay una pausa.
        self._pending_curve_args = dict(
            angles=angles, joints=joints, u_len=u_len, f_len=f_len,
            u_mass=u_mass, f_mass=f_mass, p_mass=p_mass, g=g,
            sh_flex_deg=sh_flex_deg, sh_abd_deg=sh_abd_deg, elbow_deg=elbow_deg,
            torque_sh_flex=torque_sh_flex, torque_sh_abd=torque_sh_abd, torque_elbow=torque_elbow,
            raw_sf_exo=raw_sf_exo, raw_sa_exo=raw_sa_exo, raw_el_exo=raw_el_exo,
            is_percent=is_percent,
        )
        # IMPORTANTE: "throttle", no "debounce" puro. Si update_all() se
        # llama de forma CONTINUA (animacion de articulaciones, o el pulso
        # de brillo del exo que se activa solo con asistencia > 0%), un
        # debounce que se reinicia en cada llamada NUNCA llega a disparar
        # -- las curvas quedarian congeladas para siempre mientras el
        # pulso/animacion siga corriendo. Por eso solo iniciamos el timer
        # si NO esta ya corriendo: se dispara si o si cada 150ms, aunque
        # los eventos sigan llegando sin parar.
        if not self.curve_debounce_timer.isActive():
            self.curve_debounce_timer.start(150)

    def _lookup_osim_curve(self, axis_name, sweep_degrees, sh_abd_now, sh_flex_now, elbow_now):
        """Busca en la tabla precalculada (torque_comparison_table.npz,
        generada por precompute_sweep.py) la curva de torque OpenSim para
        el eje 'axis_name' ('sh_flex', 'sh_abd', o 'elbow'), barriendo ese
        eje en 'sweep_degrees' y manteniendo los otros 2 angulos fijos en
        el punto de la grilla mas cercano al valor ACTUAL de los sliders.

        Usa vecino-mas-cercano (no interpolacion suave) por simplicidad --
        suficiente para una curva de referencia visual. Devuelve NaN fuera
        del rango cubierto por la tabla, o si la tabla no esta cargada.
        """
        result = np.full_like(sweep_degrees, np.nan)
        if self.osim_table is None:
            return result
        try:
            abd_grid = self.osim_table["sh_abd_grid"]
            flex_grid = self.osim_table["sh_flex_grid"]
            elbow_grid = self.osim_table["elbow_grid"]
            key = {"sh_flex": "tau_sh_flex", "sh_abd": "tau_sh_abd", "elbow": "tau_elbow"}[axis_name]
            table = self.osim_table[key]  # shape (n_abd, n_flex, n_elbow)

            i_abd = int(np.argmin(np.abs(abd_grid - sh_abd_now)))
            i_flex = int(np.argmin(np.abs(flex_grid - sh_flex_now)))
            i_elbow = int(np.argmin(np.abs(elbow_grid - elbow_now)))

            if axis_name == "sh_flex":
                grid, vals = flex_grid, table[i_abd, :, i_elbow]
            elif axis_name == "sh_abd":
                grid, vals = abd_grid, table[:, i_flex, i_elbow]
            else:
                grid, vals = elbow_grid, table[i_abd, i_flex, :]

            for i, deg in enumerate(sweep_degrees):
                if grid.min() <= deg <= grid.max():
                    j = int(np.argmin(np.abs(grid - deg)))
                    result[i] = vals[j]
        except Exception:
            pass
        return result

    def _refresh_torque_curves_deferred(self):
        """Recalculo pesado (300 evaluaciones de cinematica) y redibujado
        de las curvas de Torque vs Angulo -- se ejecuta solo 150 ms despues
        del ultimo movimiento de slider (ver update_all), nunca en cada
        evento individual."""
        if self._pending_curve_args is None:
            return
        try:
            self._do_refresh_torque_curves()
        except Exception:
            import traceback
            print("[Graficas de torque] Error al actualizar (ver detalle abajo):")
            traceback.print_exc()

    def _do_refresh_torque_curves(self):
        a = self._pending_curve_args
        angles, joints = a["angles"], a["joints"]
        u_len, f_len, u_mass, f_mass, p_mass, g = a["u_len"], a["f_len"], a["u_mass"], a["f_mass"], a["p_mass"], a["g"]

        angles_range = np.linspace(-180, 180, 100)
        angles_rad = np.radians(angles_range)

        sh_abd_now = math.degrees(angles.get("shoulder_abd", 0.0))
        sh_flex_now = math.degrees(angles.get("shoulder_flex", 0.0))
        elbow_now = math.degrees(angles.get("elbow_flex", 0.0))

        # --- Curva "simple" (seno/coseno) -- vectorizada, sin cinematica,
        # se recalcula siempre (es barata) para la comparacion visual ---
        torque_elbow_simple_now = (f_mass * g * (f_len / 2.0) + p_mass * g * f_len) * math.sin(angles.get("elbow_flex", 0.0))
        c_sf_simple = (u_mass * g * (u_len / 2.0) + (f_mass + p_mass) * g * u_len) * np.sin(angles_rad) + torque_elbow_simple_now
        c_sa_simple = (u_mass * g * (u_len / 2.0) + (f_mass + p_mass) * g * u_len) * math.cos(angles.get("shoulder_flex", 0.0)) * np.sin(angles_rad)
        c_el_simple = (f_mass * g * (f_len / 2.0) + p_mass * g * f_len) * np.sin(angles_rad)

        # --- Curva "OpenSim" -- interpolacion desde la tabla precalculada
        # (precompute_sweep.py), si existe. Si no, queda en NaN (no se dibuja).
        c_sf_osim = self._lookup_osim_curve("sh_flex", angles_range, sh_abd_now, sh_flex_now, elbow_now)
        c_sa_osim = self._lookup_osim_curve("sh_abd", angles_range, sh_abd_now, sh_flex_now, elbow_now)
        c_el_osim = self._lookup_osim_curve("elbow", angles_range, sh_abd_now, sh_flex_now, elbow_now)

        c_sf = np.zeros_like(angles_rad)
        c_sa = np.zeros_like(angles_rad)
        c_el = np.zeros_like(angles_rad)
        base_angles = dict(angles)
        for i, ang in enumerate(angles_rad):
            sweep_angles = dict(base_angles)
            sweep_angles["shoulder_flex"] = ang
            t_sf = self.compute_world_transforms(self.root_link, joints, sweep_angles)
            r_sf = compute_full_3d_torques(t_sf, joints, u_len, f_len, u_mass, f_mass, p_mass, g)
            c_sf[i] = r_sf["sh_flex"]

            sweep_angles = dict(base_angles)
            sweep_angles["shoulder_abd"] = ang
            t_sa = self.compute_world_transforms(self.root_link, joints, sweep_angles)
            r_sa = compute_full_3d_torques(t_sa, joints, u_len, f_len, u_mass, f_mass, p_mass, g)
            c_sa[i] = r_sa["sh_abd"]

            sweep_angles = dict(base_angles)
            sweep_angles["elbow_flex"] = ang
            t_el = self.compute_world_transforms(self.root_link, joints, sweep_angles)
            r_el = compute_full_3d_torques(t_el, joints, u_len, f_len, u_mass, f_mass, p_mass, g)
            c_el[i] = r_el["elbow"]

        if a["is_percent"]:
            e_sf = c_sf * (a["raw_sf_exo"] / 100.0)
            e_sa = c_sa * (a["raw_sa_exo"] / 100.0)
            e_el = c_el * (a["raw_el_exo"] / 100.0)
        else:
            e_sf = np.full_like(c_sf, a["raw_sf_exo"])
            e_sa = np.full_like(c_sa, a["raw_sa_exo"])
            e_el = np.full_like(c_el, a["raw_el_exo"])

        h_sf, h_sa, h_el = c_sf - e_sf, c_sa - e_sa, c_el - e_el

        self.line_sf_tot.set_ydata(c_sf)
        self.line_sf_hum.set_ydata(h_sf)
        self.line_sf_exo.set_ydata(e_sf)
        self.line_sf_simple.set_ydata(c_sf_simple)
        self.line_sf_osim.set_ydata(c_sf_osim)
        self.fill_sf.remove()
        self.fill_sf = self.ax_sh_flex.fill_between(angles_range, h_sf, c_sf, color='#3fb950', alpha=0.25)
        self.marker_sf.set_data([a["sh_flex_deg"]], [a["torque_sh_flex"]])

        self.line_sa_tot.set_ydata(c_sa)
        self.line_sa_hum.set_ydata(h_sa)
        self.line_sa_exo.set_ydata(e_sa)
        self.line_sa_simple.set_ydata(c_sa_simple)
        self.line_sa_osim.set_ydata(c_sa_osim)
        self.fill_sa.remove()
        self.fill_sa = self.ax_sh_abd.fill_between(angles_range, h_sa, c_sa, color='#3fb950', alpha=0.25)
        self.marker_sa.set_data([a["sh_abd_deg"]], [a["torque_sh_abd"]])

        self.line_el_tot.set_ydata(c_el)
        self.line_el_hum.set_ydata(h_el)
        self.line_el_exo.set_ydata(e_el)
        self.line_el_simple.set_ydata(c_el_simple)
        self.line_el_osim.set_ydata(c_el_osim)
        self.fill_el.remove()
        self.fill_el = self.ax_elbow.fill_between(angles_range, h_el, c_el, color='#3fb950', alpha=0.25)
        self.marker_el.set_data([a["elbow_deg"]], [a["torque_elbow"]])

        for ax in (self.ax_sh_flex, self.ax_sh_abd, self.ax_elbow):
            ax.relim()
            ax.autoscale_view()

        self.canvas_torque.draw()

    def compute_world_transforms(self, root_link, joints, joint_angles):
        joints_by_parent = {}
        for j in joints:
            joints_by_parent.setdefault(j["parent"], []).append(j)

        transforms = {root_link: np.eye(4)}
        stack = [root_link]
        while stack:
            parent = stack.pop()
            for j in joints_by_parent.get(parent, []):
                angle = joint_angles.get(j["name"], 0.0) if j["type"] in ("revolute", "continuous") else 0.0
                T_origin = make_transform(j["origin_xyz"], j["origin_rpy"])
                T_axis = np.eye(4)
                if j["type"] in ("revolute", "continuous"):
                    T_axis[:3, :3] = axis_angle_matrix(j["axis"], angle)
                T_joint = T_origin @ T_axis
                transforms[j["child"]] = transforms[parent] @ T_joint
                stack.append(j["child"])
        return transforms

    def _draw(self, transforms, links_geom):
        ax = self.ax
        ax.clear()

        bg_color = "#000000"
        ax.set_facecolor(bg_color)
        self.fig.patch.set_facecolor(bg_color)

        all_points = []
        segment_endpoints = {}  # link_name -> (p1, p2), para el brillo de asistencia
        LINK_COLORS = {"upper_arm": "#58a6ff", "forearm": "#3fb950"}
        LINK_LABELS = {"upper_arm": None, "forearm": None}
        legend_seen = set()

        def _labeled(key, label):
            if not label or key in legend_seen:
                return None
            legend_seen.add(key)
            return label

        hand_world_pos = None

        for link_name, T_link in transforms.items():
            geom = links_geom.get(link_name, {}).get("geometry")
            if geom is None:
                continue
            vis_xyz = links_geom[link_name]["vis_xyz"]
            vis_rpy = links_geom[link_name]["vis_rpy"]
            T_vis = T_link @ make_transform(vis_xyz, vis_rpy)
            R = T_vis[:3, :3]
            p = T_vis[:3, 3]

            if link_name == "forearm" or link_name == "hand_payload":
                hand_world_pos = p

            if geom["type"] == "cylinder":
                half = geom["length"] / 2.0
                p1 = p + R @ np.array([0, 0, -half])
                p2 = p + R @ np.array([0, 0, half])
                segment_endpoints[link_name] = (p1, p2)
                color = LINK_COLORS.get(link_name, "#57606a")
                label = _labeled(link_name, LINK_LABELS.get(link_name, None))
                ax.plot(*zip(p1, p2), linewidth=max(3, geom["radius"] * 350), solid_capstyle="round",
                        color=color, label=label)
                all_points.extend([p1, p2])

            elif geom["type"] == "sphere":
                ax.scatter(*p, s=max(40, geom["radius"] * 4500), color="#f85149",
                           edgecolor="white", linewidth=0.8, depthshade=True)
                all_points.append(p)

            elif geom["type"] == "box":
                sx, sy, sz = geom["size"]
                corners_local = np.array([
                    [dx * sx / 2, dy * sy / 2, dz * sz / 2]
                    for dx in (-1, 1) for dy in (-1, 1) for dz in (-1, 1)
                ])
                corners_world = np.array([p + R @ c for c in corners_local])
                edges = [(0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
                         (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7)]
                label = _labeled("torso", "Torso")
                for i, (a, b) in enumerate(edges):
                    ax.plot(*zip(corners_world[a], corners_world[b]), color="#262626", linewidth=1.2,
                            label=label if i == 0 else None)
                all_points.extend(list(corners_world))

        if all_points:
            pts = np.array(all_points)
            center = pts.mean(axis=0)
            max_range = max(0.3, (pts.max(axis=0) - pts.min(axis=0)).max() * 0.6)
        else:
            center = np.zeros(3)
            max_range = 0.5

        zoom_factor = self.slider_zoom.value()
        final_range = max_range / zoom_factor
        floor_z = center[2] - final_range * 0.95

        if hand_world_pos is not None:
            self.hand_trail.append(hand_world_pos)
            if len(self.hand_trail) > 60:
                self.hand_trail.pop(0)

            if len(self.hand_trail) > 1:
                trail_arr = np.array(self.hand_trail)
                ax.plot(trail_arr[:, 0], trail_arr[:, 1], trail_arr[:, 2],
                        color='#d29922', linestyle=':', linewidth=1.8, alpha=0.85,
                        label=_labeled("hand_trail", "Trajectory Trail"))

            theta_s = np.linspace(0, 2 * np.pi, 20)
            shadow_radius = 0.08
            sx_pts = hand_world_pos[0] + shadow_radius * np.cos(theta_s)
            sy_pts = hand_world_pos[1] + shadow_radius * np.sin(theta_s)
            sz_pts = np.full_like(sx_pts, floor_z + 0.001)
            ax.plot(sx_pts, sy_pts, sz_pts, color="#1f6feb", linewidth=1.5, alpha=0.6)
            ax.scatter(hand_world_pos[0], hand_world_pos[1], floor_z + 0.001, color="#1f6feb", alpha=0.3, s=150)

        # --- Asistencia de exoesqueleto activa: funciona igual en modo %/Nm
        # (se normaliza contra el máximo actual del slider en cada modo) ---
        is_percent_mode = not self.switch_exo_unit.isChecked()
        raw_sf = self.phys_sliders["shoulder_flex_exo"].value()
        raw_sa = self.phys_sliders["shoulder_abd_exo"].value()
        raw_el = self.phys_sliders["elbow_exo"].value()
        norm_max = 100.0 if is_percent_mode else 50.0

        shoulder_frac = max(raw_sf, raw_sa) / norm_max
        elbow_frac = raw_el / norm_max
        shoulder_assisted = shoulder_frac > 0
        elbow_assisted = elbow_frac > 0

        # Pulso tipo "latido": avanza con self.anim_t (30 fps mientras la
        # animación está corriendo; si está pausada, queda fijo en su fase).
        pulse = 0.5 + 0.5 * math.sin(2 * math.pi * 1.6 * getattr(self, "anim_t", 0.0))
        EXO_GLOW_COLOR = "#ffd60a"

        def draw_assist_glow(link_name, frac):
            """Brillo pulsante a lo largo de TODO el segmento asistido (no solo
            un punto), con intensidad proporcional a la asistencia aplicada."""
            if link_name not in segment_endpoints or frac <= 0:
                return
            p1, p2 = segment_endpoints[link_name]
            intensity = min(1.0, frac)
            base_width = 14 + 26 * intensity
            pulse_width = base_width * (0.7 + 0.6 * pulse)
            alpha_outer = (0.10 + 0.18 * intensity) * (0.6 + 0.4 * pulse)
            alpha_inner = (0.22 + 0.30 * intensity) * (0.6 + 0.4 * pulse)
            label = _labeled("exo_glow", "⚡ Exo Assist Active")
            ax.plot(*zip(p1, p2), color=EXO_GLOW_COLOR, linewidth=pulse_width,
                    alpha=alpha_outer, solid_capstyle="round", zorder=3, label=label)
            ax.plot(*zip(p1, p2), color=EXO_GLOW_COLOR, linewidth=pulse_width * 0.5,
                    alpha=alpha_inner, solid_capstyle="round", zorder=3)

        draw_assist_glow("upper_arm", shoulder_frac)
        draw_assist_glow("forearm", elbow_frac)

        JOINT_PIVOT_COLORS = {
            "upper_arm": ("#f85149", "Shoulder"),
            "forearm": ("#3fb950", "Elbow"),
            "hand_payload": ("#cf222e", None),
        }
        for link_name, T_link in transforms.items():
            p = T_link[:3, 3]
            color, joint_label = JOINT_PIVOT_COLORS.get(link_name, ("#1f2328", None))
            label = _labeled(f"pivot_{color}", "Joint pivot") if joint_label else None
            is_assisted = (link_name == "upper_arm" and shoulder_assisted) or \
                          (link_name == "forearm" and elbow_assisted)
            joint_frac = shoulder_frac if link_name == "upper_arm" else (
                elbow_frac if link_name == "forearm" else 0.0)

            if is_assisted:
                # Halo pulsante en el pivote, ademas del brillo del segmento.
                pulse_size = (260 + 260 * min(1.0, joint_frac)) * (0.75 + 0.5 * pulse)
                ax.scatter(*p, s=pulse_size * 1.7, color=EXO_GLOW_COLOR,
                           alpha=0.10 + 0.10 * pulse, zorder=4, edgecolor="none")
                ax.scatter(*p, s=pulse_size, color=EXO_GLOW_COLOR,
                           alpha=0.22 + 0.18 * pulse, zorder=5, edgecolor="none")

            ax.scatter(*p, s=45 if joint_label else 12, color=color, label=label,
                       edgecolor="white", linewidth=0.8, zorder=6)
            
            if joint_label and getattr(self, "show_joint_labels", True):
                label_text = f"⚡ {joint_label}" if is_assisted else joint_label
                label_color = "#ffd60a" if is_assisted else "#c9d1d9"
                ax.text(p[0] + 0.015, p[1] + 0.015, p[2] + 0.015, label_text,
                        color=label_color, fontsize=8, fontweight="bold")
            all_points.append(p)

        triad_len = final_range * 0.16
        if hasattr(self, "R_upper") and getattr(self, "show_imu_axes", True):
            torso_T = transforms.get("torso", np.eye(4))
            upper_T = transforms.get("upper_arm", np.eye(4))
            forearm_T = transforms.get("forearm", np.eye(4))

            torso_imu_pos = torso_T[:3, 3] + np.array([0.0, 0.0, -0.05])
            upper_mid = (upper_T[:3, 3] + transforms.get("forearm", upper_T)[:3, 3]) / 2.0
            forearm_p = forearm_T[:3, 3]
            hand_link = links_geom.get("hand_payload", {})
            hand_T_vis = transforms.get("hand_payload", forearm_T) @ make_transform(
                hand_link.get("vis_xyz", np.zeros(3)), hand_link.get("vis_rpy", np.zeros(3)))
            hand_p = hand_T_vis[:3, 3]
            forearm_mid = (forearm_p + hand_p) / 2.0

            draw_axis_triad(ax, torso_imu_pos, self.R_torso, length=triad_len,
                             label="IMU: Torso", add_legend_labels=True)
            draw_axis_triad(ax, upper_mid, self.R_upper, length=triad_len, label="IMU: Upper Arm")
            draw_axis_triad(ax, forearm_mid, self.R_fore, length=triad_len, label="IMU: Forearm")

        g_origin = np.array([center[0] + final_range * 0.7, center[1] + final_range * 0.7,
                             center[2] + final_range * 0.7])
        g_dir = np.array([0.0, 0.0, -final_range * 0.35])
        ax.quiver(*g_origin, *g_dir, color="#8b949e", linewidth=1.6, arrow_length_ratio=0.25,
                  label="Gravity (g)")
        ax.text(*(g_origin + np.array([0.01, 0, 0.01])), "g", color="#8b949e", fontsize=9, fontweight="bold")

        grid_n = 5
        xs = np.linspace(center[0] - final_range, center[0] + final_range, grid_n)
        ys = np.linspace(center[1] - final_range, center[1] + final_range, grid_n)
        for x in xs:
            ax.plot([x, x], [ys[0], ys[-1]], [floor_z, floor_z], color="#262626", linewidth=0.6, alpha=0.5)
        for y in ys:
            ax.plot([xs[0], xs[-1]], [y, y], [floor_z, floor_z], color="#262626", linewidth=0.6, alpha=0.5)

        ax.set_xlim(center[0] - final_range, center[0] + final_range)
        ax.set_ylim(center[1] - final_range, center[1] + final_range)
        ax.set_zlim(center[2] - final_range, center[2] + final_range)
        ax.set_box_aspect((1, 1, 1))

        ax.set_axis_off()

        elev = self.slider_elev.value()
        azim = self.slider_azim.value()
        ax.view_init(elev=elev, azim=azim)

        legend = None
        if getattr(self, "show_3d_legend", False):
            legend = ax.legend(
                loc="lower right", 
                bbox_to_anchor=(0.98, 0.05),
                fontsize=8, 
                facecolor="#080808",
                edgecolor="#262626", 
                labelcolor="#c9d1d9", 
                framealpha=0.90,
                handlelength=1.2, 
                handleheight=1.0,
                markerscale=0.55, 
                labelspacing=0.3, 
                borderpad=0.5
            )
        if legend:
            legend.set_zorder(100)

        self.canvas.draw()


def main():
    default_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_shoulder_elbow.urdf")
    urdf_path = sys.argv[1] if len(sys.argv) > 1 else default_path
    if not os.path.isfile(urdf_path):
        print(f"ERROR: URDF file not found at: {urdf_path}")
        sys.exit(1)

    app = QApplication(sys.argv)
    window = MainWindow(urdf_path)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
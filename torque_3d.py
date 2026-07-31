"""
torque_3d.py
=============
Calculo de torque gravitacional 3D COMPLETO (producto cruzado tau=(r x F)*eje)
para reemplazar la aproximacion seno/coseno de un solo eje del simulador
original (urdf_arm_simulator_EN_Final.py, funcion update_all()).

POR QUE ESTE CAMBIO:
--------------------
La formula original:
    torque_elbow   = (f_mass*g*(f_len/2) + p_mass*g*f_len) * sin(elbow_flex)
    torque_sh_flex = (...) * sin(sh_flex) + torque_elbow
    torque_sh_abd  = (...) * cos(sh_flex) * sin(sh_abd)

asume implicitamente que el hombro esta en reposo para el torque de codo
(no depende de sh_abd/sh_flex), y captura el acoplamiento hombro-hombro
con un UNICO termino cos(sh_flex) -- ambas limitaciones ya estaban
documentadas en la Seccion 12 de tu documentacion original, y confirmadas
numericamente al compararlas con OpenSim (Pasos 8 y 8b del tutorial).

Este modulo reemplaza esas formulas por el calculo vectorial completo:
    tau_eje = (r_CoM_desde_pivote x F_gravedad) . eje_articulacion_mundo

sumado sobre TODAS las masas distales relevantes (antebrazo, payload,
brazo superior segun corresponda), usando la MISMA aproximacion
antropometrica que ya tenias (masa concentrada a mitad de segmento,
payload en la punta) -- asi que cualquier diferencia contra la formula
original es PURAMENTE geometrica/de acoplamiento, no antropometrica
(esa comparacion ya se hizo por separado contra OpenSim).

Esta funcion NO depende de OpenSim -- solo usa la cinematica que tu
simulador ya calcula (transforms, joints), asi que corre en tiempo real
sin ningun costo extra relevante.
"""

import math
import numpy as np


def _joint_axis_world(joints, joint_name, transforms, links_geom_root_frame):
    """Obtiene el eje de una articulacion, rotado al marco del mundo segun
    la orientacion de su link PADRE (el eje esta definido en el URDF en el
    marco del padre de la articulacion)."""
    joint = next(j for j in joints if j["name"] == joint_name)
    parent_link = joint["parent"]
    R_parent = transforms.get(parent_link, np.eye(4))[:3, :3]
    axis_local = joint["axis"]
    axis_world = R_parent @ axis_local
    return axis_world / np.linalg.norm(axis_world)


def compute_full_3d_torques(transforms, joints, u_len, f_len, u_mass, f_mass, p_mass, g=9.81):
    """Calcula los 3 torques gravitacionales (shoulder_flex, shoulder_abd,
    elbow) usando el producto cruzado 3D completo, en vez de la
    aproximacion seno/coseno de un solo eje.

    Parametros:
        transforms: dict {link_name: matriz 4x4 mundo}, tal como ya lo
                    calcula tu MainWindow.compute_world_transforms(...)
        joints: lista de dicts de articulaciones parseadas del URDF
                (tal como los devuelve parse_urdf), cada uno con
                'name', 'parent', 'child', 'axis' (en el marco del padre)
        u_len, f_len: longitudes de brazo superior/antebrazo (m)
        u_mass, f_mass, p_mass: masas de brazo superior/antebrazo/payload (kg)
        g: aceleracion gravitacional (m/s^2)

    Devuelve: dict con 'sh_flex', 'sh_abd', 'elbow' (Nm), usando la MISMA
    convencion de signo que la formula original (torque de sostenimiento,
    positivo cuando la gravedad tiende a aumentar el angulo).
    """
    T_shoulder_dummy1 = transforms.get("shoulder_dummy_1", np.eye(4))
    T_upper = transforms.get("upper_arm", np.eye(4))
    T_forearm = transforms.get("forearm", np.eye(4))

    shoulder_pivot = T_upper[:3, 3]       # pivote hombro (shoulder_rot origin=0)
    elbow_pivot = T_forearm[:3, 3]        # pivote codo

    R_upper = T_upper[:3, :3]
    R_forearm = T_forearm[:3, :3]

    # Centros de masa (misma convencion que la formula original: masa a
    # la mitad del segmento; payload en la punta del antebrazo)
    com_upper = shoulder_pivot + R_upper @ np.array([0, 0, -u_len / 2.0])
    com_forearm = elbow_pivot + R_forearm @ np.array([0, 0, -f_len / 2.0])
    pos_payload = elbow_pivot + R_forearm @ np.array([0, 0, -f_len])

    # Fuerzas de gravedad (convencion URDF: Z = vertical, gravedad en -Z)
    F_upper = np.array([0, 0, -u_mass * g])
    F_forearm = np.array([0, 0, -f_mass * g])
    F_payload = np.array([0, 0, -p_mass * g])

    # Ejes de las 3 articulaciones, en el marco del mundo
    axis_abd = _joint_axis_world(joints, "shoulder_abd", transforms, None)
    axis_flex = _joint_axis_world(joints, "shoulder_flex", transforms, None)
    axis_elbow = _joint_axis_world(joints, "elbow_flex", transforms, None)

    # --- Torque de codo: solo antebrazo + payload, medidos desde el codo ---
    # NOTA DE SIGNO: (r x F) da el torque que la GRAVEDAD ejerce sobre la
    # articulacion; el torque que los musculos/motor deben aplicar para
    # SOSTENER la postura (la misma convencion que usa tu formula original)
    # es el opuesto -- de ahi el signo negativo aqui (mismo patron de
    # signo que ya se encontro al comparar contra OpenSim, Seccion 11.3
    # del tutorial).
    r_forearm_e = com_forearm - elbow_pivot
    r_payload_e = pos_payload - elbow_pivot
    moment_elbow = np.cross(r_forearm_e, F_forearm) + np.cross(r_payload_e, F_payload)
    tau_elbow = -float(np.dot(moment_elbow, axis_elbow))

    # --- Torques de hombro: TODAS las masas distales, medidas desde el hombro ---
    r_upper_s = com_upper - shoulder_pivot
    r_forearm_s = com_forearm - shoulder_pivot
    r_payload_s = pos_payload - shoulder_pivot
    moment_shoulder = (np.cross(r_upper_s, F_upper)
                       + np.cross(r_forearm_s, F_forearm)
                       + np.cross(r_payload_s, F_payload))
    tau_sh_flex = -float(np.dot(moment_shoulder, axis_flex))
    tau_sh_abd = -float(np.dot(moment_shoulder, axis_abd))

    return {"sh_flex": tau_sh_flex, "sh_abd": tau_sh_abd, "elbow": tau_elbow}

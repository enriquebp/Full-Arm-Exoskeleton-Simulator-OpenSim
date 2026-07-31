"""
bone_viewer.py
================
Etapa 1 de la Opcion B (visualizacion realista con PyVista, ver tutorial
Seccion 24): parsea la asociacion body -> archivo(s) .vtp del .osim, y
calcula la transformacion 4x4 en el mundo para cada malla, dada la
postura actual del modelo OpenSim.

Deliberadamente NO depende de PyVista -- solo de opensim y numpy -- para
poder validar esta capa de datos con un script de consola antes de tocar
la GUI (mismo patron que se uso para opensim_bridge.py).
"""

import os
import math
import xml.etree.ElementTree as ET
import numpy as np


def parse_body_meshes(osim_path):
    """Recorre el .osim y devuelve {nombre_body: [ {file, transform_xyzrpy, scale}, ... ]}.

    Cada entrada de malla tiene:
      file: nombre del archivo .vtp (relativo a la carpeta Geometry/)
      transform: [rX, rY, rZ, tx, ty, tz] -- rotacion XYZ (radianes) +
                 traslacion (m), relativa al marco del propio Body
      scale: [sx, sy, sz]
    """
    tree = ET.parse(osim_path)
    root = tree.getroot()
    body_meshes = {}

    for body in root.iter("Body"):
        name = body.get("name")
        vis = body.find("VisibleObject")
        if vis is None:
            continue
        meshes = []
        for dg in vis.findall("./GeometrySet/objects/DisplayGeometry"):
            gfile = dg.findtext("geometry_file")
            if not gfile or not gfile.strip():
                continue
            t_text = dg.findtext("transform")
            s_text = dg.findtext("scale_factors")
            transform = [float(x) for x in t_text.split()] if t_text else [0.0] * 6
            scale = [float(x) for x in s_text.split()] if s_text else [1.0, 1.0, 1.0]
            meshes.append({"file": gfile.strip(), "transform": transform, "scale": scale})
        if meshes:
            body_meshes[name] = meshes

    return body_meshes


def _xyz_euler_matrix(rx, ry, rz):
    """Matriz de rotacion a partir de angulos de Euler X-Y-Z (convencion
    de OpenSim para el 'transform' de VisibleObject): R = Rz @ Ry @ Rx.
    """
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def mesh_local_transform_matrix(mesh_entry):
    """Matriz 4x4 del 'transform' local de una malla (relativo a su Body)."""
    rx, ry, rz, tx, ty, tz = mesh_entry["transform"]
    T = np.eye(4)
    T[:3, :3] = _xyz_euler_matrix(rx, ry, rz)
    T[:3, 3] = [tx, ty, tz]
    return T


def get_all_mesh_world_transforms(osim_model, body_meshes, side="r"):
    """Dada la postura ACTUAL de osim_model (osim_model.state ya debe estar
    en la pose deseada, via osim_model.set_pose(...)), calcula la
    transformacion 4x4 en el mundo (ground) de cada malla del lado dado.

    Devuelve: lista de dicts {body, file, world_transform (4x4 np.array), scale}
    """
    import opensim as osim

    model = osim_model.model
    state = osim_model.state
    ground = model.getGround()
    suffix = f"_{side}"

    results = []
    for body_name, meshes in body_meshes.items():
        if not body_name.endswith(suffix):
            continue
        try:
            body = model.getBodySet().get(body_name)
        except Exception:
            continue

        body_transform_osim = body.getTransformInGround(state)
        R = body_transform_osim.R()
        p = body_transform_osim.p()
        T_body = np.eye(4)
        for i in range(3):
            for j in range(3):
                T_body[i, j] = R.get(i, j)
            T_body[i, 3] = p.get(i)

        for mesh_entry in meshes:
            T_local = mesh_local_transform_matrix(mesh_entry)
            T_world = T_body @ T_local
            results.append({
                "body": body_name,
                "file": mesh_entry["file"],
                "world_transform": T_world,
                "scale": mesh_entry["scale"],
            })

    return results

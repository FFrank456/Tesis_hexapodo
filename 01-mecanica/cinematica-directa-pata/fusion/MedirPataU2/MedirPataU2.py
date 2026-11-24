"""Lee la configuración actual del ensamble de la U2 y escribe un JSON local."""
import json
from pathlib import Path
import traceback

import adsk.core
import adsk.fusion


DESIGN_NAME = "ROBOT_ESAMBLE_alfred_mejorado"
OUTPUT = Path(__file__).resolve().parents[2] / "medicion_fusion_u2.json"
MARKER_IMAGE = Path(__file__).with_name("apoyo.png")
marker = None


def mm(point):
    return [10 * value for value in point.asArray()]


def measure_joint(joint, as_built):
    motion = adsk.fusion.RevoluteJointMotion.cast(joint.jointMotion)
    one = joint.transform if as_built else joint.geometryOneTransform
    two = None if as_built else joint.geometryTwoTransform
    return {
        "nombre": joint.name,
        "ocurrencias": [
            occurrence.fullPathName if occurrence else None
            for occurrence in (joint.occurrenceOne, joint.occurrenceTwo)
        ],
        "suprimida": joint.isSuppressed,
        "origen_mm": mm(one.translation) if one else None,
        "origen_dos_mm": mm(two.translation) if two else None,
        "eje": motion.rotationAxisVector.asArray() if motion else None,
        "angulo_rad": motion.rotationValue if motion else None,
    }


def measure_tibia(occurrence):
    transform = occurrence.transform2
    bodies = []
    for body in occurrence.component.bRepBodies:
        calculator = body.meshManager.createMeshCalculator()
        mesh = calculator.calculate()
        vertices = []
        for point in mesh.nodeCoordinates:
            point.transformBy(transform)
            vertices.append(mm(point))
        if vertices:
            bodies.append({
                "nombre": body.name,
                "min_mm": [min(coords) for coords in zip(*vertices)],
                "max_mm": [max(coords) for coords in zip(*vertices)],
                "min_z_mm": min(vertices, key=lambda p: p[2]),
            })
    return bodies


def mark_support(root, point_mm):
    global marker
    if marker and marker.isValid:
        marker.deleteMe()
    marker = root.customGraphicsGroups.add()
    coordinates = adsk.fusion.CustomGraphicsCoordinates.create(
        [value / 10 for value in point_mm]
    )
    points = marker.addPointSet(
        coordinates, [],
        adsk.fusion.CustomGraphicsPointTypes.UserDefinedCustomGraphicsPointType,
        str(MARKER_IMAGE),
    )
    if not points:
        raise RuntimeError("Fusion no pudo dibujar el marcador del pie.")


def run(context):
    app = adsk.core.Application.get()
    try:
        document = app.activeDocument
        if not document or DESIGN_NAME not in document.name:
            raise ValueError(f"Abre el ensamble {DESIGN_NAME} antes de medir.")
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent
        tibias = [
            item for item in root.occurrences
            if item.component.name.startswith("tibia_link_1")
        ]
        data = {
            "documento": document.name,
            "referencia": "Sistema de coordenadas raíz de Fusion, pose actual",
            "uniones": [measure_joint(joint, False) for joint in root.joints]
            + [measure_joint(joint, True) for joint in root.asBuiltJoints],
            "tibias": [
                {"nombre": item.fullPathName, "cuerpos": measure_tibia(item)}
                for item in tibias
            ],
        }
        sole, = [
            body for tibia in data["tibias"] for body in tibia["cuerpos"]
            if body["nombre"] == "soporte_tibia_A"
        ]
        data["pie_nominal_mm"] = sole["min_z_mm"]
        mark_support(root, data["pie_nominal_mm"])
        app.activeViewport.refresh()
        OUTPUT.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        app.userInterface.messageBox("Medición guardada en " + str(OUTPUT))
    except Exception:
        app.userInterface.messageBox(traceback.format_exc())

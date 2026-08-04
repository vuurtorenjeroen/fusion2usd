#!/usr/bin/env python3

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf, Sdf
import trimesh
import numpy as np
import json
import yaml
from pathlib import Path
import os.path, sys, os, getopt
import re
import pymeshlab


def usd_safe_name(name):
    # also possible to use this, but replaces a digit at begin with _
    # name = Tf.MakeValidIdentifier(name)

    # Replace every non-alphanumeric/underscore character with '_'
    name = re.sub(r'[^A-Za-z0-9_]', '_', name)

    # USD identifiers cannot start with a digit
    if name and name[0].isdigit():
        name = "_" + name

    return name


# ------------------------------------------------------------
# STL loader
# ------------------------------------------------------------
# def load_stl(path):
#     mesh = trimesh.load_mesh(path)
#     return np.array(mesh.vertices / 10.0), np.array(mesh.faces)

def load_stl(path, target_faces=50000):
    ms = pymeshlab.MeshSet()

    # Load STL
    ms.load_new_mesh(path)

    mesh = ms.current_mesh()

    print(
        f"{path}: before decimation: "
        f"{mesh.vertex_number()} vertices, "
        f"{mesh.face_number()} faces"
    )

    # Decimate if needed
    if mesh.face_number() > target_faces:
        ms.apply_filter(
            "meshing_decimation_quadric_edge_collapse",
            targetfacenum=target_faces,
            preservenormal=True,
            preservetopology=True,
        )

    mesh = ms.current_mesh()

    print(
        f"{path}: after decimation: "
        f"{mesh.vertex_number()} vertices, "
        f"{mesh.face_number()} faces"
    )

    vertices = np.array(mesh.vertex_matrix())
    faces = np.array(mesh.face_matrix())

    # USD is in cm, stl is in mm
    vertices = vertices / 10.0

    return vertices, faces


# ------------------------------------------------------------
# Material system
# ------------------------------------------------------------
def create_material(rootname, stage, name, rgb):
    mat = UsdShade.Material.Define(stage, f"/{rootname}/Looks/{name}")
    shader = UsdShade.Shader.Define(stage, f"/{rootname}/Looks/{name}/shader")

    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(rgb)

    mat.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(),
        "surface"
    )
    return mat


# ------------------------------------------------------------
# Link builder with full physics
# ------------------------------------------------------------
def create_link(rootname, stage, name, verts, faces, inertial, material, verts_col, faces_col):
    name = usd_safe_name(name)
    path = f"/{rootname}/{name}"

    link = UsdGeom.Xform.Define(stage, path)

    # Physics
    rb = UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
    # UsdPhysics.CollisionAPI.Apply(link.GetPrim())

    # Mass properties (CRITICAL for RL stability)
    mass_api = UsdPhysics.MassAPI.Apply(link.GetPrim())

    mass_api.CreateMassAttr(inertial["mass"])

    com = inertial["center_of_mass"]
    mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*com))

    inertia = [_*inertial["mass"] for _ in inertial["inertia"]]

    mass_api.CreateDiagonalInertiaAttr(
        Gf.Vec3f(inertia[0], inertia[1], inertia[2])
    )

    # NOTE: principal axes are optional; identity is safest
    mass_api.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))

    # -------------------------
    # Visual mesh
    # -------------------------
    if inertial.get("create_visual_mesh", True):
        vis = UsdGeom.Mesh.Define(stage, f"{path}/visuals/{name}")
        vis.CreatePointsAttr([Gf.Vec3f(*v) for v in verts])
        vis.CreateFaceVertexCountsAttr([3] * len(faces))
        vis.CreateFaceVertexIndicesAttr(faces.flatten().tolist())
        vis.CreateSubdivisionSchemeAttr("none")

        # Mark this as a renderable visual mesh
        vis.CreatePurposeAttr("render")

        # -------------------------
        # Material
        # -------------------------
        UsdShade.MaterialBindingAPI(vis).Bind(material)
        UsdShade.MaterialBindingAPI.Apply(vis.GetPrim())


    # -------------------------
    # Collision mesh (explicit)
    # -------------------------
    if inertial.get("create_collision_mesh", True):
        col = UsdGeom.Mesh.Define(stage, f"{path}/collisions/{name}")
        # col.CreatePointsAttr([Gf.Vec3f(*v) for v in verts])
        # col.CreateFaceVertexCountsAttr([3] * len(faces))
        # col.CreateFaceVertexIndicesAttr(faces.flatten().tolist())
        col.CreatePointsAttr([Gf.Vec3f(*v) for v in verts_col])
        col.CreateFaceVertexCountsAttr([3] * len(faces_col))
        col.CreateFaceVertexIndicesAttr(faces_col.flatten().tolist())
        col.CreateSubdivisionSchemeAttr("none")

        # Mark as collision/proxy geometry
        col.CreatePurposeAttr("proxy")

        # Hide it in the viewport while keeping it available for physics
        col.CreateVisibilityAttr("invisible")

        UsdPhysics.CollisionAPI.Apply(col.GetPrim())    #TODO do we need this one?
        UsdPhysics.MeshCollisionAPI.Apply(col.GetPrim())
        UsdPhysics.MeshCollisionAPI(col.GetPrim()).CreateApproximationAttr("convexHull")
        # UsdPhysics.MeshCollisionAPI(col.GetPrim()).CreateApproximationAttr("convexDecomposition")


    return link


# ------------------------------------------------------------
# Joint builder (stable DOF ordering)
# ------------------------------------------------------------
def create_root_joint(rootname, stage):
    path = f"/{rootname}/root_joint"
    joint = UsdPhysics.FixedJoint.Define(stage, path)
    UsdPhysics.ArticulationRootAPI.Apply(joint.GetPrim())

    joint.CreateBody0Rel().SetTargets([f"/{rootname}"])
    joint.CreateBody1Rel().SetTargets([f"/{rootname}/base_link"])

    joint.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))

    #TODO check if we need this as well
    # PhysxSchema is not available in pxr as it's an isaac thing
    # physx_art = PhysxSchema.PhysxArticulationAPI.Apply(joint)
    # physx_art.CreateEnabledSelfCollisionsAttr(False)
    # prepend apiSchemas = ["PhysxArticulationAPI"]
    # bool physxArticulation:enabledSelfCollisions = 0
    # int physxArticulation:solverPositionIterationCount = 32
    # int physxArticulation:solverVelocityIterationCount = 1

    return joint


def create_joint(rootname, stage, name, jdata, inertial):
    jtype = jdata["type"]
    parent = jdata["parent"]
    child = jdata["child"]
    name = usd_safe_name(name)
    parent = usd_safe_name(parent)
    child = usd_safe_name(child)

    path = f"/{rootname}/joints/{name}"

    if jtype == "continuous" or jtype == "revolute":
        joint = UsdPhysics.RevoluteJoint.Define(stage, path)
        # Rotation axis
        joint.CreateAxisAttr(
            jdata["usd_axis"]
        )

        if jtype == "revolute":
            joint.CreateLowerLimitAttr(jdata["lower_limit"])
            joint.CreateUpperLimitAttr(jdata["upper_limit"])

        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        if "drive" in jdata:
            drivedata = jdata["drive"]
            if "type" in drivedata:
                drive.CreateTypeAttr(drivedata["type"])
            if "damping" in drivedata:
                drive.CreateDampingAttr(drivedata["damping"])
            if "maxforce" in drivedata:
                drive.CreateMaxForceAttr(drivedata["maxforce"])
            if "stiffness" in drivedata:
                drive.CreateStiffnessAttr(drivedata["stiffness"])
            if "targetposition" in drivedata:
                drive.CreateTargetPositionAttr(drivedata["targetposition"])  # degrees


    elif jtype == "fixed":
        joint = UsdPhysics.FixedJoint.Define(stage, path)

    else:
        raise ValueError(f"Unknown joint type: {jtype}")

    joint.CreateBody0Rel().SetTargets([f"/{rootname}/{parent}"])
    joint.CreateBody1Rel().SetTargets([f"/{rootname}/{child}"])

    # ---------------------------------------
    # Joint position
    # ---------------------------------------
    parent_pos = jdata.get(
        "parent_xyz",
        [0,0,0]
    )

    child_pos = jdata.get(
        "child_xyz",
        [0,0,0]
    )


    joint.CreateLocalPos0Attr(
        Gf.Vec3f(
            parent_pos[0],
            parent_pos[1],
            parent_pos[2]
        )
    )


    joint.CreateLocalPos1Attr(
        Gf.Vec3f(
            child_pos[0],
            child_pos[1],
            child_pos[2]
        )
    )

    # -------------------------------
    # Joint orientation
    # -------------------------------
    parent_rot = jdata.get(
        "parent_orientation",
        [1,0,0,0]
    )

    child_rot = jdata.get(
        "child_orientation",
        [1,0,0,0]
    )

    joint.CreateLocalRot0Attr(
        Gf.Quatf(
            parent_rot[0],
            Gf.Vec3f(
                parent_rot[1],
                parent_rot[2],
                parent_rot[3]
            )
        )
    )

    joint.CreateLocalRot1Attr(
        Gf.Quatf(
            child_rot[0],
            Gf.Vec3f(
                child_rot[1],
                child_rot[2],
                child_rot[3]
            )
        )
    )


    return joint


# ------------------------------------------------------------
# Merge configs
# ------------------------------------------------------------
def deep_merge(base, override):
    """Recursively merge override into base."""
    result = base.copy()

    for key, value in override.items():
        if value == None:
            continue
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            # Both values are dicts, merge recursively
            result[key] = deep_merge(result[key], value)
        else:
            # Override the value
            result[key] = value

    return result


# ------------------------------------------------------------
# Usage
# ------------------------------------------------------------
def print_usage(exit_code = 0):
    print("Usage: %s [-o <output>] <input.json>" % 'fusion2usd.py')
    print("       %s -o             Output file    (default: input.usda)" % 'fusion2usd.py')
    print("       %s --meshes       Meshes folder  (default: meshes)" % 'fusion2usd.py')
    print("       %s --overrides    Overrides file (default: input-overrides.json)" % 'fusion2usd.py')
    sys.exit(exit_code)


# ------------------------------------------------------------
# MAIN
# ------------------------------------------------------------
def main():
    try:
        opts, args = getopt.gnu_getopt(sys.argv[1:], "ho:", ['meshes=', 'overrides='])
    except getopt.GetoptError as err:
        print(str(err))
        print_usage(2)

    if len(args) < 1:
        print("No input given")
        print_usage(2)

    path = os.path.dirname(args[0])
    rootname = os.path.splitext(os.path.basename(args[0]))[0]
    inputfile = args[0]
    output = os.path.join(path, rootname + ".usda")
    meshes_folder = os.path.join(path, "meshes")
    overrides = os.path.join(path, rootname + "-overrides.yaml")

    for o, a in opts:
        if o == '-h':
            print_usage(0)
        elif o == '-o':
            output = a
        elif o == '--meshes':
            meshes_folder = a
        elif o == '--overrides':
            overrides = a

    rootname = usd_safe_name(rootname)
    # print(f"rootname: {rootname}")
    # print(f"inputfile: {inputfile}")
    # print(f"output: {output}")
    # print(f"meshes: {meshes_folder}")
    # print(f"overrides: {overrides}")


    with open(inputfile) as f:
        data = json.load(f)

    if not Path(overrides).exists():
        print(f"[WARN] missing override file: {overrides}")
    else:
        with open(overrides) as f:
            data2 = yaml.load(f, Loader=yaml.SafeLoader)
            # data2 = json.load(f)
        data = deep_merge(data, data2)

    stage = Usd.Stage.CreateNew(output)

    root = UsdGeom.Xform.Define(stage, f"/{rootname}")
    # UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    create_root_joint(rootname, stage)

    # IMPORTANT: Scope (not Xform)
    UsdGeom.Scope.Define(stage, f"/{rootname}/joints")
    UsdGeom.Scope.Define(stage, f"/{rootname}/Looks")


    # ------------------------------------------------------------
    # Materials
    # ------------------------------------------------------------

    materials = {}
    for name, rgb_str in data["color"].items():
        rgb = tuple(float(x) for x in rgb_str.split()[:3])
        materials[name] = create_material(rootname, stage, name, rgb)


    # ------------------------------------------------------------
    # Load meshes (STL assumption)
    # ------------------------------------------------------------

    meshes = {}
    meshes_col = {}
    for link_name in data["inertial"].keys():
        stl_path = os.path.join(meshes_folder, f"{link_name}.stl")

        if not Path(stl_path).exists():
            print(f"[WARN] missing STL: {stl_path}")
            continue

        if not data["inertial"][link_name].get("create_link", True):
            # print(f"skipping link: {name}")
            continue

        vis_faces = data["inertial"][link_name].get("visual_mesh_faces", 50000)
        col_faces = data["inertial"][link_name].get("collision_mesh_faces", 500)
        meshes[link_name] = load_stl(stl_path, target_faces=vis_faces)
        meshes_col[link_name] = load_stl(stl_path, target_faces=col_faces)


    # ------------------------------------------------------------
    # Build links (from inertial dict)
    # ------------------------------------------------------------

    links = {}

    for name, inertial in data["inertial"].items():
        if name not in meshes:
            continue
        if not inertial.get("create_link", True):
            # print(f"skipping link: {name}")
            continue

        verts, faces = meshes[name]
        verts_col, faces_col = meshes_col[name]

        mat_name = data["material"].get(name, {}).get("material", None)
        material = materials.get(mat_name, None)

        links[name] = create_link(rootname, stage, name, verts, faces, inertial, material, verts_col, faces_col)


    # ------------------------------------------------------------
    # Build joints (STRICT ORDER = JSON order)
    # ------------------------------------------------------------

    for jname, jdata in data["joints"].items():
        if not jdata.get("create_joint", True):
            continue
        create_joint(rootname, stage, jname, jdata, data["inertial"])


    # ------------------------------------------------------------
    # Physics units (IMPORTANT for your cm model)
    # ------------------------------------------------------------

    UsdGeom.SetStageMetersPerUnit(stage, 0.01)  # cm
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)  # Set to Z-up

    stage.SetDefaultPrim(root.GetPrim())

    stage.GetRootLayer().Save()

    print(f"DONE: {output}")



if __name__ == "__main__":
	main()

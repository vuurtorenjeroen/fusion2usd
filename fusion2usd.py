#!/usr/bin/env python3

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf, Sdf
import trimesh
import numpy as np
import json
from pathlib import Path
import os.path, sys, os, getopt


# ------------------------------------------------------------
# STL loader
# ------------------------------------------------------------
def load_stl(path):
    mesh = trimesh.load_mesh(path)
    return np.array(mesh.vertices / 10.0), np.array(mesh.faces)


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
def create_link(rootname, stage, name, verts, faces, inertial, material):
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

    inertia = inertial["inertia"]

    mass_api.CreateDiagonalInertiaAttr(
        Gf.Vec3f(inertia[0], inertia[1], inertia[2])
    )

    # NOTE: principal axes are optional; identity is safest
    mass_api.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))

    # -------------------------
    # Visual mesh
    # -------------------------
    vis = UsdGeom.Mesh.Define(stage, f"{path}/visuals/{name}")
    vis.CreatePointsAttr([Gf.Vec3f(*v) for v in verts])
    vis.CreateFaceVertexCountsAttr([3] * len(faces))
    vis.CreateFaceVertexIndicesAttr(faces.flatten().tolist())
    vis.CreateSubdivisionSchemeAttr("none")

    # -------------------------
    # Collision mesh (explicit)
    # -------------------------
    col = UsdGeom.Mesh.Define(stage, f"{path}/collisions/{name}")
    col.CreatePointsAttr([Gf.Vec3f(*v) for v in verts])
    col.CreateFaceVertexCountsAttr([3] * len(faces))
    col.CreateFaceVertexIndicesAttr(faces.flatten().tolist())
    col.CreateSubdivisionSchemeAttr("none")

    UsdPhysics.CollisionAPI.Apply(col.GetPrim())    #TODO do we need this one?
    UsdPhysics.MeshCollisionAPI.Apply(col.GetPrim())
    UsdPhysics.MeshCollisionAPI(col.GetPrim()).CreateApproximationAttr("convexHull")

    # -------------------------
    # Material
    # -------------------------
    UsdShade.MaterialBindingAPI(vis).Bind(material)
    UsdShade.MaterialBindingAPI.Apply(vis.GetPrim())

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
    # prepend apiSchemas = ["PhysxArticulationAPI"]
    # bool physxArticulation:enabledSelfCollisions = 0
    # int physxArticulation:solverPositionIterationCount = 32
    # int physxArticulation:solverVelocityIterationCount = 1

    return joint


def create_joint(rootname, stage, name, jdata):
    jtype = jdata["type"]
    parent = jdata["parent"]
    child = jdata["child"]
    axis = jdata["axis"]

    path = f"/{rootname}/joints/{name.replace(' ', '_')}"

    if jtype == "continuous" or jtype == "revolute":
        joint = UsdPhysics.RevoluteJoint.Define(stage, path)
        # Rotation axis
        # TODO check rotation axis when angled
        if axis[0] == 1.0:
            joint.CreateAxisAttr("X")
        elif axis[1] == 1.0:
            joint.CreateAxisAttr("Y")
        elif axis[2] == 1.0:
            joint.CreateAxisAttr("Z")

        # TODO check min/max position when revolute
        if jtype == "revolute":
            joint.CreateLowerLimitAttr(jdata["lower_limit"])
            joint.CreateUpperLimitAttr(jdata["upper_limit"])

        # TODO make all parameters optional
        if "drive" in jdata:
            drivedata = jdata["drive"]

            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
            drive.CreateTypeAttr(drivedata["type"])
            drive.CreateDampingAttr(drivedata["damping"])
            drive.CreateMaxForceAttr(drivedata["maxforce"])
            drive.CreateStiffnessAttr(drivedata["stiffness"])
            drive.CreateTargetPositionAttr(drivedata["targetposition"])  # degrees


    elif jtype == "fixed":
        joint = UsdPhysics.FixedJoint.Define(stage, path)

    else:
        raise ValueError(f"Unknown joint type: {jtype}")

    joint.CreateBody0Rel().SetTargets([f"/{rootname}/{parent}"])
    joint.CreateBody1Rel().SetTargets([f"/{rootname}/{child}"])

    # TODO this should be using joint offset instead of xyz or we need to add a lot of translates ...
    # joint.CreateLocalPos0Attr(Gf.Vec3f(*jdata["xyz"]))
    joint.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))

    return joint


# ------------------------------------------------------------
# Merge configs
# ------------------------------------------------------------
def deep_merge(base, override):
    """Recursively merge override into base."""
    result = base.copy()

    for key, value in override.items():
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
    overrides = os.path.join(path, rootname + "-overrides.json")

    for o, a in opts:
        if o == '-h':
            print_usage(0)
        elif o == '-o':
            output = a
        elif o == '--meshes':
            meshes_folder = a
        elif o == '--overrides':
            overrides = a

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
            data2 = json.load(f)
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
    for link_name in data["inertial"].keys():
        stl_path = os.path.join(meshes_folder, f"{link_name}.stl")

        if not Path(stl_path).exists():
            print(f"[WARN] missing STL: {stl_path}")
            continue

        meshes[link_name] = load_stl(stl_path)


    # ------------------------------------------------------------
    # Build links (from inertial dict)
    # ------------------------------------------------------------

    links = {}

    for name, inertial in data["inertial"].items():
        if name not in meshes:
            continue

        verts, faces = meshes[name]

        mat_name = data["material"].get(name, {}).get("material", None)
        material = materials.get(mat_name, None)

        links[name] = create_link(rootname, stage, name, verts, faces, inertial, material)


    # ------------------------------------------------------------
    # Build joints (STRICT ORDER = JSON order)
    # ------------------------------------------------------------

    for jname, jdata in data["joints"].items():
        create_joint(rootname, stage, jname, jdata)


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

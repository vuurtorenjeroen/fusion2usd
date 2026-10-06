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
# Convex hull collision meshes
# ------------------------------------------------------------
# Collision meshes use the "convexHull" approximation, so PhysX only ever
# sees the convex hull of the collision mesh's points. load_stl()'s quadric
# edge-collapse decimation (collision_mesh_faces, default 500) MOVES
# vertices, and mirrored parts decimate differently: on the qmini feet it
# produced a vertex 3 mm below the real sole on the right foot only (a
# rocker), while the left foot rested on its toe/heel tips -- the sim
# learned a left/right asymmetric gait from that (2026-10-06).
#
# collision_mode: "hull" instead builds the collision mesh from the EXACT
# convex hull of the full-resolution STL. No vertex is moved; if the hull
# has more than collision_hull_max_vertices (PhysX GPU convex hulls are
# limited to 64), vertices are SELECTED (never displaced): the bottom
# (lowest collision_hull_bottom_band_mm) is kept in detail -- its outline
# and its lateral profile along the part, which is what touches the ground
# -- and the rest is farthest-point sampled.
#
# Optional collision_contact_patch (for feet) models a compliant sole
# layer: STL points within tolerance_mm of the lowest point and within
# +-half_width_mm (along X) of the sole centreline are flattened onto the
# lowest z. Assumes Z up and the foot's length along Y (true for this
# Fusion export). Example (qmini feet, measured real flat part ~100x12 mm):
#   Left_Foot_1:
#     collision_mode: hull
#     collision_contact_patch: {tolerance_mm: 1.3, half_width_mm: 6.0}

def _convex_hull(points):
    """Exact convex hull of a point cloud via pymeshlab (qhull):
    returns (vertices (V,3), triangles (F,3)) with outward winding."""
    ms = pymeshlab.MeshSet()
    ms.add_mesh(pymeshlab.Mesh(vertex_matrix=np.asarray(points, dtype=np.float64)))
    ms.generate_convex_hull()
    m = ms.current_mesh()
    v = np.array(m.vertex_matrix())
    f = np.array(m.face_matrix())
    c = v.mean(axis=0)
    for i, (a, b, d) in enumerate(f):
        n = np.cross(v[b] - v[a], v[d] - v[a])
        if np.dot(n, v[a] - c) < 0:
            f[i] = [a, d, b]
    return v, f


def _convex_hull_2d(points):
    """Andrew's monotone chain; returns the hull polygon (counter-clockwise)."""
    pts = sorted(set(map(tuple, np.round(points, 6))))
    if len(pts) <= 2:
        return np.array(pts)
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def _simplify_closed_polygon(poly, n):
    """Indices of the n vertices that best keep a closed 2D polygon's shape
    (repeatedly drops the vertex spanning the smallest triangle)."""
    idx = list(range(len(poly)))
    while len(idx) > n:
        areas = []
        for k in range(len(idx)):
            a, b, c = poly[idx[k - 1]], poly[idx[k]], poly[idx[(k + 1) % len(idx)]]
            areas.append(abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])))
        idx.pop(int(np.argmin(areas)))
    return idx


def load_stl_hull(path, max_vertices=60, contact_patch=None, bottom_band_mm=8.0,
                  band_bins=12, outline_vertices=16):
    """Collision mesh = exact convex hull of the full-resolution STL (see the
    comment block above). Returns (vertices_cm, faces) like load_stl()."""
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(path)
    P = np.array(ms.current_mesh().vertex_matrix())  # mm
    z0 = P[:, 2].min()

    patch_info = None
    if contact_patch:
        tol = float(contact_patch.get("tolerance_mm", 1.0))
        half_w = float(contact_patch.get("half_width_mm", 6.0))
        xc = P[P[:, 2] < z0 + 0.3, 0].mean()  # sole centreline
        patch = (P[:, 2] < z0 + tol) & (np.abs(P[:, 0] - xc) <= half_w)
        P = P.copy()
        P[patch, 2] = z0
        patch_info = (xc, P[patch, 1].min(), P[patch, 1].max(), np.ptp(P[patch, 0]))

    H, F = _convex_hull(P)
    if len(H) > max_vertices:
        keep = []
        # 1) outline of the lowest face (the contact patch, if any)
        sole = H[H[:, 2] <= z0 + 1e-6]
        if len(sole) >= 3:
            outline = _convex_hull_2d(sole[:, :2])
            for i in _simplify_closed_polygon(outline, outline_vertices):
                j = np.argmin(np.linalg.norm(sole[:, :2] - outline[i], axis=1))
                keep.append(sole[j])
        else:
            keep += list(sole)
        # 2) bottom band: per slice along Y the outermost points on both sides + the lowest
        band = H[(H[:, 2] > z0 + 1e-6) & (H[:, 2] < z0 + bottom_band_mm)]
        edges = np.linspace(H[:, 1].min(), H[:, 1].max(), band_bins + 1)
        for lo, hi in zip(edges[:-1], edges[1:]):
            b = band[(band[:, 1] >= lo) & (band[:, 1] < hi)]
            if len(b):
                for k in (np.argmin(b[:, 0]), np.argmax(b[:, 0]), np.argmin(b[:, 2])):
                    keep.append(b[k])
        chosen = list(np.unique(np.array(keep), axis=0))
        # 3) farthest-point sampling of everything above the band
        rest = H[H[:, 2] >= z0 + bottom_band_mm]
        while len(rest):
            if len(_convex_hull(np.array(chosen))[0]) >= max_vertices:
                break
            d = np.min(np.linalg.norm(rest[:, None, :] - np.array(chosen)[None, :, :], axis=2), axis=1)
            k = int(np.argmax(d))
            chosen.append(rest[k])
            rest = np.delete(rest, k, axis=0)
        while len(_convex_hull(np.array(chosen))[0]) > max_vertices:
            chosen.pop()
        H, F = _convex_hull(np.array(chosen))

    print(f"{path}: collision hull {len(H)} vertices, {len(F)} faces, lowest z {z0:.2f} mm")
    if patch_info:
        xc, y_front, y_rear, width = patch_info
        print(f"   contact patch {y_rear - y_front:.1f} x {width:.1f} mm; centreline x={xc:.2f} mm, "
              f"ends y={y_front:.2f} / {y_rear:.2f} mm, z={z0:.2f} mm  (base-frame heel/toe points, m: "
              f"({xc/1000:.5f}, {y_rear/1000:.5f}, {z0/1000:.5f}) / ({xc/1000:.5f}, {y_front/1000:.5f}, {z0/1000:.5f}))")
    # USD is in cm, stl is in mm
    return H / 10.0, F


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
    if data.get("create_root_joint", True):
        create_root_joint(rootname, stage)
    else:
        path = f"/{rootname}/root_joint"
        UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())

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
        link_cfg = data["inertial"][link_name]
        if link_cfg.get("collision_mode", "decimate") == "hull":
            # exact convex hull, see load_stl_hull's comment block
            meshes_col[link_name] = load_stl_hull(
                stl_path,
                max_vertices=link_cfg.get("collision_hull_max_vertices", 60),
                contact_patch=link_cfg.get("collision_contact_patch"),
                bottom_band_mm=link_cfg.get("collision_hull_bottom_band_mm", 8.0),
            )
        else:
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

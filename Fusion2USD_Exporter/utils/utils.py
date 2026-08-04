# -*- coding: utf-8 -*-
"""
Created on Sun May 12 19:15:34 2019
Modified on Sun Jan 17 2021

@author: syuntoku
@author: spacemaster85
"""

import adsk
import adsk.core
import adsk.fusion
import os.path
import re
from xml.etree import ElementTree
from xml.dom import minidom


def export_stl(_app, save_dir):
    """
    export stl files into "sace_dir/"


    Parameters
    ----------
    _app: adsk.core.Application.get()
    save_dir: str
        directory path to save
    """

    def traverse( occ):
    # recursive method to get all bodies from components and sub-components
        body = adsk.fusion.BRepBody.cast(None)
        liste = []
        if occ.childOccurrences:
            for child in occ.childOccurrences:
                liste = liste + traverse(child)
        liste = liste + [body for body in occ.bRepBodies]
        return liste


    des: adsk.fusion.Design = _app.activeProduct
    root: adsk.fusion.Component = des.rootComponent

    showBodies = []
    body = adsk.fusion.BRepBody.cast(None)
    if root.isBodiesFolderLightBulbOn:
        lst = [body for body in root.bRepBodies]
        if len(lst) > 0:
            showBodies.append(['root', lst])

        occ = adsk.fusion.Occurrence.cast(None)
        for occ in root.allOccurrences:
            if not occ.assemblyContext:
                lst = [body for body in occ.bRepBodies]
                if occ.childOccurrences:
                    for child in occ.childOccurrences:
                        lst = lst + traverse(child)
                if len(lst) > 0:
                    showBodies.append([occ.name, lst])

        # get clone body
        tmpBrepMng = adsk.fusion.TemporaryBRepManager.get()
        tmpBodies = []
        for name, bodies in showBodies:
            lst = [tmpBrepMng.copy(body) for body in bodies]
            if len(lst) > 0:
                tmpBodies.append([name, lst])

        # create export Doc - DirectDesign
        fusionDocType = adsk.core.DocumentTypes.FusionDesignDocumentType
        expDoc: adsk.fusion.FusionDocument = _app.documents.add(fusionDocType)
        expDes: adsk.fusion.Design = expDoc.design
        expDes.designType = adsk.fusion.DesignTypes.DirectDesignType

        # get export rootComponent
        expRoot: adsk.fusion.Component = expDes.rootComponent

        # paste clone body
        mat0 = adsk.core.Matrix3D.create()
        for name, bodies in tmpBodies:
            occ = expRoot.occurrences.addNewComponent(mat0)
            comp = occ.component
            comp.name = name
            for body in bodies:
                comp.bRepBodies.add(body)

        # export stl
        try:
            os.mkdir(save_dir + '/meshes')
        except:
            pass
        exportFolder = save_dir + '/meshes'

        exportMgr = des.exportManager
        for occ in expRoot.allOccurrences:
            if "base_link" in occ.component.name:
                expName = "base_link"
            else:
                expName = re.sub(r'[^A-Za-z0-9_]', '_', occ.component.name)
            expPath = os.path.join(exportFolder, '{}.stl'.format(expName))
            stlOpts = exportMgr.createSTLExportOptions(occ, expPath)

            exportMgr.execute(stlOpts)

        # remove export Doc
        expDoc.close(False)


# def export_stl(_app, save_dir):
#     """
#     Export STL files directly from Fusion occurrences.

#     Uses custom tessellation settings to reduce triangle count.
#     """

#     import os
#     import re
#     import time

#     des: adsk.fusion.Design = _app.activeProduct
#     root: adsk.fusion.Component = des.rootComponent

#     # Create export directory
#     exportFolder = os.path.join(save_dir, "meshes")
#     os.makedirs(exportFolder, exist_ok=True)

#     exportMgr = des.exportManager

#     t0 = time.time()

#     # Export all occurrences
#     for occ in root.allOccurrences:

#         # Skip empty occurrences
#         if occ.component.bRepBodies.count == 0:
#             continue

#         name = occ.component.name

#         if "base_link" in name:
#             expName = "base_link"
#         else:
#             expName = re.sub(r"[^A-Za-z0-9_]", "_", name)

#         expPath = os.path.join(
#             exportFolder,
#             f"{expName}.stl"
#         )

#         print(f"Exporting {expName}")

#         stlOpts = exportMgr.createSTLExportOptions(
#             occ,
#             expPath
#         )

#         # Custom mesh refinement
#         stlOpts.meshRefinement = (
#             adsk.fusion.MeshRefinementSettings.MeshRefinementCustom
#         )

#         # Larger values = fewer triangles
#         stlOpts.surfaceDeviation = 1.0       # mm
#         stlOpts.normalDeviation = 20.0       # degrees
#         stlOpts.maximumEdgeLength = 10.0     # mm

#         # Binary STL is smaller/faster
#         stlOpts.isBinaryFormat = True

#         print(
#             " settings:",
#             "surface=", stlOpts.surfaceDeviation,
#             "normal=", stlOpts.normalDeviation,
#             "edge=", stlOpts.maximumEdgeLength
#         )

#         exportMgr.execute(stlOpts)

#         print(
#             f"Finished {expName}, elapsed {time.time()-t0:.1f}s"
#         )

#     print(
#         f"STL export complete. Total time: {time.time()-t0:.1f}s"
#     )



def file_dialog(ui):
    """
    display the dialog to save the file
    """
    # Set styles of folder dialog.
    folderDlg = ui.createFolderDialog()
    folderDlg.title = 'Fusion Folder Dialog'

    # Show folder dialog
    dlgResult = folderDlg.showDialog()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        return folderDlg.folder
    return False


def origin2center_of_mass(inertia, center_of_mass, mass):
    """
    convert the moment of the inertia about the world coordinate into
    that about center of mass coordinate


    Parameters
    ----------
    moment of inertia about the world coordinate:  [xx, yy, zz, xy, yz, xz]
    center_of_mass: [x, y, z]


    Returns
    ----------
    moment of inertia about center of mass : [xx, yy, zz, xy, yz, xz]
    """
    x = center_of_mass[0]
    y = center_of_mass[1]
    z = center_of_mass[2]
    translation_matrix = [y**2+z**2, x**2+z**2, x**2+y**2,
                          -x*y, -y*z, -x*z]
    return [i - mass*t for i, t in zip(inertia, translation_matrix)]


def prettify(elem):
    """
    Return a pretty-printed XML string for the Element.
    Parameters
    ----------
    elem : xml.etree.ElementTree.Element


    Returns
    ----------
    pretified xml : str
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")



def get_parent(occ):
# function to find the root component of the joint. This is necessary for the correct component name in the urdf file
    if occ.assemblyContext != None:
        #print(occ.name)
        occ = get_parent(occ.assemblyContext)
    return occ


def matrix_from_axes(x, y, z):
    """
    Build a Fusion Matrix3D from orthonormal axes.
    """

    m = adsk.core.Matrix3D.create()

    a = [0.0] * 16

    a[0] = x.x
    a[1] = y.x
    a[2] = z.x
    a[3] = 0.0

    a[4] = x.y
    a[5] = y.y
    a[6] = z.y
    a[7] = 0.0

    a[8] = x.z
    a[9] = y.z
    a[10] = z.z
    a[11] = 0.0

    a[12] = 0.0
    a[13] = 0.0
    a[14] = 0.0
    a[15] = 1.0

    m.setWithArray(a)

    return m


def get_usd_joint_axis(axis, joint_frame):
    """
    Convert Fusion joint axis into USD RevoluteJoint local axis.

    Fusion axis is in component/world space.
    USD axis is in the joint local frame.
    """

    # Make a copy because transformBy modifies the vector
    local_axis = adsk.core.Vector3D.create(
        axis.x,
        axis.y,
        axis.z
    )

    # Convert world/component axis into joint frame
    inv = joint_frame.copy()
    inv.invert()

    local_axis.transformBy(inv)

    x = abs(local_axis.x)
    y = abs(local_axis.y)
    z = abs(local_axis.z)

    if x > y and x > z:
        return "X", local_axis.x

    elif y > x and y > z:
        return "Y", local_axis.y

    else:
        return "Z", local_axis.z


def matrix_to_quaternion(mat):
    """
    Convert Fusion Matrix3D rotation to quaternion.
    Returns [w,x,y,z] for USD Gf.Quatf.
    """

    m = mat.asArray()

    # Fusion Matrix3D is row-major:
    #
    # [ r00 r01 r02 tx ]
    # [ r10 r11 r12 ty ]
    # [ r20 r21 r22 tz ]
    # [  0   0   0  1 ]

    r00 = m[0]
    r01 = m[1]
    r02 = m[2]

    r10 = m[4]
    r11 = m[5]
    r12 = m[6]

    r20 = m[8]
    r21 = m[9]
    r22 = m[10]


    trace = r00 + r11 + r22

    if trace > 0:

        s = 0.5 / (trace + 1.0)**0.5

        w = 0.25 / s
        x = (r21-r12)*s
        y = (r02-r20)*s
        z = (r10-r01)*s

    elif r00 > r11 and r00 > r22:

        s = 2.0 * (1.0+r00-r11-r22)**0.5

        w = (r21-r12)/s
        x = 0.25*s
        y = (r01+r10)/s
        z = (r02+r20)/s

    elif r11 > r22:

        s = 2.0*(1.0+r11-r00-r22)**0.5

        w = (r02-r20)/s
        x = (r01+r10)/s
        y = 0.25*s
        z = (r12+r21)/s

    else:

        s = 2.0*(1.0+r22-r00-r11)**0.5

        w = (r10-r01)/s
        x = (r02+r20)/s
        y = (r12+r21)/s
        z = 0.25*s


    return [
        float(w),
        float(x),
        float(y),
        float(z)
    ]

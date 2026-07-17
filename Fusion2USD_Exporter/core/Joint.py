# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:17:17 2019
Modified on Sun Jan 17 2021

@author: syuntoku
@author: spacemaster85
"""

import adsk, re, traceback
from xml.etree.ElementTree import Element, SubElement
from itertools import chain
import math

def matrix_to_list(mat):
    return [x for x in mat.asArray()]


class Joint:
    def __init__(self, name, xyz, axis, parent, child, joint_type, upper_limit, lower_limit):
        """
        Attributes
        ----------
        name: str
            name of the joint
        type: str
            type of the joint(ex: rev)
        xyz: [x, y, z]
            coordinate of the joint
        axis: [x, y, z]
            coordinate of axis of the joint
        parent: str
            parent link
        child: str
            child link
        joint_xml: str
            generated xml describing about the joint
        tran_xml: str
            generated xml describing about the transmission
        """
        self.name = name
        self.type = joint_type
        self.xyz = xyz
        self.parent = parent
        self.child = child
        self.joint_xml = None
        self.tran_xml = None
        self.axis = axis  # for 'revolute' and 'continuous'
        self.upper_limit = upper_limit  # for 'revolute' and 'prismatic'
        self.lower_limit = lower_limit  # for 'revolute' and 'prismatic'


def make_joints_dict(root, msg):
    """
    joints_dict holds parent, axis and xyz informatino of the joints


    Parameters
    ----------
    root: adsk.fusion.Design.cast(product)
        Root component
    msg: str
        Tell the status

    Returns
    ----------
    joints_dict:
        {name: {type, axis, upper_limit, lower_limit, parent, child, xyz}}
    msg: str
        Tell the status
    """

    joint_type_list = [
    'fixed', 'revolute', 'prismatic', 'Cylinderical',
    'PinSlot', 'Planner', 'Ball']  # these are the names in urdf

    joints_dict = {}



    # import inspect

    # for joint in chain(root.joints, root.asBuiltJoints):
    #     print("=" * 80)
    #     print(joint.name)
    #     print(type(joint))

    #     # for attr in dir(joint):
    #     #     if not attr.startswith("_"):
    #     #         try:
    #     #             value = getattr(joint, attr)
    #     #             print(attr, type(value))
    #     #         except:
    #     #             pass

    #     # print()

    #     try:
    #         # print(type(joint.geometryOrOriginTwo))
    #         # print(dir(joint.geometryOrOriginTwo))
    #         # print(type(joint.geometryOrOriginTwo.geometry))
    #         # print(dir(joint.geometryOrOriginTwo.geometry))

    #         print("Joint transform:", joint.transform.asArray())
    #         print("Parent transform:", joint.occurrenceTwo.transform2.asArray())
    #         print("Child transform :", joint.occurrenceOne.transform2.asArray())
    #     except:
    #         pass




    for joint in chain(root.joints, root.asBuiltJoints):
        joint_dict = {}
        joint_type = joint_type_list[joint.jointMotion.jointType]
        joint_dict['type'] = joint_type

        # switch by the type of the joint
        joint_dict['axis'] = [0, 0, 0]
        joint_dict['upper_limit'] = 0.0
        joint_dict['lower_limit'] = 0.0

        # support  "Revolute", "Rigid" and "Slider"
        if joint_type == 'revolute':
            joint_dict['axis'] = [i for i in \
                joint.jointMotion.rotationAxisVector.asArray()] ## In Fusion, exported axis is normalized.
            max_enabled = joint.jointMotion.rotationLimits.isMaximumValueEnabled
            min_enabled = joint.jointMotion.rotationLimits.isMinimumValueEnabled
            if max_enabled and min_enabled:
                joint_dict['upper_limit'] = -math.degrees(joint.jointMotion.rotationLimits.minimumValue)
                joint_dict['lower_limit'] = -math.degrees(joint.jointMotion.rotationLimits.maximumValue)
            elif max_enabled and not min_enabled:
                msg = joint.name + 'is not set its lower limit. Please set it and try again.'
                break
            elif not max_enabled and min_enabled:
                msg = joint.name + 'is not set its upper limit. Please set it and try again.'
                break
            else:  # if there is no angle limit
                joint_dict['type'] = 'continuous'

        elif joint_type == 'prismatic':
            joint_dict['axis'] = [i for i in \
                joint.jointMotion.slideDirectionVector.asArray()]  # Also normalized
            max_enabled = joint.jointMotion.slideLimits.isMaximumValueEnabled
            min_enabled = joint.jointMotion.slideLimits.isMinimumValueEnabled
            if max_enabled and min_enabled:
                joint_dict['upper_limit'] = joint.jointMotion.slideLimits.maximumValue/100
                joint_dict['lower_limit'] = joint.jointMotion.slideLimits.minimumValue/100
            elif max_enabled and not min_enabled:
                msg = joint.name + 'is not set its lower limit. Please set it and try again.'
                break
            elif not max_enabled and min_enabled:
                msg = joint.name + 'is not set its upper limit. Please set it and try again.'
                break
        elif joint_type == 'fixed':
            pass


        def get_parent(occ):
        # function to find the root component of the joint. This is necessary for the correct component name in the urdf file
            if occ.assemblyContext != None:
                #print(occ.name)
                occ = get_parent(occ.assemblyContext)
            return occ

        if joint.occurrenceTwo != None and joint.occurrenceOne != None:
            parent_occ = get_parent(joint.occurrenceTwo)
            # print("Joint 1 Parent: " +parent_occ.name)
            if "base_link" in parent_occ.name:
                joint_dict['parent'] = 'base_link'
                base_link = parent_occ
            else:
                joint_dict['parent'] = re.sub(r'[^A-Za-z0-9_]', '_', parent_occ.name)
            # print("Joint 2: " +joint.occurrenceOne.name)
            parent_occ = get_parent(joint.occurrenceOne)
            # print("Joint 2 Parent: " +parent_occ.name)
            joint_dict['child'] = re.sub(r'[^A-Za-z0-9_]', '_', parent_occ.name)
        else:
            break

        def getJointOriginWorldCoordinates(joint):
            """
            Returns joint origin in world coordinates.
            Works for Joint and AsBuiltJoint.
            """

            def get_occurrence_world_matrix(occ):
                """
                Builds transform from occurrence to world.
                """
                mat = adsk.core.Matrix3D.create()

                chain = []

                while occ:
                    chain.append(occ)

                    if occ.assemblyContext:
                        occ = occ.assemblyContext
                    else:
                        break

                # apply from root down
                for item in reversed(chain):
                    if item.transform:
                        mat.transformBy(item.transform)

                return mat


            # -----------------------------
            # Find the correct joint origin
            # -----------------------------

            if isinstance(joint, adsk.fusion.AsBuiltJoint):

                origin = joint.geometry.origin
                occ = joint.occurrenceOne

            else:
                origin = joint.geometryOrOriginOne.origin
                occ = joint.occurrenceOne


            # copy because transform modifies object
            world_point = origin.copy()

            # transform into world coordinates
            mat = get_occurrence_world_matrix(occ)

            world_point.transformBy(mat)

            return world_point

        # try:
        #     if type(joint)==adsk.fusion.AsBuiltJoint:
        #         # print("asbuilt joint")
        #         if joint_type == 'fixed':
        #             # TODO: this might need to be fixed to fetch actual position
        #             joint_dict['xyz'] = [0.0, 0.0, 0.0]
        #         else:
        #             #xyz_of_joint = getJointOriginWorldCoordinates(joint)
        #             xyz_of_joint = joint.geometry.origin
        #             joint_dict['xyz'] = [i for i in xyz_of_joint.asArray()]
        #             # print(f"xyz : {joint_dict['xyz']}")

        #     else:
        #         # print("normal joint")
        #         #xyz_of_joint = getJointOriginWorldCoordinates(joint)
        #         xyz_of_joint = joint.geometryOrOriginTwo.origin
        #         joint_dict['xyz'] = [i for i in xyz_of_joint.asArray()]
        #         #print(f"xyz : {joint_dict['xyz']}")



        # except:
        #     print('Failed:\n{}'.format(traceback.format_exc()))
        #     try:
        #         if type(joint.geometryOrOriginTwo)==adsk.fusion.JointOrigin:
        #             data = joint.geometryOrOriginTwo.geometry.origin.asArray()
        #         else:
        #             data = joint.geometryOrOriginTwo.origin.asArray()
        #         joint_dict['xyz'] = [i for i in data]
        #     except:
        #         msg = joint.name + " doesn't have joint origin. Please set it and run again."
        #         break

        # try:
        #     xyz_of_joint = getJointOriginWorldCoordinates(joint)

        #     joint_dict['xyz'] = [
        #         float(xyz_of_joint.x),
        #         float(xyz_of_joint.y),
        #         float(xyz_of_joint.z)
        #     ]

        # except Exception:
        #     print("Joint origin failed:")
        #     print(traceback.format_exc())

        #     joint_dict['xyz'] = [0.0, 0.0, 0.0]

        # try:

        #     # -----------------------------------------
        #     # Get joint world transform
        #     # -----------------------------------------

        #     if isinstance(joint, adsk.fusion.AsBuiltJoint):

        #         joint_origin = joint.geometry

        #     else:

        #         joint_origin = joint.geometryOrOriginTwo


        #     print("JOINT TYPE:", type(joint))
        #     print("GEOMETRY TYPE:", type(joint.geometry))
        #     print("ORIGIN TWO TYPE:", type(joint.geometryOrOriginTwo))
        #     # JointOrigin has a transform property
        #     if hasattr(joint_origin, "transform"):

        #         joint_transform = joint_origin.transform

        #     else:

        #         joint_transform = adsk.core.Matrix3D.create()
        #         joint_transform.translation = joint_origin.origin


        #     joint_dict["world_transform"] = [
        #         x for x in joint_transform.asArray()
        #     ]

        #     joint_dict["xyz"] = [
        #         x for x in joint_transform.translation.asArray()
        #     ]


        # except Exception:

        #     print("Joint transform failed:")
        #     print(traceback.format_exc())

        #     joint_dict["world_transform"] = [
        #         1,0,0,0,
        #         0,1,0,0,
        #         0,0,1,0,
        #         0,0,0,1
        #     ]

        #     joint_dict["xyz"] = [0,0,0]


        # print("JOINT:", joint.name)
        # print("ONE:", joint.occurrenceOne.name)
        # print("TWO:", joint.occurrenceTwo.name)
        # print("TYPE:", type(joint))
        # print("AXIS:", joint.jointMotion.rotationAxisVector.asArray())
        # print("TRANSFORM ONE:", joint.occurrenceOne.transform.asArray())
        # print("TRANSFORM TWO:", joint.occurrenceTwo.transform.asArray())


        try:
            def get_world_point(point, occ):
                """
                Convert a Fusion local point into world coordinates.
                """

                p = point.copy()

                mat = adsk.core.Matrix3D.create()

                chain = []

                while occ:
                    chain.append(occ)

                    if occ.assemblyContext:
                        occ = occ.assemblyContext
                    else:
                        break


                for item in reversed(chain):
                    if item.transform:
                        mat.transformBy(item.transform)

                p.transformBy(mat)

                return p


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



            # Fusion Joint
            if isinstance(joint, adsk.fusion.Joint):

                origin_parent = joint.geometryOrOriginTwo.origin
                origin_child = joint.geometryOrOriginOne.origin

                parent_world = get_world_point(
                    origin_parent,
                    joint.occurrenceTwo
                )

                child_world = get_world_point(
                    origin_child,
                    joint.occurrenceOne
                )


            # Fusion AsBuiltJoint
            else:
                # origin_parent = joint.geometry.origin
                # origin_child = joint.geometry.origin
                if joint.geometry:
                    # Normal AsBuiltJoint
                    origin_parent = joint.geometry.origin
                    origin_child = joint.geometry.origin

                else:
                    # Fixed AsBuiltJoint has no geometry.
                    # Use the component origins instead.
                    origin_parent = adsk.core.Point3D.create(0, 0, 0)
                    origin_child = adsk.core.Point3D.create(0, 0, 0)

                parent_world = get_world_point(
                    origin_parent,
                    joint.occurrenceTwo
                )

                child_world = get_world_point(
                    origin_child,
                    joint.occurrenceOne
                )


                # If there is no explicit joint geometry (fixed joint),
                # use the midpoint between the occurrences as the joint position.
                if joint.geometry is None:
                    parent_world = adsk.core.Point3D.create(
                        (parent_world.x + child_world.x) * 0.5,
                        (parent_world.y + child_world.y) * 0.5,
                        (parent_world.z + child_world.z) * 0.5
                    )


            joint_dict["world_xyz"] = [
                parent_world.x,
                parent_world.y,
                parent_world.z
            ]


            # store local joint positions too
            joint_dict["parent_xyz"] = [
                origin_parent.x,
                origin_parent.y,
                origin_parent.z
            ]

            joint_dict["child_xyz"] = [
                origin_child.x,
                origin_child.y,
                origin_child.z
            ]

            # ---------------------------------------
            # Joint orientation
            # ---------------------------------------

            parent_transform = joint.occurrenceTwo.transform2

            child_transform = joint.occurrenceOne.transform2

            joint_dict["parent_orientation"] = matrix_to_quaternion(
                parent_transform
            )

            joint_dict["child_orientation"] = matrix_to_quaternion(
                child_transform
            )


            # keep backwards compatibility
            joint_dict["xyz"] = joint_dict["world_xyz"]


        except Exception:
            print("Joint transform failed:")
            print(traceback.format_exc())

            joint_dict["xyz"] = [0,0,0]





        # def matrix_to_dict(matrix):
        #     """
        #     Convert Fusion Matrix3D into JSON serializable data.
        #     """

        #     return {
        #         "translation": [
        #             matrix.translation.x,
        #             matrix.translation.y,
        #             matrix.translation.z
        #         ],
        #         "rotation": [
        #             matrix.getCell(0,0), matrix.getCell(0,1), matrix.getCell(0,2),
        #             matrix.getCell(1,0), matrix.getCell(1,1), matrix.getCell(1,2),
        #             matrix.getCell(2,0), matrix.getCell(2,1), matrix.getCell(2,2),
        #         ]
        #     }


        # def get_relative_transform(parent_occ, child_occ):
        #     """
        #     Get child transform relative to parent occurrence.
        #     """

        #     parent_matrix = parent_occ.transform2.copy()
        #     child_matrix = child_occ.transform2.copy()

        #     parent_matrix.invert()

        #     parent_matrix.transformBy(child_matrix)

        #     return parent_matrix



        # #
        # # Get joint transform
        # #

        # try:

        #     if isinstance(joint, adsk.fusion.AsBuiltJoint):

        #         print("AS BUILT JOINT")

        #         parent_occ = joint.occurrenceTwo
        #         child_occ = joint.occurrenceOne


        #         relative = get_relative_transform(
        #             parent_occ,
        #             child_occ
        #         )


        #         joint_dict["xyz"] = [
        #             relative.translation.x,
        #             relative.translation.y,
        #             relative.translation.z
        #         ]

        #         joint_dict["orientation"] = matrix_to_dict(relative)


        #         #
        #         # Rigid AsBuilt joints have no geometry
        #         #
        #         if joint_type == "fixed":

        #             joint_dict["axis"] = [0,0,0]



        #     else:

        #         print("NORMAL JOINT")


        #         if hasattr(joint, "geometryOrOriginTwo"):

        #             origin = joint.geometryOrOriginTwo.origin


        #         elif hasattr(joint, "geometry"):

        #             origin = joint.geometry.origin


        #         else:

        #             raise RuntimeError(
        #                 "Joint has no origin information"
        #             )


        #         joint_dict["xyz"] = [
        #             origin.x,
        #             origin.y,
        #             origin.z
        #         ]


        #         #
        #         # Build orientation from joint transform
        #         #
        #         transform = adsk.core.Matrix3D.create()


        #         if hasattr(joint.geometryOrOriginTwo, "transform"):
        #             transform = joint.geometryOrOriginTwo.transform


        #         joint_dict["orientation"] = matrix_to_dict(transform)



        # except Exception:

        #     print(
        #         "Joint transform failed:\n{}"
        #         .format(traceback.format_exc())
        #     )


        #     #
        #     # Safe fallback
        #     #
        #     joint_dict["xyz"] = [0,0,0]

        #     joint_dict["orientation"] = {
        #         "translation":[0,0,0],
        #         "rotation":[
        #             1,0,0,
        #             0,1,0,
        #             0,0,1
        #         ]
        #     }





        joints_dict[re.sub(r'[^A-Za-z0-9_]', '_', joint.name)] = joint_dict
    return joints_dict, msg

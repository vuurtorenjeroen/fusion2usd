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
from ..utils import utils


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


    for joint in chain(root.joints, root.asBuiltJoints):
        joint_dict = {}
        joint_type = joint_type_list[joint.jointMotion.jointType]
        joint_dict['type'] = joint_type

        joint_dict['upper_limit'] = 0.0
        joint_dict['lower_limit'] = 0.0
        joint_frame = False
        usd_axis = False
        axis_sign = False

        # ---------------------------------------
        # Joint geometry
        # ---------------------------------------

        # Fusion Joint
        if isinstance(joint, adsk.fusion.Joint):
            origin_parent = joint.geometryOrOriginTwo.origin
            origin_child = joint.geometryOrOriginOne.origin

            parent_joint_frame = utils.matrix_from_axes(
                joint.geometryOrOriginTwo.secondaryAxisVector,
                joint.geometryOrOriginTwo.thirdAxisVector,
                joint.geometryOrOriginTwo.primaryAxisVector,
            )
            # child_joint_frame = utils.matrix_from_axes(
            #     joint.geometryOrOriginOne.secondaryAxisVector,
            #     joint.geometryOrOriginOne.thirdAxisVector,
            #     joint.geometryOrOriginOne.primaryAxisVector,
            # )
            joint_frame = parent_joint_frame
            child_joint_frame = joint_frame

        # Fusion AsBuiltJoint
        else:
            if joint.geometry:
                # Normal AsBuiltJoint
                origin_parent = joint.geometry.origin
                origin_child = joint.geometry.origin

                joint_frame = utils.matrix_from_axes(
                    joint.geometry.secondaryAxisVector,   # X
                    joint.geometry.thirdAxisVector,       # Y
                    joint.geometry.primaryAxisVector      # Z
                )
                parent_joint_frame = joint_frame
                child_joint_frame = joint_frame

            else:
                # Fixed AsBuiltJoint has no geometry.
                # Use the component origins instead.
                origin_parent = adsk.core.Point3D.create(0, 0, 0)
                origin_child = adsk.core.Point3D.create(0, 0, 0)


        # ---------------------------------------
        # Joint type/min/max
        # ---------------------------------------

        # support  "Revolute", "Rigid" and "Slider"
        if joint_type == 'revolute':
            usd_axis, axis_sign = utils.get_usd_joint_axis(
                joint.jointMotion.rotationAxisVector,
                joint_frame
            )
            joint_dict["usd_axis"] = usd_axis

            max_enabled = joint.jointMotion.rotationLimits.isMaximumValueEnabled
            min_enabled = joint.jointMotion.rotationLimits.isMinimumValueEnabled
            if max_enabled and min_enabled:
                joint_dict['upper_limit'] = math.degrees(joint.jointMotion.rotationLimits.maximumValue)
                joint_dict['lower_limit'] = math.degrees(joint.jointMotion.rotationLimits.minimumValue)
            elif max_enabled and not min_enabled:
                msg = joint.name + 'is not set its lower limit. Please set it and try again.'
                break
            elif not max_enabled and min_enabled:
                msg = joint.name + 'is not set its upper limit. Please set it and try again.'
                break
            else:  # if there is no angle limit
                joint_dict['type'] = 'continuous'

        elif joint_type == 'prismatic':
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


        # ---------------------------------------
        # Joint parent/child
        # ---------------------------------------

        if joint.occurrenceTwo != None and joint.occurrenceOne != None:
            parent_occ = utils.get_parent(joint.occurrenceTwo)
            # print("Joint 1 Parent: " +parent_occ.name)
            if "base_link" in parent_occ.name:
                joint_dict['parent'] = 'base_link'
            else:
                joint_dict['parent'] = re.sub(r'[^A-Za-z0-9_]', '_', parent_occ.name)
            # print("Joint 2: " +joint.occurrenceOne.name)
            parent_occ = utils.get_parent(joint.occurrenceOne)
            # print("Joint 2 Parent: " +parent_occ.name)
            joint_dict['child'] = re.sub(r'[^A-Za-z0-9_]', '_', parent_occ.name)
        else:
            break

        # Fix axis rotation direction
        if axis_sign and axis_sign < 0:
            par = joint_dict['parent']
            chi = joint_dict['child']
            joint_dict['parent'] = chi
            joint_dict['child'] = par


        # ---------------------------------------
        # Joint position
        # ---------------------------------------

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
        if joint_frame:
            joint_dict["parent_orientation"] = utils.matrix_to_quaternion(
                parent_joint_frame
            )

            joint_dict["child_orientation"] = utils.matrix_to_quaternion(
                child_joint_frame
            )


        joints_dict[re.sub(r'[^A-Za-z0-9_]', '_', joint.name)] = joint_dict
    return joints_dict, msg

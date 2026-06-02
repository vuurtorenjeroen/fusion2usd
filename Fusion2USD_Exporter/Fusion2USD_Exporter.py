#Author-syuntoku14
#Author-spacemaster85
#Modified on Sun Jan 17 2021
#Description-Generate URDF file from Fusion 360

import adsk, adsk.core, adsk.fusion, traceback
import os
from .utils import utils
from .core import Link, Joint
import json

"""
# length unit is 'cm' and inertial unit is 'kg/cm^2'
# If there is no 'body' in the root component, maybe the corrdinates are wrong.
"""

# joint effort: 100
# joint velocity: 100
# supports "Revolute", "Rigid" and "Slider" joint types



# I'm not sure how prismatic joint acts if there is no limit in fusion model

def run(context):
    ui = None
    success_msg = 'Successfully created JSON file'
    msg = success_msg

    try:
        # --------------------
        # initialize
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        title = 'Fusion2URDF'
        if not design:
            ui.messageBox('No active Fusion design', title)
            return

        root = design.rootComponent  # root component
        components = design.allComponents

        # set the names
        robot_name = root.name.split()[0].lower()
        package_name = robot_name + '_export'
        save_dir = utils.file_dialog(ui)
        if save_dir == False:
            ui.messageBox('Fusion2USD was canceled', title)
            return 0


        save_dir= save_dir + '/' + package_name
        try: os.mkdir(save_dir)
        except: pass


        # --------------------
        # set dictionaries

        # Generate joints_dict. All joints are related to root.
        joints_dict, msg = Joint.make_joints_dict(root, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0
        print(joints_dict)

        # Generate inertial_dict
        inertial_dict, msg = Link.make_inertial_dict(root, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0
        elif not 'base_link' in inertial_dict:
            msg = 'There is no base_link. Please set base_link and run again.'
            ui.messageBox(msg, title)
            return 0
        print(inertial_dict)

        # Generate material_dict, color_dict
        material_dict, color_dict, msg = Link.make_material_dict(root, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0
        print(material_dict)
        print(color_dict)

        # Generate json file
        file_name = save_dir + '/' + robot_name.lower() + '.json'  # the name of json file
        data = {
            "joints": joints_dict,
            "inertial": inertial_dict,
            "material": material_dict,
            "color": color_dict
        }
        with open(file_name, mode='w') as f:
            json.dump(data, f, ensure_ascii=False, indent=4)


        # Generate STl files
        utils.export_stl(app, save_dir)

        print(msg)
        ui.messageBox(msg, title)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

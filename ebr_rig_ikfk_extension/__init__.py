bl_info = {
    "name": "EBR Rig IKFK Extension",
    "author": "Dgards, DvqJackson_2, NJ Central",
    "version": (1, 0, 0),
    "blender": (3, 0, 0),
    "location": "View3D > Sidebar > Item",
    "description": "Adds IK/FK snapping and rig controls for EBR rigs",
    "category": "Rigging",
    "support": "COMMUNITY",
    "doc_url": "",
    "tracker_url": "",
}

import bpy
import math
import json
import collections
import traceback
from math import pi
from bpy.props import BoolProperty, StringProperty
from mathutils import Euler, Matrix, Quaternion, Vector
from rna_prop_ui import rna_idprop_quote_path

# Unique identifier for the rig
EBR_rig_id = "eb41kfksn4p"

# Define the constraint details
# Arm constraints
fk_to_ik_constraint_pairs_right_arm = [
    ("FK Arm Top.R", "FK To IK Snap Manual Arm Top.R"),
    ("FK Arm Bottom.R", "FK To IK Snap Manual Arm Bottom.R"),
    ("FK Wrist.R", "FK To IK Snap Manual Wrist.R")
]
fk_to_ik_constraint_pairs_left_arm = [
    ("FK Arm Top.L", "FK To IK Snap Manual Arm Top.L"),
    ("FK Arm Bottom.L", "FK To IK Snap Manual Arm Bottom.L"),
    ("FK Wrist.L", "FK To IK Snap Manual Wrist.L")
]
ik_to_fk_constraint_pairs_right_arm = [
    ("Arm IK.R", "Arm IK To FK Snap Manual.R"),
    ("IK pole.R", "IK To FK Snap Manual pole.R")
]
ik_to_fk_constraint_pairs_left_arm = [
    ("Arm IK.L", "Arm IK To FK Snap Manual.L"),
    ("IK pole.L", "IK To FK Snap Manual pole.L")
]

# Leg constraints
fk_to_ik_constraint_pairs_right_leg = [
    ("FK Leg Top.R", "FK To IK Snap Manual Leg Top.R"),
    ("FK Leg Bottom.R", "FK To IK Snap Manual Leg Bottom.R"),
    ("FK Foot.R", "FK To IK Snap Manual Foot.R")
]
ik_to_fk_constraint_pairs_right_leg = [
    ("Foot IK.R", "Foot IK To FK Snap Manual.R")
]
fk_to_ik_constraint_pairs_left_leg = [
    ("FK Leg Top.L", "FK To IK Snap Manual Leg Top.L"),
    ("FK Leg Bottom.L", "FK To IK Snap Manual Leg Bottom.L"),
    ("FK Foot.L", "FK To IK Snap Manual Foot.L")
]
ik_to_fk_constraint_pairs_left_leg = [
    ("Foot IK.L", "Foot IK To FK Snap Manual.L")
]

# MLeg constraints
fk_to_ik_constraint_pairs_right_mleg = [
    ("FK Leg Top T.R", "FK To IK Snap Manual Leg Top T.R"),
    ("FK Leg Bottom T.R", "FK To IK Snap Manual Leg Bottom T.R"),
    ("FK Foot T.R", "FK To IK Snap Manual Foot T.R")
]
ik_to_fk_constraint_pairs_right_mleg = [
    ("Foot IK T.R", "Foot IK To FK Snap Manual T.R")
]
fk_to_ik_constraint_pairs_left_mleg = [
    ("FK Leg Top T.L", "FK To IK Snap Manual Leg Top T.L"),
    ("FK Leg Bottom T.L", "FK To IK Snap Manual Leg Bottom T.L"),
    ("FK Foot T.L", "FK To IK Snap Manual Foot T.L")
]
ik_to_fk_constraint_pairs_left_mleg = [
    ("Foot IK T.L", "Foot IK To FK Snap Manual T.L")
]

# SLeg constraints
fk_to_ik_constraint_pairs_right_sleg = [
    ("FK Leg S Top.R", "FK To IK Snap Manual Leg S Top.R"),
    ("FK Leg S Bottom.R", "FK To IK Snap Manual Leg S Bottom.R"),
    ("FK Foot S.R", "FK To IK Snap Manual Foot S.R")
]
ik_to_fk_constraint_pairs_right_sleg = [
    ("Foot S IK.R", "Foot S IK To FK Snap Manual.R")
]
fk_to_ik_constraint_pairs_left_sleg = [
    ("FK Leg S Top.L", "FK To IK Snap Manual Leg S Top.L"),
    ("FK Leg S Bottom.L", "FK To IK Snap Manual Leg S Bottom.L"),
    ("FK Foot S.L", "FK To IK Snap Manual Foot S.L")
]
ik_to_fk_constraint_pairs_left_sleg = [
    ("Foot S IK.L", "Foot S IK To FK Snap Manual.L")
]

# Master Bone constraint
master_bone_constraint_pairs = [
    ("Master Bone", "Master Bone Snapping")
]

# Operator for FK to IK Right Arm Snap
class OBJECT_OT_fk_to_ik_snap_right_arm(bpy.types.Operator):
    bl_idname = "object.fk_to_ik_snap_right_arm"
    bl_label = "ArmR FKIK Snap"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        armature = bpy.context.active_object
        if not armature or armature.type != 'ARMATURE':
            print("Please select an armature object as the active object.")
            return {'CANCELLED'}
        
        current_mode = armature.mode
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
        
        constrained_bones = []
        for source_bone_name, target_bone_name in fk_to_ik_constraint_pairs_right_arm:
            source_bone = armature.pose.bones.get(source_bone_name)
            target_bone = armature.pose.bones.get(target_bone_name)
            if source_bone and target_bone:
                existing_constraint = None
                for constraint in source_bone.constraints:
                    if constraint.type == 'COPY_TRANSFORMS' and constraint.target == armature and constraint.subtarget == target_bone_name:
                        existing_constraint = constraint
                        break
                if not existing_constraint:
                    constraint = source_bone.constraints.new(type='COPY_TRANSFORMS')
                    constraint.target = armature
                    constraint.subtarget = target_bone_name
                    constraint.target_space = 'WORLD'
                    constraint.owner_space = 'WORLD'
                    constraint.mix_mode = 'REPLACE'
                    constrained_bones.append(source_bone_name)
                    print(f"Created copy transform constraint for {source_bone_name} to {target_bone_name}")
                else:
                    constrained_bones.append(source_bone_name)
                    print(f"Constraint already exists for {source_bone_name} to {target_bone_name}")
            else:
                print(f"One or both bones not found: {source_bone_name} or {target_bone_name}. Available bones: {[pbone.name for pbone in armature.pose.bones]}")
        
        if constrained_bones:
            current_frame = bpy.context.scene.frame_current
            for pbone in armature.pose.bones:
                pbone.bone.select = pbone.name in constrained_bones
            bpy.ops.nla.bake(
                frame_start=current_frame,
                frame_end=current_frame,
                step=1,
                only_selected=True,
                visual_keying=True,
                clear_constraints=True,
                clear_parents=False,
                bake_types={'POSE'},
                use_current_action=True
            )
            for pbone in armature.pose.bones:
                pbone.bone.select = False
        
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode=current_mode)
        return {'FINISHED'}

# Operator for FK to IK Left Arm Snap
class OBJECT_OT_fk_to_ik_snap_left_arm(bpy.types.Operator):
    bl_idname = "object.fk_to_ik_snap_left_arm"
    bl_label = "ArmL FKIK Snap"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        armature = bpy.context.active_object
        if not armature or armature.type != 'ARMATURE':
            print("Please select an armature object as the active object.")
            return {'CANCELLED'}
        
        current_mode = armature.mode
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
        
        constrained_bones = []
        for source_bone_name, target_bone_name in fk_to_ik_constraint_pairs_left_arm:
            source_bone = armature.pose.bones.get(source_bone_name)
            target_bone = armature.pose.bones.get(target_bone_name)
            if source_bone and target_bone:
                existing_constraint = None
                for constraint in source_bone.constraints:
                    if constraint.type == 'COPY_TRANSFORMS' and constraint.target == armature and constraint.subtarget == target_bone_name:
                        existing_constraint = constraint
                        break
                if not existing_constraint:
                    constraint = source_bone.constraints.new(type='COPY_TRANSFORMS')
                    constraint.target = armature
                    constraint.subtarget = target_bone_name
                    constraint.target_space = 'WORLD'
                    constraint.owner_space = 'WORLD'
                    constraint.mix_mode = 'REPLACE'
                    constrained_bones.append(source_bone_name)
                    print(f"Created copy transform constraint for {source_bone_name} to {target_bone_name}")
                else:
                    constrained_bones.append(source_bone_name)
                    print(f"Constraint already exists for {source_bone_name} to {target_bone_name}")
            else:
                print(f"One or both bones not found: {source_bone_name} or {target_bone_name}. Available bones: {[pbone.name for pbone in armature.pose.bones]}")
        
        if constrained_bones:
            current_frame = bpy.context.scene.frame_current
            for pbone in armature.pose.bones:
                pbone.bone.select = pbone.name in constrained_bones
            bpy.ops.nla.bake(
                frame_start=current_frame,
                frame_end=current_frame,
                step=1,
                only_selected=True,
                visual_keying=True,
                clear_constraints=True,
                clear_parents=False,
                bake_types={'POSE'},
                use_current_action=True
            )
            for pbone in armature.pose.bones:
                pbone.bone.select = False
        
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode=current_mode)
        return {'FINISHED'}

# Operator for IK to FK Right Arm Snap
class OBJECT_OT_ik_to_fk_snap_right_arm(bpy.types.Operator):
    bl_idname = "object.ik_to_fk_snap_right_arm"
    bl_label = "ArmR IKFK Snap"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        armature = bpy.context.active_object
        if not armature or armature.type != 'ARMATURE':
            print("Please select an armature object as the active object.")
            return {'CANCELLED'}
        
        current_mode = armature.mode
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
        
        constrained_bones = []
        for source_bone_name, target_bone_name in ik_to_fk_constraint_pairs_right_arm:
            source_bone = armature.pose.bones.get(source_bone_name)
            target_bone = armature.pose.bones.get(target_bone_name)
            if source_bone and target_bone:
                existing_constraint = None
                for constraint in source_bone.constraints:
                    if constraint.type == 'COPY_TRANSFORMS' and constraint.target == armature and constraint.subtarget == target_bone_name:
                        existing_constraint = constraint
                        break
                if not existing_constraint:
                    constraint = source_bone.constraints.new(type='COPY_TRANSFORMS')
                    constraint.target = armature
                    constraint.subtarget = target_bone_name
                    constraint.target_space = 'WORLD'
                    constraint.owner_space = 'WORLD'
                    constraint.mix_mode = 'REPLACE'
                    constrained_bones.append(source_bone_name)
                    print(f"Created copy transform constraint for {source_bone_name} to {target_bone_name}")
                else:
                    constrained_bones.append(source_bone_name)
                    print(f"Constraint already exists for {source_bone_name} to {target_bone_name}")
            else:
                print(f"One or both bones not found: {source_bone_name} or {target_bone_name}. Available bones: {[pbone.name for pbone in armature.pose.bones]}")
        
        if constrained_bones:
            current_frame = bpy.context.scene.frame_current
            for pbone in armature.pose.bones:
                pbone.bone.select = pbone.name in constrained_bones
            bpy.ops.nla.bake(
                frame_start=current_frame,
                frame_end=current_frame,
                step=1,
                only_selected=True,
                visual_keying=True,
                clear_constraints=True,
                clear_parents=False,
                bake_types={'POSE'},
                use_current_action=True
            )
            for pbone in armature.pose.bones:
                pbone.bone.select = False
        
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode=current_mode)
        return {'FINISHED'}

# Operator for IK to FK Left Arm Snap
class OBJECT_OT_ik_to_fk_snap_left_arm(bpy.types.Operator):
    bl_idname = "object.ik_to_fk_snap_left_arm"
    bl_label = "ArmL IKFK Snap"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        armature = bpy.context.active_object
        if not armature or armature.type != 'ARMATURE':
            print("Please select an armature object as the active object.")
            return {'CANCELLED'}
        
        current_mode = armature.mode
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
        
        constrained_bones = []
        for source_bone_name, target_bone_name in ik_to_fk_constraint_pairs_left_arm:
            source_bone = armature.pose.bones.get(source_bone_name)
            target_bone = armature.pose.bones.get(target_bone_name)
            if source_bone and target_bone:
                existing_constraint = None
                for constraint in source_bone.constraints:
                    if constraint.type == 'COPY_TRANSFORMS' and constraint.target == armature and constraint.subtarget == target_bone_name:
                        existing_constraint = constraint
                        break
                if not existing_constraint:
                    constraint = source_bone.constraints.new(type='COPY_TRANSFORMS')
                    constraint.target = armature
                    constraint.subtarget = target_bone_name
                    constraint.target_space = 'WORLD'
                    constraint.owner_space = 'WORLD'
                    constraint.mix_mode = 'REPLACE'
                    constrained_bones.append(source_bone_name)
                    print(f"Created copy transform constraint for {source_bone_name} to {target_bone_name}")
                else:
                    constrained_bones.append(source_bone_name)
                    print(f"Constraint already exists for {source_bone_name} to {target_bone_name}")
            else:
                print(f"One or both bones not found: {source_bone_name} or {target_bone_name}. Available bones: {[pbone.name for pbone in armature.pose.bones]}")
        
        if constrained_bones:
            current_frame = bpy.context.scene.frame_current
            for pbone in armature.pose.bones:
                pbone.bone.select = pbone.name in constrained_bones
            bpy.ops.nla.bake(
                frame_start=current_frame,
                frame_end=current_frame,
                step=1,
                only_selected=True,
                visual_keying=True,
                clear_constraints=True,
                clear_parents=False,
                bake_types={'POSE'},
                use_current_action=True
            )
            for pbone in armature.pose.bones:
                pbone.bone.select = False
        
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode=current_mode)
        return {'FINISHED'}

# Operator for FK to IK Right Leg Snap
class OBJECT_OT_fk_to_ik_snap_right_leg(bpy.types.Operator):
    bl_idname = "object.fk_to_ik_snap_right_leg"
    bl_label = "LegR FKIK Snap"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        armature = bpy.context.active_object
        if not armature or armature.type != 'ARMATURE':
            print("Please select an armature object as the active object.")
            return {'CANCELLED'}
        
        current_mode = armature.mode
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
        
        constrained_bones = []
        for source_bone_name, target_bone_name in fk_to_ik_constraint_pairs_right_leg:
            source_bone = armature.pose.bones.get(source_bone_name)
            target_bone = armature.pose.bones.get(target_bone_name)
            if source_bone and target_bone:
                existing_constraint = None
                for constraint in source_bone.constraints:
                    if constraint.type == 'COPY_TRANSFORMS' and constraint.target == armature and constraint.subtarget == target_bone_name:
                        existing_constraint = constraint
                        break
                if not existing_constraint:
                    constraint = source_bone.constraints.new(type='COPY_TRANSFORMS')
                    constraint.target = armature
                    constraint.subtarget = target_bone_name
                    constraint.target_space = 'WORLD'
                    constraint.owner_space = 'WORLD'
                    constraint.mix_mode = 'REPLACE'
                    constrained_bones.append(source_bone_name)
                    print(f"Created copy transform constraint for {source_bone_name} to {target_bone_name}")
                else:
                    constrained_bones.append(source_bone_name)
                    print(f"Constraint already exists for {source_bone_name} to {target_bone_name}")
            else:
                print(f"One or both bones not found: {source_bone_name} or {target_bone_name}. Available bones: {[pbone.name for pbone in armature.pose.bones]}")
        
        if constrained_bones:
            current_frame = bpy.context.scene.frame_current
            for pbone in armature.pose.bones:
                pbone.bone.select = pbone.name in constrained_bones
            bpy.ops.nla.bake(
                frame_start=current_frame,
                frame_end=current_frame,
                step=1,
                only_selected=True,
                visual_keying=True,
                clear_constraints=True,
                clear_parents=False,
                bake_types={'POSE'},
                use_current_action=True
            )
            for pbone in armature.pose.bones:
                pbone.bone.select = False
        
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode=current_mode)
        return {'FINISHED'}

# Operator for IK to FK Right Leg Snap
class OBJECT_OT_ik_to_fk_snap_right_leg(bpy.types.Operator):
    bl_idname = "object.ik_to_fk_snap_right_leg"
    bl_label = "LegR IKFK Snap"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        armature = bpy.context.active_object
        if not armature or armature.type != 'ARMATURE':
            print("Please select an armature object as the active object.")
            return {'CANCELLED'}
        
        current_mode = armature.mode
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
        
        constrained_bones = []
        for source_bone_name, target_bone_name in ik_to_fk_constraint_pairs_right_leg:
            source_bone = armature.pose.bones.get(source_bone_name)
            target_bone = armature.pose.bones.get(target_bone_name)
            if source_bone and target_bone:
                existing_constraint = None
                for constraint in source_bone.constraints:
                    if constraint.type == 'COPY_TRANSFORMS' and constraint.target == armature and constraint.subtarget == target_bone_name:
                        existing_constraint = constraint
                        break
                if not existing_constraint:
                    constraint = source_bone.constraints.new(type='COPY_TRANSFORMS')
                    constraint.target = armature
                    constraint.subtarget = target_bone_name
                    constraint.target_space = 'WORLD'
                    constraint.owner_space = 'WORLD'
                    constraint.mix_mode = 'REPLACE'
                    constrained_bones.append(source_bone_name)
                    print(f"Created copy transform constraint for {source_bone_name} to {target_bone_name}")
                else:
                    constrained_bones.append(source_bone_name)
                    print(f"Constraint already exists for {source_bone_name} to {target_bone_name}")
            else:
                print(f"One or both bones not found: {source_bone_name} or {target_bone_name}. Available bones: {[pbone.name for pbone in armature.pose.bones]}")
        
        if constrained_bones:
            current_frame = bpy.context.scene.frame_current
            for pbone in armature.pose.bones:
                pbone.bone.select = pbone.name in constrained_bones
            bpy.ops.nla.bake(
                frame_start=current_frame,
                frame_end=current_frame,
                step=1,
                only_selected=True,
                visual_keying=True,
                clear_constraints=True,
                clear_parents=False,
                bake_types={'POSE'},
                use_current_action=True
            )
            for pbone in armature.pose.bones:
                pbone.bone.select = False
        
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode=current_mode)
        return {'FINISHED'}

# Operator for FK to IK Left Leg Snap
class OBJECT_OT_fk_to_ik_snap_left_leg(bpy.types.Operator):
    bl_idname = "object.fk_to_ik_snap_left_leg"
    bl_label = "LegL FKIK Snap"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        armature = bpy.context.active_object
        if not armature or armature.type != 'ARMATURE':
            print("Please select an armature object as the active object.")
            return {'CANCELLED'}
        
        current_mode = armature.mode
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
        
        constrained_bones = []
        for source_bone_name, target_bone_name in fk_to_ik_constraint_pairs_left_leg:
            source_bone = armature.pose.bones.get(source_bone_name)
            target_bone = armature.pose.bones.get(target_bone_name)
            if source_bone and target_bone:
                existing_constraint = None
                for constraint in source_bone.constraints:
                    if constraint.type == 'COPY_TRANSFORMS' and constraint.target == armature and constraint.subtarget == target_bone_name:
                        existing_constraint = constraint
                        break
                if not existing_constraint:
                    constraint = source_bone.constraints.new(type='COPY_TRANSFORMS')
                    constraint.target = armature
                    constraint.subtarget = target_bone_name
                    constraint.target_space = 'WORLD'
                    constraint.owner_space = 'WORLD'
                    constraint.mix_mode = 'REPLACE'
                    constrained_bones.append(source_bone_name)
                    print(f"Created copy transform constraint for {source_bone_name} to {target_bone_name}")
                else:
                    constrained_bones.append(source_bone_name)
                    print(f"Constraint already exists for {source_bone_name} to {target_bone_name}")
            else:
                print(f"One or both bones not found: {source_bone_name} or {target_bone_name}. Available bones: {[pbone.name for pbone in armature.pose.bones]}")
        
        if constrained_bones:
            current_frame = bpy.context.scene.frame_current
            for pbone in armature.pose.bones:
                pbone.bone.select = pbone.name in constrained_bones
            bpy.ops.nla.bake(
                frame_start=current_frame,
                frame_end=current_frame,
                step=1,
                only_selected=True,
                visual_keying=True,
                clear_constraints=True,
                clear_parents=False,
                bake_types={'POSE'},
                use_current_action=True
            )
            for pbone in armature.pose.bones:
                pbone.bone.select = False
        
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode=current_mode)
        return {'FINISHED'}

# Operator for IK to FK Left Leg Snap
class OBJECT_OT_ik_to_fk_snap_left_leg(bpy.types.Operator):
    bl_idname = "object.ik_to_fk_snap_left_leg"
    bl_label = "LegL IKFK Snap"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        armature = bpy.context.active_object
        if not armature or armature.type != 'ARMATURE':
            print("Please select an armature object as the active object.")
            return {'CANCELLED'}
        
        current_mode = armature.mode
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
        
        constrained_bones = []
        for source_bone_name, target_bone_name in ik_to_fk_constraint_pairs_left_leg:
            source_bone = armature.pose.bones.get(source_bone_name)
            target_bone = armature.pose.bones.get(target_bone_name)
            if source_bone and target_bone:
                existing_constraint = None
                for constraint in source_bone.constraints:
                    if constraint.type == 'COPY_TRANSFORMS' and constraint.target == armature and constraint.subtarget == target_bone_name:
                        existing_constraint = constraint
                        break
                if not existing_constraint:
                    constraint = source_bone.constraints.new(type='COPY_TRANSFORMS')
                    constraint.target = armature
                    constraint.subtarget = target_bone_name
                    constraint.target_space = 'WORLD'
                    constraint.owner_space = 'WORLD'
                    constraint.mix_mode = 'REPLACE'
                    constrained_bones.append(source_bone_name)
                    print(f"Created copy transform constraint for {source_bone_name} to {target_bone_name}")
                else:
                    constrained_bones.append(source_bone_name)
                    print(f"Constraint already exists for {source_bone_name} to {target_bone_name}")
            else:
                print(f"One or both bones not found: {source_bone_name} or {target_bone_name}. Available bones: {[pbone.name for pbone in armature.pose.bones]}")
        
        if constrained_bones:
            current_frame = bpy.context.scene.frame_current
            for pbone in armature.pose.bones:
                pbone.bone.select = pbone.name in constrained_bones
            bpy.ops.nla.bake(
                frame_start=current_frame,
                frame_end=current_frame,
                step=1,
                only_selected=True,
                visual_keying=True,
                clear_constraints=True,
                clear_parents=False,
                bake_types={'POSE'},
                use_current_action=True
            )
            for pbone in armature.pose.bones:
                pbone.bone.select = False
        
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode=current_mode)
        return {'FINISHED'}

# Operator for FK to IK Right MLeg Snap
class OBJECT_OT_fk_to_ik_snap_right_mleg(bpy.types.Operator):
    bl_idname = "object.fk_to_ik_snap_right_mleg"
    bl_label = "MLegR FKIK Snap"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        armature = bpy.context.active_object
        if not armature or armature.type != 'ARMATURE':
            print("Please select an armature object as the active object.")
            return {'CANCELLED'}
        
        current_mode = armature.mode
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
        
        constrained_bones = []
        for source_bone_name, target_bone_name in fk_to_ik_constraint_pairs_right_mleg:
            source_bone = armature.pose.bones.get(source_bone_name)
            target_bone = armature.pose.bones.get(target_bone_name)
            if source_bone and target_bone:
                existing_constraint = None
                for constraint in source_bone.constraints:
                    if constraint.type == 'COPY_TRANSFORMS' and constraint.target == armature and constraint.subtarget == target_bone_name:
                        existing_constraint = constraint
                        break
                if not existing_constraint:
                    constraint = source_bone.constraints.new(type='COPY_TRANSFORMS')
                    constraint.target = armature
                    constraint.subtarget = target_bone_name
                    constraint.target_space = 'WORLD'
                    constraint.owner_space = 'WORLD'
                    constraint.mix_mode = 'REPLACE'
                    constrained_bones.append(source_bone_name)
                    print(f"Created copy transform constraint for {source_bone_name} to {target_bone_name}")
                else:
                    constrained_bones.append(source_bone_name)
                    print(f"Constraint already exists for {source_bone_name} to {target_bone_name}")
            else:
                print(f"One or both bones not found: {source_bone_name} or {target_bone_name}. Available bones: {[pbone.name for pbone in armature.pose.bones]}")
        
        if constrained_bones:
            current_frame = bpy.context.scene.frame_current
            for pbone in armature.pose.bones:
                pbone.bone.select = pbone.name in constrained_bones
            bpy.ops.nla.bake(
                frame_start=current_frame,
                frame_end=current_frame,
                step=1,
                only_selected=True,
                visual_keying=True,
                clear_constraints=True,
                clear_parents=False,
                bake_types={'POSE'},
                use_current_action=True
            )
            for pbone in armature.pose.bones:
                pbone.bone.select = False
        
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode=current_mode)
        return {'FINISHED'}

# Operator for IK to FK Right MLeg Snap
class OBJECT_OT_ik_to_fk_snap_right_mleg(bpy.types.Operator):
    bl_idname = "object.ik_to_fk_snap_right_mleg"
    bl_label = "MLegR IKFK Snap"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        armature = bpy.context.active_object
        if not armature or armature.type != 'ARMATURE':
            print("Please select an armature object as the active object.")
            return {'CANCELLED'}
        
        current_mode = armature.mode
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
        
        constrained_bones = []
        for source_bone_name, target_bone_name in ik_to_fk_constraint_pairs_right_mleg:
            source_bone = armature.pose.bones.get(source_bone_name)
            target_bone = armature.pose.bones.get(target_bone_name)
            if source_bone and target_bone:
                existing_constraint = None
                for constraint in source_bone.constraints:
                    if constraint.type == 'COPY_TRANSFORMS' and constraint.target == armature and constraint.subtarget == target_bone_name:
                        existing_constraint = constraint
                        break
                if not existing_constraint:
                    constraint = source_bone.constraints.new(type='COPY_TRANSFORMS')
                    constraint.target = armature
                    constraint.subtarget = target_bone_name
                    constraint.target_space = 'WORLD'
                    constraint.owner_space = 'WORLD'
                    constraint.mix_mode = 'REPLACE'
                    constrained_bones.append(source_bone_name)
                    print(f"Created copy transform constraint for {source_bone_name} to {target_bone_name}")
                else:
                    constrained_bones.append(source_bone_name)
                    print(f"Constraint already exists for {source_bone_name} to {target_bone_name}")
            else:
                print(f"One or both bones not found: {source_bone_name} or {target_bone_name}. Available bones: {[pbone.name for pbone in armature.pose.bones]}")
        
        if constrained_bones:
            current_frame = bpy.context.scene.frame_current
            for pbone in armature.pose.bones:
                pbone.bone.select = pbone.name in constrained_bones
            bpy.ops.nla.bake(
                frame_start=current_frame,
                frame_end=current_frame,
                step=1,
                only_selected=True,
                visual_keying=True,
                clear_constraints=True,
                clear_parents=False,
                bake_types={'POSE'},
                use_current_action=True
            )
            for pbone in armature.pose.bones:
                pbone.bone.select = False
        
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode=current_mode)
        return {'FINISHED'}

# Operator for FK to IK Left MLeg Snap
class OBJECT_OT_fk_to_ik_snap_left_mleg(bpy.types.Operator):
    bl_idname = "object.fk_to_ik_snap_left_mleg"
    bl_label = "MLegL FKIK Snap"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        armature = bpy.context.active_object
        if not armature or armature.type != 'ARMATURE':
            print("Please select an armature object as the active object.")
            return {'CANCELLED'}
        
        current_mode = armature.mode
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
        
        constrained_bones = []
        for source_bone_name, target_bone_name in fk_to_ik_constraint_pairs_left_mleg:
            source_bone = armature.pose.bones.get(source_bone_name)
            target_bone = armature.pose.bones.get(target_bone_name)
            if source_bone and target_bone:
                existing_constraint = None
                for constraint in source_bone.constraints:
                    if constraint.type == 'COPY_TRANSFORMS' and constraint.target == armature and constraint.subtarget == target_bone_name:
                        existing_constraint = constraint
                        break
                if not existing_constraint:
                    constraint = source_bone.constraints.new(type='COPY_TRANSFORMS')
                    constraint.target = armature
                    constraint.subtarget = target_bone_name
                    constraint.target_space = 'WORLD'
                    constraint.owner_space = 'WORLD'
                    constraint.mix_mode = 'REPLACE'
                    constrained_bones.append(source_bone_name)
                    print(f"Created copy transform constraint for {source_bone_name} to {target_bone_name}")
                else:
                    constrained_bones.append(source_bone_name)
                    print(f"Constraint already exists for {source_bone_name} to {target_bone_name}")
            else:
                print(f"One or both bones not found: {source_bone_name} or {target_bone_name}. Available bones: {[pbone.name for pbone in armature.pose.bones]}")
        
        if constrained_bones:
            current_frame = bpy.context.scene.frame_current
            for pbone in armature.pose.bones:
                pbone.bone.select = pbone.name in constrained_bones
            bpy.ops.nla.bake(
                frame_start=current_frame,
                frame_end=current_frame,
                step=1,
                only_selected=True,
                visual_keying=True,
                clear_constraints=True,
                clear_parents=False,
                bake_types={'POSE'},
                use_current_action=True
            )
            for pbone in armature.pose.bones:
                pbone.bone.select = False
        
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode=current_mode)
        return {'FINISHED'}

# Operator for IK to FK Left MLeg Snap
class OBJECT_OT_ik_to_fk_snap_left_mleg(bpy.types.Operator):
    bl_idname = "object.ik_to_fk_snap_left_mleg"
    bl_label = "MLegL IKFK Snap"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        armature = bpy.context.active_object
        if not armature or armature.type != 'ARMATURE':
            print("Please select an armature object as the active object.")
            return {'CANCELLED'}
        
        current_mode = armature.mode
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
        
        constrained_bones = []
        for source_bone_name, target_bone_name in ik_to_fk_constraint_pairs_left_mleg:
            source_bone = armature.pose.bones.get(source_bone_name)
            target_bone = armature.pose.bones.get(target_bone_name)
            if source_bone and target_bone:
                existing_constraint = None
                for constraint in source_bone.constraints:
                    if constraint.type == 'COPY_TRANSFORMS' and constraint.target == armature and constraint.subtarget == target_bone_name:
                        existing_constraint = constraint
                        break
                if not existing_constraint:
                    constraint = source_bone.constraints.new(type='COPY_TRANSFORMS')
                    constraint.target = armature
                    constraint.subtarget = target_bone_name
                    constraint.target_space = 'WORLD'
                    constraint.owner_space = 'WORLD'
                    constraint.mix_mode = 'REPLACE'
                    constrained_bones.append(source_bone_name)
                    print(f"Created copy transform constraint for {source_bone_name} to {target_bone_name}")
                else:
                    constrained_bones.append(source_bone_name)
                    print(f"Constraint already exists for {source_bone_name} to {target_bone_name}")
            else:
                print(f"One or both bones not found: {source_bone_name} or {target_bone_name}. Available bones: {[pbone.name for pbone in armature.pose.bones]}")
        
        if constrained_bones:
            current_frame = bpy.context.scene.frame_current
            for pbone in armature.pose.bones:
                pbone.bone.select = pbone.name in constrained_bones
            bpy.ops.nla.bake(
                frame_start=current_frame,
                frame_end=current_frame,
                step=1,
                only_selected=True,
                visual_keying=True,
                clear_constraints=True,
                clear_parents=False,
                bake_types={'POSE'},
                use_current_action=True
            )
            for pbone in armature.pose.bones:
                pbone.bone.select = False
        
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode=current_mode)
        return {'FINISHED'}

# Operator for FK to IK Right SLeg Snap
class OBJECT_OT_fk_to_ik_snap_right_sleg(bpy.types.Operator):
    bl_idname = "object.fk_to_ik_snap_right_sleg"
    bl_label = "SLegR FKIK Snap"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        armature = bpy.context.active_object
        if not armature or armature.type != 'ARMATURE':
            print("Please select an armature object as the active object.")
            return {'CANCELLED'}
        
        current_mode = armature.mode
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
        
        constrained_bones = []
        for source_bone_name, target_bone_name in fk_to_ik_constraint_pairs_right_sleg:
            source_bone = armature.pose.bones.get(source_bone_name)
            target_bone = armature.pose.bones.get(target_bone_name)
            if source_bone and target_bone:
                existing_constraint = None
                for constraint in source_bone.constraints:
                    if constraint.type == 'COPY_TRANSFORMS' and constraint.target == armature and constraint.subtarget == target_bone_name:
                        existing_constraint = constraint
                        break
                if not existing_constraint:
                    constraint = source_bone.constraints.new(type='COPY_TRANSFORMS')
                    constraint.target = armature
                    constraint.subtarget = target_bone_name
                    constraint.target_space = 'WORLD'
                    constraint.owner_space = 'WORLD'
                    constraint.mix_mode = 'REPLACE'
                    constrained_bones.append(source_bone_name)
                    print(f"Created copy transform constraint for {source_bone_name} to {target_bone_name}")
                else:
                    constrained_bones.append(source_bone_name)
                    print(f"Constraint already exists for {source_bone_name} to {target_bone_name}")
            else:
                print(f"One or both bones not found: {source_bone_name} or {target_bone_name}. Available bones: {[pbone.name for pbone in armature.pose.bones]}")
        
        if constrained_bones:
            current_frame = bpy.context.scene.frame_current
            for pbone in armature.pose.bones:
                pbone.bone.select = pbone.name in constrained_bones
            bpy.ops.nla.bake(
                frame_start=current_frame,
                frame_end=current_frame,
                step=1,
                only_selected=True,
                visual_keying=True,
                clear_constraints=True,
                clear_parents=False,
                bake_types={'POSE'},
                use_current_action=True
            )
            for pbone in armature.pose.bones:
                pbone.bone.select = False
        
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode=current_mode)
        return {'FINISHED'}

# Operator for IK to FK Right SLeg Snap
class OBJECT_OT_ik_to_fk_snap_right_sleg(bpy.types.Operator):
    bl_idname = "object.ik_to_fk_snap_right_sleg"
    bl_label = "SLegR IKFK Snap"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        armature = bpy.context.active_object
        if not armature or armature.type != 'ARMATURE':
            print("Please select an armature object as the active object.")
            return {'CANCELLED'}
        
        current_mode = armature.mode
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
        
        constrained_bones = []
        for source_bone_name, target_bone_name in ik_to_fk_constraint_pairs_right_sleg:
            source_bone = armature.pose.bones.get(source_bone_name)
            target_bone = armature.pose.bones.get(target_bone_name)
            if source_bone and target_bone:
                existing_constraint = None
                for constraint in source_bone.constraints:
                    if constraint.type == 'COPY_TRANSFORMS' and constraint.target == armature and constraint.subtarget == target_bone_name:
                        existing_constraint = constraint
                        break
                if not existing_constraint:
                    constraint = source_bone.constraints.new(type='COPY_TRANSFORMS')
                    constraint.target = armature
                    constraint.subtarget = target_bone_name
                    constraint.target_space = 'WORLD'
                    constraint.owner_space = 'WORLD'
                    constraint.mix_mode = 'REPLACE'
                    constrained_bones.append(source_bone_name)
                    print(f"Created copy transform constraint for {source_bone_name} to {target_bone_name}")
                else:
                    constrained_bones.append(source_bone_name)
                    print(f"Constraint already exists for {source_bone_name} to {target_bone_name}")
            else:
                print(f"One or both bones not found: {source_bone_name} or {target_bone_name}. Available bones: {[pbone.name for pbone in armature.pose.bones]}")
        
        if constrained_bones:
            current_frame = bpy.context.scene.frame_current
            for pbone in armature.pose.bones:
                pbone.bone.select = pbone.name in constrained_bones
            bpy.ops.nla.bake(
                frame_start=current_frame,
                frame_end=current_frame,
                step=1,
                only_selected=True,
                visual_keying=True,
                clear_constraints=True,
                clear_parents=False,
                bake_types={'POSE'},
                use_current_action=True
            )
            for pbone in armature.pose.bones:
                pbone.bone.select = False
        
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode=current_mode)
        return {'FINISHED'}

# Operator for FK to IK Left SLeg Snap
class OBJECT_OT_fk_to_ik_snap_left_sleg(bpy.types.Operator):
    bl_idname = "object.fk_to_ik_snap_left_sleg"
    bl_label = "SLegL FKIK Snap"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        armature = bpy.context.active_object
        if not armature or armature.type != 'ARMATURE':
            print("Please select an armature object as the active object.")
            return {'CANCELLED'}
        
        current_mode = armature.mode
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
        
        constrained_bones = []
        for source_bone_name, target_bone_name in fk_to_ik_constraint_pairs_left_sleg:
            source_bone = armature.pose.bones.get(source_bone_name)
            target_bone = armature.pose.bones.get(target_bone_name)
            if source_bone and target_bone:
                existing_constraint = None
                for constraint in source_bone.constraints:
                    if constraint.type == 'COPY_TRANSFORMS' and constraint.target == armature and constraint.subtarget == target_bone_name:
                        existing_constraint = constraint
                        break
                if not existing_constraint:
                    constraint = source_bone.constraints.new(type='COPY_TRANSFORMS')
                    constraint.target = armature
                    constraint.subtarget = target_bone_name
                    constraint.target_space = 'WORLD'
                    constraint.owner_space = 'WORLD'
                    constraint.mix_mode = 'REPLACE'
                    constrained_bones.append(source_bone_name)
                    print(f"Created copy transform constraint for {source_bone_name} to {target_bone_name}")
                else:
                    constrained_bones.append(source_bone_name)
                    print(f"Constraint already exists for {source_bone_name} to {target_bone_name}")
            else:
                print(f"One or both bones not found: {source_bone_name} or {target_bone_name}. Available bones: {[pbone.name for pbone in armature.pose.bones]}")
        
        if constrained_bones:
            current_frame = bpy.context.scene.frame_current
            for pbone in armature.pose.bones:
                pbone.bone.select = pbone.name in constrained_bones
            bpy.ops.nla.bake(
                frame_start=current_frame,
                frame_end=current_frame,
                step=1,
                only_selected=True,
                visual_keying=True,
                clear_constraints=True,
                clear_parents=False,
                bake_types={'POSE'},
                use_current_action=True
            )
            for pbone in armature.pose.bones:
                pbone.bone.select = False
        
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode=current_mode)
        return {'FINISHED'}

# Operator for IK to FK Left SLeg Snap
class OBJECT_OT_ik_to_fk_snap_left_sleg(bpy.types.Operator):
    bl_idname = "object.ik_to_fk_snap_left_sleg"
    bl_label = "SLegL IKFK Snap"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        armature = bpy.context.active_object
        if not armature or armature.type != 'ARMATURE':
            print("Please select an armature object as the active object.")
            return {'CANCELLED'}
        
        current_mode = armature.mode
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
        
        constrained_bones = []
        for source_bone_name, target_bone_name in ik_to_fk_constraint_pairs_left_sleg:
            source_bone = armature.pose.bones.get(source_bone_name)
            target_bone = armature.pose.bones.get(target_bone_name)
            if source_bone and target_bone:
                existing_constraint = None
                for constraint in source_bone.constraints:
                    if constraint.type == 'COPY_TRANSFORMS' and constraint.target == armature and constraint.subtarget == target_bone_name:
                        existing_constraint = constraint
                        break
                if not existing_constraint:
                    constraint = source_bone.constraints.new(type='COPY_TRANSFORMS')
                    constraint.target = armature
                    constraint.subtarget = target_bone_name
                    constraint.target_space = 'WORLD'
                    constraint.owner_space = 'WORLD'
                    constraint.mix_mode = 'REPLACE'
                    constrained_bones.append(source_bone_name)
                    print(f"Created copy transform constraint for {source_bone_name} to {target_bone_name}")
                else:
                    constrained_bones.append(source_bone_name)
                    print(f"Constraint already exists for {source_bone_name} to {target_bone_name}")
            else:
                print(f"One or both bones not found: {source_bone_name} or {target_bone_name}. Available bones: {[pbone.name for pbone in armature.pose.bones]}")
        
        if constrained_bones:
            current_frame = bpy.context.scene.frame_current
            for pbone in armature.pose.bones:
                pbone.bone.select = pbone.name in constrained_bones
            bpy.ops.nla.bake(
                frame_start=current_frame,
                frame_end=current_frame,
                step=1,
                only_selected=True,
                visual_keying=True,
                clear_constraints=True,
                clear_parents=False,
                bake_types={'POSE'},
                use_current_action=True
            )
            for pbone in armature.pose.bones:
                pbone.bone.select = False
        
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode=current_mode)
        return {'FINISHED'}

# Operator for Master Bone Snap
class OBJECT_OT_master_bone_snap(bpy.types.Operator):
    bl_idname = "object.master_bone_snap"
    bl_label = "Master Bone Snap"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        armature = bpy.context.active_object
        if not armature or armature.type != 'ARMATURE':
            print("Please select an armature object as the active object.")
            return {'CANCELLED'}
        
        current_mode = armature.mode
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
        
        constrained_bones = []
        for source_bone_name, target_bone_name in master_bone_constraint_pairs:
            source_bone = armature.pose.bones.get(source_bone_name)
            target_bone = armature.pose.bones.get(target_bone_name)
            if source_bone and target_bone:
                existing_constraint = None
                for constraint in source_bone.constraints:
                    if constraint.type == 'COPY_TRANSFORMS' and constraint.target == armature and constraint.subtarget == target_bone_name:
                        existing_constraint = constraint
                        break
                if not existing_constraint:
                    constraint = source_bone.constraints.new(type='COPY_TRANSFORMS')
                    constraint.target = armature
                    constraint.subtarget = target_bone_name
                    constraint.target_space = 'WORLD'
                    constraint.owner_space = 'WORLD'
                    constraint.mix_mode = 'REPLACE'
                    constrained_bones.append(source_bone_name)
                    print(f"Created copy transform constraint for {source_bone_name} to {target_bone_name}")
                else:
                    constrained_bones.append(source_bone_name)
                    print(f"Constraint already exists for {source_bone_name} to {target_bone_name}")
            else:
                print(f"One or both bones not found: {source_bone_name} or {target_bone_name}. Available bones: {[pbone.name for pbone in armature.pose.bones]}")
        
        if constrained_bones:
            current_frame = bpy.context.scene.frame_current
            for pbone in armature.pose.bones:
                pbone.bone.select = pbone.name in constrained_bones
            bpy.ops.nla.bake(
                frame_start=current_frame,
                frame_end=current_frame,
                step=1,
                only_selected=True,
                visual_keying=True,
                clear_constraints=True,
                clear_parents=False,
                bake_types={'POSE'},
                use_current_action=True
            )
            for pbone in armature.pose.bones:
                pbone.bone.select = False
        
        if current_mode != 'POSE':
            bpy.ops.object.mode_set(mode=current_mode)
        return {'FINISHED'}

# Main Panel for EBR Rig Controls
class VIEW3D_PT_EBR_Rig_Layers_UI(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Item"
    bl_label = "EBR Rig Controls"
    bl_idname = f"VIEW3D_PT_rig_ui_{EBR_rig_id}"
    
    @classmethod
    def poll(cls, context):
        try:
            return context.active_object.data.get("rig_id") == EBR_rig_id
        except (AttributeError, KeyError, TypeError):
            return False

    def draw(self, context):
        pass  # Content moved to subpanels

# Subpanel for IK/FK Snap Controls
class VIEW3D_PT_EBR_Snap_Controls(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Item"
    bl_label = "IK/FK Snap Controls"
    bl_idname = f"VIEW3D_PT_snap_controls_{EBR_rig_id}"
    bl_parent_id = f"VIEW3D_PT_rig_ui_{EBR_rig_id}"
    bl_options = {'DEFAULT_CLOSED'}
    
    @classmethod
    def poll(cls, context):
        try:
            return context.active_object.data.get("rig_id") == EBR_rig_id
        except (AttributeError, KeyError, TypeError):
            return False

    def draw(self, context):
        layout = self.layout
        box = layout.box()
        
        # Arm pairs
        row = box.row()
        row.operator("object.fk_to_ik_snap_right_arm", icon="SNAP_ON", text="ArmR FKIK Snap")
        row.operator("object.ik_to_fk_snap_right_arm", icon="SNAP_ON", text="ArmR IKFK Snap")
        
        row = box.row()
        row.operator("object.fk_to_ik_snap_left_arm", icon="SNAP_ON", text="ArmL FKIK Snap")
        row.operator("object.ik_to_fk_snap_left_arm", icon="SNAP_ON", text="ArmL IKFK Snap")
        
        # Leg pairs
        row = box.row()
        row.operator("object.fk_to_ik_snap_right_leg", icon="SNAP_ON", text="LegR FKIK Snap")
        row.operator("object.ik_to_fk_snap_right_leg", icon="SNAP_ON", text="LegR IKFK Snap")
        
        row = box.row()
        row.operator("object.fk_to_ik_snap_left_leg", icon="SNAP_ON", text="LegL FKIK Snap")
        row.operator("object.ik_to_fk_snap_left_leg", icon="SNAP_ON", text="LegL IKFK Snap")
        
        # MLeg pairs
        row = box.row()
        row.operator("object.fk_to_ik_snap_right_mleg", icon="SNAP_ON", text="MLegR FKIK Snap")
        row.operator("object.ik_to_fk_snap_right_mleg", icon="SNAP_ON", text="MLegR IKFK Snap")
        
        row = box.row()
        row.operator("object.fk_to_ik_snap_left_mleg", icon="SNAP_ON", text="MLegL FKIK Snap")
        row.operator("object.ik_to_fk_snap_left_mleg", icon="SNAP_ON", text="MLegL IKFK Snap")
        
        # SLeg pairs
        row = box.row()
        row.operator("object.fk_to_ik_snap_right_sleg", icon="SNAP_ON", text="SLegR FKIK Snap")
        row.operator("object.ik_to_fk_snap_right_sleg", icon="SNAP_ON", text="SLegR IKFK Snap")
        
        row = box.row()
        row.operator("object.fk_to_ik_snap_left_sleg", icon="SNAP_ON", text="SLegL FKIK Snap")
        row.operator("object.ik_to_fk_snap_left_sleg", icon="SNAP_ON", text="SLegL IKFK Snap")
        
        # Master Bone button
        box.operator("object.master_bone_snap", icon="SNAP_ON", text="Master Bone Snap")

# Subpanel for IK/FK Blend Controls
class VIEW3D_PT_EBR_Blend_Controls(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Item"
    bl_label = "IK/FK Blend Controls"
    bl_idname = f"VIEW3D_PT_blend_controls_{EBR_rig_id}"
    bl_parent_id = f"VIEW3D_PT_rig_ui_{EBR_rig_id}"
    bl_options = {'DEFAULT_CLOSED'}
    
    @classmethod
    def poll(cls, context):
        try:
            return context.active_object.data.get("rig_id") == EBR_rig_id
        except (AttributeError, KeyError, TypeError):
            return False

    def draw(self, context):
        layout = self.layout
        armature = context.active_object.data
        box = layout.box()
        col = box.column()
        
        # Arm Controls section
        row = col.row()
        row.prop(context.scene, "ebr_arm_controls_visible", text="Arm Controls", icon='TRIA_DOWN' if context.scene.ebr_arm_controls_visible else 'TRIA_RIGHT', emboss=False)
        if context.scene.ebr_arm_controls_visible:
            col.prop(armature, '["Left_Arm_FK_IK_Switch"]', text='ArmL IK-FK', slider=True)
            col.prop(armature, '["Right_Arm_FK_IK_Switch"]', text='ArmR IK-FK', slider=True)
        
        # Leg Controls section
        row = col.row()
        row.prop(context.scene, "ebr_leg_controls_visible", text="Leg Controls", icon='TRIA_DOWN' if context.scene.ebr_leg_controls_visible else 'TRIA_RIGHT', emboss=False)
        if context.scene.ebr_leg_controls_visible:
            col.prop(armature, '["Left_Leg_FK_IK_Switch"]', text='LegL IK-FK', slider=True)
            col.prop(armature, '["Right_Leg_FK_IK_Switch"]', text='LegR IK-FK', slider=True)
        
        # Additional Controls section
        row = col.row()
        row.prop(context.scene, "ebr_additional_controls_visible", text="Additional Controls", icon='TRIA_DOWN' if context.scene.ebr_additional_controls_visible else 'TRIA_RIGHT', emboss=False)
        if context.scene.ebr_additional_controls_visible:
            col.prop(armature, '["2nd Center of mass"]', text='2nd Center of Mass', slider=True)
            col.prop(armature, '["Figure Size"]', text='Figure Size', slider=True)
            col.prop(armature, '["Left Top Eyeline/Eyelash"]', text='Left Eyelash/Line', slider=True)
            col.prop(armature, '["Right Top Eyeline/Eyelash"]', text='Right Eyelash/Line', slider=True)
            col.prop(armature, '["Left Bottom Eyeline"]', text='Left Eyeline', slider=True)
            col.prop(armature, '["Right Bottom Eyeline"]', text='Right Eyeline', slider=True)
            col.prop(armature, '["Left Pupil"]', text='Left Pupil', slider=True)
            col.prop(armature, '["Left Pupil 2"]', text='Left Pupil 2', slider=True)
            col.prop(armature, '["Right Pupil"]', text='Right Pupil', slider=True)
            col.prop(armature, '["Right Pupil 2"]', text='Right Pupil 2', slider=True)
            col.prop(armature, '["Eye Glow"]', text='Eye Glow', slider=True)
            col.prop(armature, '["Switch Eye Glow"]', text='Switch Eye Glow', slider=True)
            col.prop(armature, '["Lipstick"]', text='Lipstick', slider=True)

        # Alternative Eyelashes
        row = col.row()
        row.prop(context.scene, "ebr_alternative_eyelashes_visible", text="Alternative Eyelashes", icon='TRIA_DOWN' if context.scene.ebr_alternative_eyelashes_visible else 'TRIA_RIGHT', emboss=False)
        if context.scene.ebr_alternative_eyelashes_visible:
            col.prop(armature, '["Left Eyelash"]', text='Left Eyleash', slider=True)
            col.prop(armature, '["Right Eyelash"]', text='Right Eyleash', slider=True)
            col.prop(armature, '["Eyelash Makeup"]', text='Eyleash Makeup', slider=True)

        # Face Colors section
        row = col.row()
        row.prop(context.scene, "ebr_face_colors_visible", text="Face Colors", icon='TRIA_DOWN' if context.scene.ebr_face_colors_visible else 'TRIA_RIGHT', emboss=False)
        if context.scene.ebr_face_colors_visible:
            col.prop(armature, '["Head Color"]', text='Head Color')
            col.prop(armature, '["Dimples Color"]', text='Dimples Color')
            col.prop(armature, '["Left Eyelash Color"]', text='Left Eyelash Color')
            col.prop(armature, '["Left Eyelash Makeup Color"]', text='Left Eyelash Makeup Color')
            col.prop(armature, '["Right Eyelash Color"]', text='Right Eyelash Color')
            col.prop(armature, '["Right Eyelash Makeup Color"]', text='Right Eyelash Makeup Color')
            col.prop(armature, '["Left Outer Eye Color"]', text='Left Outer Eye Color')
            col.prop(armature, '["Left Inner Eye Color"]', text='Left Inner Eye Color')
            col.prop(armature, '["Left Pupil Color"]', text='Left Pupil Color')
            col.prop(armature, '["Left Pupil 2 Color"]', text='Left Pupil 2 Color')
            col.prop(armature, '["Right Outer Eye Color"]', text='Right Outer Eye Color')
            col.prop(armature, '["Right Inner Eye Color"]', text='Right Inner Eye Color')
            col.prop(armature, '["Right Pupil Color"]', text='Right Pupil Color')
            col.prop(armature, '["Right Pupil 2 Color"]', text='Right Pupil 2 Color')
            col.prop(armature, '["Lipstick Color"]', text='Lipstick Color')
            col.prop(armature, '["Mouth Color"]', text='Mouth Color')
            col.prop(armature, '["Inner Mouth Color"]', text='Inner Mouth Color')
            col.prop(armature, '["Teeth Color"]', text='Teeth Color')
            col.prop(armature, '["Tougue Color"]', text='Tougue Color')
            col.prop(armature, '["Eyelines Color"]', text='Eyelines Color')

# Panel for Rig Layers
class VIEW3D_PT_EBR_IK_FK_Snap_UI(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Item"
    bl_label = "EBR Rig Layers"
    bl_idname = f"VIEW3D_PT_rig_layers_ui_{EBR_rig_id}"
    
    @classmethod
    def poll(cls, context):
        try:
            return context.active_object.data.get("rig_id") == EBR_rig_id
        except (AttributeError, KeyError, TypeError):
            return False

    def draw(self, context):
        layout = self.layout
        current_column = layout.column()
        
        collection_name = "Face"
        bone_collection = bpy.context.object.data.collections_all.get(collection_name)
        if bone_collection:
            current_row = current_column.row()
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_solo', toggle=True, text="", icon="SOLO_ON")
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_visible', toggle=True, text=collection_name)
            current_column.separator()

        collection_name = "Additional Face"
        bone_collection = bpy.context.object.data.collections_all.get(collection_name)
        if bone_collection:
            current_row = current_column.row()
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_solo', toggle=True, text="", icon="SOLO_ON")
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_visible', toggle=True, text=collection_name)
            current_column.separator()
        
        collection_name = "Torso"
        bone_collection = bpy.context.object.data.collections_all.get(collection_name)
        if bone_collection:
            current_row = current_column.row()
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_solo', toggle=True, text="", icon="SOLO_ON")
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_visible', toggle=True, text=collection_name)
            current_column.separator()
        
        collection_name = "FK Arm.R"
        bone_collection = bpy.context.object.data.collections_all.get(collection_name)
        if bone_collection:
            current_row = current_column.row()
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_solo', toggle=True, text="", icon="SOLO_ON")
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_visible', toggle=True, text=collection_name)
        
        collection_name = "IK Arm.R"
        bone_collection = bpy.context.object.data.collections_all.get(collection_name)
        if bone_collection:
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_visible', toggle=True, text=collection_name)
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_solo', toggle=True, text="", icon="SOLO_ON")
            current_column.separator()
        
        collection_name = "FK Arm.L"
        bone_collection = bpy.context.object.data.collections_all.get(collection_name)
        if bone_collection:
            current_row = current_column.row()
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_solo', toggle=True, text="", icon="SOLO_ON")
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_visible', toggle=True, text=collection_name)
        
        collection_name = "IK Arm.L"
        bone_collection = bpy.context.object.data.collections_all.get(collection_name)
        if bone_collection:
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_visible', toggle=True, text=collection_name)
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_solo', toggle=True, text="", icon="SOLO_ON")
            current_column.separator()
        
        collection_name = "FK Leg.L"
        bone_collection = bpy.context.object.data.collections_all.get(collection_name)
        if bone_collection:
            current_row = current_column.row()
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_solo', toggle=True, text="", icon="SOLO_ON")
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_visible', toggle=True, text=collection_name)
        
        collection_name = "IK Leg.L"
        bone_collection = bpy.context.object.data.collections_all.get(collection_name)
        if bone_collection:
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_visible', toggle=True, text=collection_name)
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_solo', toggle=True, text="", icon="SOLO_ON")
            current_column.separator()
        
        collection_name = "FK Leg.R"
        bone_collection = bpy.context.object.data.collections_all.get(collection_name)
        if bone_collection:
            current_row = current_column.row()
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_solo', toggle=True, text="", icon="SOLO_ON")
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_visible', toggle=True, text=collection_name)
        
        collection_name = "IK Leg.R"
        bone_collection = bpy.context.object.data.collections_all.get(collection_name)
        if bone_collection:
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_visible', toggle=True, text=collection_name)
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_solo', toggle=True, text="", icon="SOLO_ON")
            current_column.separator()
        
        collection_name = "Root"
        bone_collection = bpy.context.object.data.collections_all.get(collection_name)
        if bone_collection:
            current_row = current_column.row()
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_solo', toggle=True, text="", icon="SOLO_ON")
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_visible', toggle=True, text=collection_name)
            current_column.separator()
        
        collection_name = "Figure Height"
        bone_collection = bpy.context.object.data.collections_all.get(collection_name)
        if bone_collection:
            current_row = current_column.row()
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_solo', toggle=True, text="", icon="SOLO_ON")
            current_nested_row = current_row.row()
            current_nested_row.prop(bone_collection, 'is_visible', toggle=True, text=collection_name)

# Classes to register
classes = (
    OBJECT_OT_fk_to_ik_snap_right_arm,
    OBJECT_OT_fk_to_ik_snap_left_arm,
    OBJECT_OT_ik_to_fk_snap_right_arm,
    OBJECT_OT_ik_to_fk_snap_left_arm,
    OBJECT_OT_fk_to_ik_snap_right_leg,
    OBJECT_OT_ik_to_fk_snap_right_leg,
    OBJECT_OT_fk_to_ik_snap_left_leg,
    OBJECT_OT_ik_to_fk_snap_left_leg,
    OBJECT_OT_fk_to_ik_snap_right_mleg,
    OBJECT_OT_ik_to_fk_snap_right_mleg,
    OBJECT_OT_fk_to_ik_snap_left_mleg,
    OBJECT_OT_ik_to_fk_snap_left_mleg,
    OBJECT_OT_fk_to_ik_snap_right_sleg,
    OBJECT_OT_ik_to_fk_snap_right_sleg,
    OBJECT_OT_fk_to_ik_snap_left_sleg,
    OBJECT_OT_ik_to_fk_snap_left_sleg,
    OBJECT_OT_master_bone_snap,
    VIEW3D_PT_EBR_Rig_Layers_UI,
    VIEW3D_PT_EBR_Snap_Controls,
    VIEW3D_PT_EBR_Blend_Controls,
    VIEW3D_PT_EBR_IK_FK_Snap_UI,
)

# Registration function
def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.ebr_arm_controls_visible = BoolProperty(default=True)
    bpy.types.Scene.ebr_leg_controls_visible = BoolProperty(default=True)
    bpy.types.Scene.ebr_additional_controls_visible = BoolProperty(default=True)
    bpy.types.Scene.ebr_alternative_eyelashes_visible = BoolProperty(default=True)
    bpy.types.Scene.ebr_face_colors_visible = BoolProperty(default=True)

# Unregistration function
def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.ebr_arm_controls_visible
    del bpy.types.Scene.ebr_leg_controls_visible
    del bpy.types.Scene.ebr_additional_controls_visible
    del bpy.types.Scene.ebr_alternative_eyelashes_visible
    del bpy.types.Scene.ebr_face_colors_visible

if __name__ == "__main__":
    register()
# This module maps joint data with avatar's joints
from bge import logic
from mathutils import Vector, Quaternion
from blenderCoordinates import RelativeJointInfo


# get the current scene
scene = logic.getCurrentScene()
# get objects from the scene
obj = scene.objects
# get the metarig object
met = obj.get('metarig')
# get the armature channel
main_arm = met.channels

# get specified bones of avatar
head = main_arm.get('head')
#shoulder_R = main_arm.get('shoulder.R')
#shoulder_L = main_arm.get('shoulder.L')
upper_arm_R = main_arm.get('upper_arm.R')
forearm_R = main_arm.get('forearm.R')
upper_arm_L = main_arm.get('upper_arm.L')
forearm_L = main_arm.get('forearm.L')
#chest = main_arm.get('chest')
spine = main_arm.get('spine')
thigh_R = main_arm.get('thigh.R')
thigh_L = main_arm.get('thigh.L')
shin_R = main_arm.get('shin.R')
shin_L = main_arm.get('shin.L')

# get old direction vector of all the specified bones
old_headV = head.pose_head - head.pose_tail
#old_shoulder_RV = shoulder_R.pose_head - shoulder_L.pose_tail
#old_shoulder_LV = shoulder_R.pose_head - shoulder_R.pose_tail
old_upper_arm_RV = upper_arm_R.pose_head - upper_arm_R.pose_tail
old_upper_arm_LV = upper_arm_L.pose_head - upper_arm_L.pose_tail
old_forearm_RV = forearm_R.pose_head - forearm_R.pose_tail
old_forearm_LV = forearm_L.pose_head - forearm_L.pose_tail
#old_chestV = chest.pose_head - chest.pose_tail
old_spineV = spine.pose_head - spine.pose_tail
old_thigh_RV = thigh_R.pose_head - thigh_R.pose_tail
old_thigh_LV = thigh_L.pose_head - thigh_L.pose_tail
old_shin_RV = shin_R.pose_head - shin_R.pose_tail
old_shin_LV = shin_L.pose_head - shin_L.pose_tail

# get the RelativeJointInfo object
rj = RelativeJointInfo()

# Scale factor
sf = 50

# get new direction vector of all the specified bones
# gotcha - right -> left; left -> right
new_headV = (rj.head - rj.neck)/sf
#new_shoulder_RV = (rj.neck - rj.left_shoulder)/sf
#new_shoulder_LV = (rj.neck - rj.right_shoulder)/sf
new_upper_arm_RV = (rj.left_shoulder - rj.left_elbow)/sf
new_upper_arm_LV = (rj.right_shoulder - rj.right_elbow)/sf
new_forearm_RV = (rj.left_elbow - rj.left_hand)/sf
new_forearm_LV = (rj.right_elbow - rj.right_hand)/sf
new_spineV = spine.pose_head - spine.pose_tail #rj.torso # check which is head and pose_tail
#new_chestV = (spine.pose_tail - rj.neck)
new_thigh_RV = (rj.left_hip - rj.left_knee)/sf
new_thigh_LV = (rj.right_hip - rj.right_knee)/sf
new_shin_RV = (rj.left_knee - rj.left_foot)/sf
new_shin_LV = (rj.right_knee -rj.right_foot)/sf

# get rotation Quaternion between old and new bone direction vectors
quat_head = old_headV.rotation_difference(new_headV)
#quat_shoulder_R = old_shoulder_RV.rotation_difference(new_shoulder_RV)
#quat_shoulder_L = old_shoulder_LV.rotation_difference(new_shoulder_LV)
quat_upper_arm_R = old_upper_arm_RV.rotation_difference(new_upper_arm_RV)
quat_upper_arm_L = old_upper_arm_LV.rotation_difference(new_upper_arm_LV)
quat_forearm_R = old_forearm_RV.rotation_difference(new_forearm_RV)
quat_forearm_L = old_forearm_LV.rotation_difference(new_forearm_LV)
quat_spine = old_spineV.rotation_difference(new_spineV)
#quat_chest = old_chestV.rotation_difference(new_chestV)
quat_thigh_R = old_thigh_RV.rotation_difference(new_thigh_RV)
quat_thigh_L = old_thigh_LV.rotation_difference(new_thigh_LV)
quat_shin_R = old_shin_RV.rotation_difference(new_shin_RV)
quat_shin_L = old_shin_LV.rotation_difference(new_shin_LV)

# update the bone joints
head.joint_rotation = [quat_head[1], quat_head[2], quat_head[3]]
#shoulder_R.joint_rotation = [quat_shoulder_R[1], quat_shoulder_R[2], quat_shoulder_R[3]]
#shoulder_L.joint_rotation = [quat_shoulder_L[1], quat_shoulder_L[2], quat_shoulder_L[3]]
upper_arm_R.joint_rotation = [quat_upper_arm_R[1], quat_upper_arm_R[2], quat_upper_arm_R[3]]
upper_arm_L.joint_rotation = [quat_upper_arm_L[1], quat_upper_arm_L[2], quat_upper_arm_L[3]]
forearm_R.joint_rotation = [quat_forearm_R[1], quat_forearm_R[2], quat_forearm_R[3]]
forearm_L.joint_rotation = [quat_forearm_L[1], quat_forearm_L[2], quat_forearm_L[3]]
#chest.joint_rotation = [quat_chest[1], quat_chest[2], quat_chest[3]]
spine.joint_rotation = [quat_spine[1], quat_spine[2], quat_spine[3]]
thigh_R.joint_rotation = [quat_thigh_R[1], quat_thigh_R[2], quat_thigh_R[3]]
thigh_L.joint_rotation = [quat_thigh_L[1], quat_thigh_L[2], quat_thigh_L[3]]
shin_R.joint_rotation = [quat_shin_R[1], quat_shin_R[2], quat_shin_R[3]]
shin_L.joint_rotation = [quat_shin_L[1], quat_shin_L[2], quat_shin_L[3]]
met.update()

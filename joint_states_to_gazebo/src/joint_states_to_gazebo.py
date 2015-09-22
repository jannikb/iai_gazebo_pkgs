#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('joint_states_to_gazebo')
import rospy
from gazebo_msgs.srv import SetModelConfiguration
from sensor_msgs.msg import JointState

service_name = 'gazebo/set_model_configuration'
# joint_names = ['right_arm_0_joint', 'right_arm_1_joint', 'right_arm_2_joint',
#                 'right_arm_3_joint', 'right_arm_4_joint', 'right_arm_5_joint',
#                 'right_arm_6_joint', 'left_arm_0_joint', 'left_arm_1_joint',
#                 'left_arm_2_joint', 'left_arm_3_joint', 'left_arm_4_joint',
#                 'left_arm_5_joint', 'left_arm_6_joint', 'triangle_base_joint',
#                 'neck_shoulder_pan_joint', 'neck_shoulder_lift_joint', 'neck_elbow_joint',
#                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

def js_callback(msg):
    set_model_configuration = rospy.ServiceProxy(service_name, SetModelConfiguration)
    # req = SetModelConfiguration()
    # req.model_name = 'ur3'
    # req.urdf_param_name = 'robot_description'
    # req.joint_names = joint_names
    # req.joint_positions = msg.position
    resp = set_model_configuration('boxy', 'robot_description', msg.name, msg.position) #[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    if not resp.success:
        rospy.logwarn(resp.status_message)

def main():
    try:
        rospy.init_node("joint_states_to_gazebo", anonymous=True, disable_signals=True)
        rospy.wait_for_service(service_name)
        sub = rospy.Subscriber('joint_states', JointState, js_callback)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()

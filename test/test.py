import sys
import rospy
import numpy as np
import geometry_msgs.msg as geom_msg
import time
import subprocess
from dynamic_reconfigure.client import Client
from absl import app, flags, logging
from scipy.spatial.transform import Rotation as R
import os

FLAGS = flags.FLAGS
flags.DEFINE_string("robot_ip", None, "IP address of the robot.", required=True)
flags.DEFINE_string("load_gripper", 'false', "Whether or not to load the gripper.")


def main(_):
    try:
        input("\033[33mPress enter to start roscore and the impedance controller.\033[0m")
        try:
            roscore = subprocess.Popen('roscore')
            time.sleep(1)
        except:
            pass

        impedence_controller = subprocess.Popen(['roslaunch', 'serl_franka_controllers', 'impedance.launch',
                                                f'robot_ip:={FLAGS.robot_ip}', f'load_gripper:={FLAGS.load_gripper}'],
                                                stdout=subprocess.PIPE)

        eepub = rospy.Publisher('/cartesian_impedance_controller/equilibrium_pose', geom_msg.PoseStamped, queue_size=10)
        rospy.init_node('franka_control_api')
        client = Client("/cartesian_impedance_controllerdynamic_reconfigure_compliance_param_node")

        # Reset the arm
        msg = geom_msg.PoseStamped()
        msg.header.frame_id = "0"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position = geom_msg.Point(0.5, 0, 0.2)
        quat = R.from_euler('xyz', [np.pi, 0, np.pi/2]).as_quat()
        msg.pose.orientation = geom_msg.Quaternion(quat[0], quat[1], quat[2], quat[3])
        input("\033[33m\nObserve the surroundings. Press enter to move the robot to the initial position.\033[0m")
        eepub.publish(msg)
        time.sleep(1)

        time.sleep(1)
        # Setting the reference limiting values through ros dynamic reconfigure
        for direction in ['x', 'y', 'z', 'neg_x', 'neg_y', 'neg_z']:
            client.update_configuration({"translational_clip_" + direction: 0.005})
            client.update_configuration({"rotational_clip_" + direction: 0.04})
        time.sleep(1)
        print("\nNew reference limiting values has been set")

        time.sleep(1)
        input("\033[33mPress enter to move the robot up with the reference limiting engaged. Notice that the arm motion should be slower this time because the maximum force is effectively limited. \033[0m")
        for i in range(10):
            msg = geom_msg.PoseStamped()
            msg.header.frame_id = "0"
            msg.header.stamp = rospy.Time.now()
            msg.pose.position = geom_msg.Point(0.5, 0, 0.2+i*0.02)
            quat = R.from_euler('xyz', [np.pi, 0, np.pi/2]).as_quat()
            msg.pose.orientation = geom_msg.Quaternion(quat[0], quat[1], quat[2], quat[3])
            eepub.publish(msg)
            time.sleep(0.2)
        time.sleep(1)

        time.sleep(1)
        input("\033[33m\nPress enter to reset the robot arm back to the initial pose. \033[0m")
        for i in range(10):
            msg = geom_msg.PoseStamped()
            msg.header.frame_id = "0"
            msg.header.stamp = rospy.Time.now()
            msg.pose.position = geom_msg.Point(0.5, 0, 0.4-i*0.02)
            quat = R.from_euler('xyz', [np.pi, 0, np.pi/2]).as_quat()
            msg.pose.orientation = geom_msg.Quaternion(quat[0], quat[1], quat[2], quat[3])
            eepub.publish(msg)
            time.sleep(0.1)

        input("\033[33m\n \nPress enter to exit the test and stop the controller.\033[0m")
        impedence_controller.terminate()
        roscore.terminate()
        sys.exit()
    except:
        rospy.logerr("Error occured. Terminating the controller.")
        impedence_controller.terminate()
        roscore.terminate()
        sys.exit()


if __name__ == "__main__":
    app.run(main)

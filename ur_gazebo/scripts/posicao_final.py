#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String

def get_current_pose():
    # Inicializando o MoveIt! e o nó ROS (se não estiverem inicializados)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('get_current_pose_node', anonymous=True)

    # Criando o objeto RobotCommander
    robot = moveit_commander.RobotCommander()

    # Obtendo a pose atual do efetuador final
    current_pose = robot.get_current_state().get_global_link_transform("end_effector_link")  #precisa substituir "end_effector_link" pelo no correto

    # Convertendo a pose para uma lista
    pose_list = pose_to_list(current_pose)

    # Publicando a posição do efetuador final em um tópico
    pub = rospy.Publisher('current_pose', String, queue_size=10)
    pose_str = f"x: {pose_list[0]}, y: {pose_list[1]}, z: {pose_list[2]}"
    pub.publish(pose_str)

    # Saindo do MoveIt!
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    try:
        get_current_pose()
    except rospy.ROSInterruptException:
        pass
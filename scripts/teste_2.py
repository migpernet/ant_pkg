#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String

def move_to_position(target_pose):
    # Inicializando o moveit_commander e o nó ROS
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_ur5_node', anonymous=True)

    # Inicializando o grupo de controle do MoveIt!
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")  # Nome do grupo do UR5 no MoveIt

    # Definindo a pose-alvo
    pose_target = geometry_msgs.msg.Pose()   #Cria uma mensagem Pose para definir a posição e a orientação desejada.
    pose_target.orientation.w = target_pose['orientation_w']
    pose_target.position.x = target_pose['position_x']
    pose_target.position.y = target_pose['position_y']
    pose_target.position.z = target_pose['position_z']

    group.set_pose_target(pose_target)  #Define a pose-alvo para o grupo de juntas.

    # Planejamento do movimento
    plan = group.go(wait=True)

    # Garantindo que o robô não se mova mais após alcançar a pose
    group.stop()

    # Limpando os alvos
    group.clear_pose_targets()

    # Saindo do MoveIt!
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    try:
        # Defina aqui a pose-alvo desejada (posição e orientação)
        target_pose = {
            'orientation_w': 1.0,     #1.0
            'position_x': 0.4,        #0.4
            'position_y': 0.1,        #0.1
            'position_z': 0.4         #0.4
        }

        # Chamando a função para mover o robô para a posição-alvo
        move_to_position(target_pose)

    except rospy.ROSInterruptException:
        pass

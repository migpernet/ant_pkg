#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time

def all_close(goal, actual, tolerance):
    """
    Verifica se todas as posições da junta estão dentro da tolerância
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False
    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    return True

def move_to_pose(group, pose_goal):
    """
    Move o braço robótico para a pose desejada
    """
    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    return plan

def main():
    # Inicializa o ROS e MoveIt!
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    # Inicializa o objeto `RobotCommander` (informações do robô)
    robot = moveit_commander.RobotCommander()

    # Inicializa o objeto `PlanningSceneInterface` (informações sobre o ambiente)
    scene = moveit_commander.PlanningSceneInterface()

    # Inicializa o grupo de controle do braço robótico
    group_name = "manipulator"  # Nome do grupo de juntas do braço
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Define a taxa de atualização
    rate = rospy.Rate(10)

    # Posição inicial
    print("Movendo para a posição inicial...")
    initial_joint_goal = move_group.get_current_joint_values()
    initial_joint_goal[0] = 0.0  # ombro
    initial_joint_goal[1] = -1.57  # elevação  # -1.57
    initial_joint_goal[2] = 0.0  # cotovelo  # 1.57
    initial_joint_goal[3] = 0.0  # punho 1
    initial_joint_goal[4] = 0.0  # punho 2
    initial_joint_goal[5] = 0.0  # punho 3
    move_group.go(initial_joint_goal, wait=True)
    move_group.stop()

    # Move para posição A (pegar o objeto)
    print("Movendo para a posição A (pegando o objeto)...")
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.5
    pose_goal.position.y = 0.0
    pose_goal.position.z = 0.4
    move_to_pose(move_group, pose_goal)
    
    # Simula o tempo para pegar o objeto
    print("Pegando o objeto...")
    time.sleep(2)  # Simula a ação de pegar o objeto

    # Move para posição B (levar o objeto)
    print("Movendo para a posição B (levando o objeto)...")
    pose_goal.position.x = 0.0
    pose_goal.position.y = 0.5
    pose_goal.position.z = 0.4
    move_to_pose(move_group, pose_goal)

    # Simula o tempo para soltar o objeto
    print("Soltando o objeto...")
    time.sleep(2)  # Simula a ação de soltar o objeto

    # Retorna para a posição inicial
    print("Retornando para a posição inicial...")
    move_group.go(initial_joint_goal, wait=True)
    move_group.stop()

    print("Movimento completo!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

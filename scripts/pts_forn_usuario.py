#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import copy

def get_user_input():
    """Função para obter entradas de posição do usuário."""
    x = float(input("Insira a posição X: "))
    y = float(input("Insira a posição Y: "))
    z = float(input("Insira a posição Z: "))
    return x, y, z

def main():
    # Inicializar o sistema ROS e o moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_user_input_trajectory', anonymous=True)

    # Inicializar o RobotCommander, PlanningSceneInterface e MoveGroupCommander
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"  # Para o UR5
    group = moveit_commander.MoveGroupCommander(group_name)

    # Definir o quadro de referência e velocidade/aceleração máximas
    group.set_pose_reference_frame("base_link")
    group.set_max_velocity_scaling_factor(0.5)
    group.set_max_acceleration_scaling_factor(0.5)

    # Definir tolerância de posicionamento e orientação
    group.set_goal_position_tolerance(0.01)
    group.set_goal_orientation_tolerance(0.01)

    # Armazenar a pose inicial do robô
    start_pose = group.get_current_pose().pose

    # Lista de waypoints para o robô seguir
    waypoints = []

    # Adicionar a pose inicial à lista de waypoints
    waypoints.append(copy.deepcopy(start_pose))

    # Perguntar ao usuário quantos pontos de passagem ele deseja definir
    num_points = int(input("Quantos pontos de passagem você deseja definir? "))

    for i in range(num_points):
        print(f"Definindo ponto de passagem {i+1}:")
        x, y, z = get_user_input()  # Obter coordenadas do usuário

        # Criar nova pose baseada nas entradas do usuário
        wpose = copy.deepcopy(start_pose)
        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = z

        # Adicionar o novo ponto de passagem à lista de waypoints
        waypoints.append(copy.deepcopy(wpose))

    # Planejar a trajetória cartesian para seguir os waypoints
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,   # Lista de waypoints
        0.01,        # Resolução - pequenos incrementos em metros
        0.0)         # Permitir zero saltos

    print("Fração do caminho planejado: {}".format(fraction))

    # Executar a trajetória planejada
    group.execute(plan, wait=True)

    # Parar o movimento residual
    group.stop()

    # Limpar os objetivos do robô
    group.clear_pose_targets()

    # Finalizar a sessão MoveIt
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()

#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import copy

def main():
    # Inicializar o sistema ROS e o moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_complex_trajectory', anonymous=True)

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

    # Definir uma série de pontos de passagem (waypoints)
    wpose = copy.deepcopy(start_pose)

    # Primeiro ponto de passagem: subir o robô em 10 cm no eixo Z
    wpose.position.z += 0.1
    waypoints.append(copy.deepcopy(wpose))

    # Segundo ponto de passagem: mover para frente no eixo X
    wpose.position.x += 0.2
    waypoints.append(copy.deepcopy(wpose))

    # Terceiro ponto de passagem: mover lateralmente no eixo Y
    wpose.position.y += 0.2
    waypoints.append(copy.deepcopy(wpose))

    # Quarto ponto de passagem: descer o robô em 10 cm no eixo Z
    wpose.position.z -= 0.1
    waypoints.append(copy.deepcopy(wpose))

    # Quinto ponto de passagem: mover para trás no eixo X
    wpose.position.x -= 0.2
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
    try:
        main()
    except rospy.ROSInterruptException:
        pass




    

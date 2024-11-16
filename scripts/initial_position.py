#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def set_initial_positions():
    rospy.init_node('set_initial_positions')
    
    # Publicador para o controlador de trajetória
    pub = rospy.Publisher('/joint_trajectory_controller/command', JointTrajectory, queue_size=10)
    
    # Aguardar o publicador estar ativo
    rospy.sleep(1)

    # Criar a mensagem de comando
    traj = JointTrajectory()
    traj.joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    ]

    # Definir posições
    point = JointTrajectoryPoint()
    point.positions = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]  # Posições em radianos
    point.time_from_start = rospy.Duration(3.0)  # Tempo para alcançar a posição

    traj.points.append(point)

    # Publicar o comando
    pub.publish(traj)
    rospy.loginfo("Posições iniciais enviadas.")

if __name__ == '__main__':
    try:
        set_initial_positions()
    except rospy.ROSInterruptException:
        pass

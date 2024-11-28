#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from std_srvs.srv import Trigger, TriggerResponse
from moveit_msgs.msg import CollisionObject
import geometry_msgs.msg

def handle_init_request(req):
    rospy.loginfo("Inicializando MoveIt! e ROS...")

    # Inicializando o MoveIt! e o nó ROS
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('init_node', anonymous=True)

    # Criando o objeto RobotCommander
    robot = moveit_commander.RobotCommander()

    # Criando o objeto PlanningSceneInterface
    scene = moveit_commander.PlanningSceneInterface()

    # Definindo um objeto de colisão (exemplo de um cubo)
    collision_object = CollisionObject()
    collision_object.header.frame_id = "world"  # Frame de referência do objeto
    collision_object.id = "my_obstacle"  # ID do objeto
    collision_object.primitives.append(geometry_msgs.msg.Mesh())
    # ... (Defina a forma do objeto, por exemplo, um cubo)
    collision_object.primitive_poses.append(geometry_msgs.msg.Pose())
    # ... (Defina a posição do objeto)
    collision_object.operation = CollisionObject.ADD

    # Adicionando o objeto à cena
    scene.add_collision_object(collision_object)

    rospy.loginfo("Inicialização concluída!")
    return TriggerResponse(success=True, message="Inicialização realizada com sucesso")

if __name__ == "__main__":
    rospy.init_node('init_node')
    service = rospy.Service('initialize', Trigger, handle_init_request)
    rospy.spin()
import rospy
import moveit_commander
from urdf_parser_py.urdf import URDF

# Inicializa o ROS node
rospy.init_node("get_joint_limits", anonymous=True)

# Carrega o modelo URDF do robô
robot = URDF.from_parameter_server()

# Inicializa o MoveIt Commander
moveit_commander.roscpp_initialize([])

# Extrai os limites das juntas
joint_limits = {}
for joint in robot.joints:
    if joint.type != "fixed":
        # Verifica se a junta tem limites definidos
        if joint.limit:
            joint_limits[joint.name] = (joint.limit.lower, joint.limit.upper)

print("Joint limits for UR5:", joint_limits)

#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import Float64
from controller_manager_msgs.srv import LoadController, SwitchController
from urdf_parser_py.urdf import URDF


def main():
    rospy.init_node("rotor_controller_spawner", anonymous=True)

    # get robot_description
    rotor_controller_params = None
    if not rospy.has_param("robot_description"):
        rospy.logerr("Parameter 'robot_description' not found.")
        return
    urdf_str = rospy.get_param("robot_description")

    # parse URDF to ensure
    robot = None
    try:
        robot = URDF.from_xml_string(urdf_str)
    except Exception as e:
        rospy.logerr("Failed to parse URDF: %s", e)
        return

    rotor_joints = [j.name for j in robot.joints if "rotor" in j.name]
    rotor_joints.sort()
    rospy.loginfo("Found {} rotors".format(len(rotor_joints)))

    # wait for controller manager services to be available
    try:
        rospy.wait_for_service("controller_manager/load_controller", timeout=10.0)
        rospy.wait_for_service("controller_manager/switch_controller", timeout=10.0)
    except rospy.ROSException:
        rospy.logerr("Controller manager services not available.")
        sys.exit(1)

    load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
    switch_srv = rospy.ServiceProxy(
        "controller_manager/switch_controller", SwitchController
    )

    common_type = "rotor_controllers/RotorController"

    # set parameters for each controller and prepare list of controllers to spawn
    controllers_to_spawn = []
    for i in range(len(rotor_joints)):
        controller_name = f"rotor_controller/controller{i+1}"
        rospy.set_param(f"{controller_name}/type", common_type)
        rospy.set_param(f"{controller_name}/joint", rotor_joints[i])
        controllers_to_spawn.append(controller_name)
        rospy.sleep(0.1)

    for controller in controllers_to_spawn:
        load_srv(controller)
        rospy.loginfo(f"load controller: {controller}")

        # wait
        pub = rospy.Publisher(f"{controller}/command", Float64, queue_size=10)
        while pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.2)

        # switch
        resp = switch_srv(
            start_controllers=[controller],
            stop_controllers=[],
            strictness=2,  # STRICT
            start_asap=True,
            timeout=0.0,
        )
        if resp.ok:
            rospy.loginfo(f"start controller: {controller}")
        else:
            rospy.logerr(f"Failed to switch controller: {controller}")
        rospy.sleep(0.5)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

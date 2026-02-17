#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import Float64
from controller_manager_msgs.srv import LoadController, SwitchController


def main():
    rospy.init_node("rotor_controller_spawner", anonymous=True)

    # get paramter from "rotor_controller" namespace
    rotor_controller_params = None
    try:
        rotor_controller_params = rospy.get_param("rotor_controller", None)
    except KeyError:
        rospy.logerr(f"No parameters found in namespace '{target_ns}'.")
        sys.exit(1)

    rospy.loginfo("Waiting for controller_manager services...")

    # wait for controller manager services to be available
    try:
        rospy.wait_for_service("controller_manager/load_controller", timeout=10.0)
        rospy.wait_for_service("controller_manager/switch_controller", timeout=10.0)
    except rospy.ROSException:
        rospy.logerr("Controller manager services not available.")
        sys.exit(1)

    rospy.wait_for_service("controller_manager/load_controller")
    load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
    rospy.wait_for_service("controller_manager/switch_controller")
    switch_srv = rospy.ServiceProxy("controller_manager/switch_controller", SwitchController)

    common_type = rotor_controller_params["type"]
    controllers_to_spawn = []

    # set parameters for each controller and prepare list of controllers to spawn
    for key, value in rotor_controller_params.items():
        if key.startswith("controller"):
            rospy.set_param(f"rotor_controller/{key}/type", common_type)
            rospy.sleep(0.1)  # ensure parameter is set before loading controller

            controllers_to_spawn.append(f"rotor_controller/{key}")
    rospy.sleep(1.0)  # wait for parameters to be set

    controller_publishers = {}
    for controller in controllers_to_spawn:
        load_srv(controller)
        rospy.loginfo(f"loading controller: {controller}")

        # wait
        pub = rospy.Publisher(f"{controller}/command", Float64, queue_size=10)
        # controller_publishers[controller] = pub
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

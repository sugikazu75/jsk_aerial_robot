#!/bin/bash

# apply a constant vertical force (e.g., cable weight) equally divided to the gimbal links in gazebo
# root link is not used, since apply_external_wrench.py clears the wrenches on it
#
# usage:
#   apply_gimbal_link_force.sh [total_force_z [N]] [robot_ns] [link_num]   # apply (default: -2.0 gimbalrotor 4)
#   apply_gimbal_link_force.sh clear [robot_ns] [link_num]                  # clear

if [ "$1" = "clear" ]; then
  ROBOT_NS=${2:-gimbalrotor}
  LINK_NUM=${3:-4}
  for i in $(seq 1 ${LINK_NUM}); do
    rosservice call /gazebo/clear_body_wrenches "{body_name: '${ROBOT_NS}::gimbal${i}_link'}"
  done
  echo "cleared the force on ${LINK_NUM} gimbal links"
  exit 0
fi

TOTAL_FORCE_Z=${1:--2.0}
ROBOT_NS=${2:-gimbalrotor}
LINK_NUM=${3:-4}
FORCE_Z=$(printf "%.4f" $(echo "${TOTAL_FORCE_Z} / ${LINK_NUM}" | bc -l))

for i in $(seq 1 ${LINK_NUM}); do
  rosservice call /gazebo/apply_body_wrench "{body_name: '${ROBOT_NS}::gimbal${i}_link', reference_frame: 'world', wrench: {force: {x: 0.0, y: 0.0, z: ${FORCE_Z}}}, start_time: {secs: 0, nsecs: 0}, duration: {secs: -1, nsecs: 0}}" || exit 1
done
echo "applied ${TOTAL_FORCE_Z} [N] in total (${FORCE_Z} [N] x ${LINK_NUM}) to the gimbal links"

#!/usr/bin/env python3
"""The perching task as a SMACH state machine: the trajectory of follow_trajectory.py cut at the hooking,
with the operator checking it and teleoperating when it went wrong.

  rosrun gimbalrotor perching_task.py __ns:=gimbalrotor [_takeoff:=true]

  TAKEOFF      on the ground the joints to ~ground_joints (the first frame); with ~takeoff the motors
               armed and the takeoff; the hover
  GO_TO_START  the CoG to the first frame, then the joints; held ~settle_time
  FLIGHT       the trajectory up to the start of the approach (~approach_window before the contact)
  HOOK         held there until the robot converges (~converge_*), then the approach at
               ~approach_speed_scale: the hook through its opening onto the bar
                 not converged in ~converge_timeout -> TELEOP
  JUDGE_HOOK   the operator: OK (R1, ~ok) hooked -> AFTER_HOOK; NG (R2, ~ng) -> TELEOP
  TELEOP       no target sent: the navigation takes the joystick (sticks, its usual buttons); the
               joints stay as they were. OK (R1, ~ok) -> RETURN; NG (R2, ~ng) -> finish (holding)
  RETURN       the contacts switched off; from where the robot is back to the start of the approach
               (the CoG in a straight line: clear the bar by hand first), then HOOK again
  AFTER_HOOK   the first contact (the hook) switched on, then the rest of the trajectory (the load
               onto the bar, the hang, the reach to the wall, the push), each other contact switched
               on and off where it begins and ends in the file; the last frame held.
               With ~after_hook "stop": only the hook switched on, the task ends there (outcome
               "stopped"), holding the hooked pose: e.g. for hanging_teleop.py next
               With ~after_hook "unhook": HOOKED instead
  HOOKED       (after_hook "unhook") the hook contact switched on, the hooked pose held; the operator:
               OK (R1) -> UNHOOK; NG (R2): the task ends with the contact on, holding ("succeeded")
  UNHOOK       the contacts switched off, held through their ramp_time and ~unhook_wait, then the
               approach backwards at ~approach_speed_scale (the hook out through its opening), the start
               of the approach held; the task ends ("unhooked"): the rest by teleoperation
               With a "release" event in the file (plan_perching_motion.py), the trajectory up to it
  RELEASE      the frame there held; the operator: OK (R1, ~ok) every contact switched off and the
               rest of the file played (the load back onto the rotors, the rear arm back, the hook out
               of the bar with the body going down); without the event, held while the load moves
               back (the ramp_time of the contacts). Then the task ends ("released"): away from the
               bar and land by teleoperation. NG (R2, ~ng): the task ends with the contacts as they
               are ("succeeded")

The contacts are those of the whole-body controller (bringup whole_body:=true), switched through
<ns>/whole_body/contacts/<name>/activate (std_srvs/SetBool) with the names of the trajectory file
(~switch_contacts false: not switched; the controller not running: a warning, and the task goes on).

The operator answers with the joystick or a topic: ok_button / ng_button, raw indices of
sensor_msgs/Joy on <ns>/joy, pressed after a release (a button held from before does not count), or
std_msgs/Empty on ~ok / ~ng (<ns>/perching_task/ok, .../ng; only what is sent while the question is
asked counts):

  rostopic pub -1 /gimbalrotor/perching_task/ok std_msgs/Empty

Every state aborts if the robot leaves the hover state. Playback parameters are in
config/quad_with_dual_arm/follow_trajectory.yaml; task parameters are in perching_task.yaml.
The states are
published for smach_viewer (introspection server /perching_task).
"""

import os

import rospy
import rospkg
import smach
import smach_ros
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from std_srvs.srv import SetBool

from gimbalrotor.trajectory_follower import load_params, setup, take_off


TASK_KEYS = {
    "approach_window",
    "approach_speed_scale",
    "converge_pos",
    "converge_vel",
    "converge_joint",
    "converge_timeout",
    "ok_button",
    "ng_button",
    "switch_contacts",
    "after_hook",
    "unhook_wait",
}
TASK_CONFIG = os.path.join(
    rospkg.RosPack().get_path("gimbalrotor"), "config", "quad_with_dual_arm", "perching_task.yaml"
)


class Buttons:
    """The operator's OK and NG: the buttons of the joystick (a press after a release) or a message on
    the topics ~ok and ~ng (std_msgs/Empty, sent while waiting)."""

    def __init__(self, robot_ns, ok, ng):
        self.ok, self.ng = int(ok), int(ng)
        self.pressed = {self.ok: False, self.ng: False}
        self.sent = {"ok": False, "ng": False}
        rospy.Subscriber(robot_ns + "/joy", Joy, self._joy)
        rospy.Subscriber("~ok", Empty, lambda _: self._sent("ok"))
        rospy.Subscriber("~ng", Empty, lambda _: self._sent("ng"))
        self.topics = {k: rospy.resolve_name("~" + k) for k in ("ok", "ng")}

    def _joy(self, msg):
        for button in self.pressed:
            self.pressed[button] = button < len(msg.buttons) and msg.buttons[button] == 1

    def _sent(self, outcome):
        self.sent[outcome] = True

    def wait(self, prompt, hold=None):
        """'ok' or 'ng' on a fresh press of one of the buttons or a message on its topic ('preempted' on
        shutdown); hold(): called while waiting (e.g. sending the targets), returning False to give up
        ('aborted')."""
        rospy.loginfo(
            "\033[32m%s\n  (or: rostopic pub -1 %s std_msgs/Empty / %s)\033[0m",
            prompt,
            self.topics["ok"],
            self.topics["ng"],
        )
        released = {b: False for b in self.pressed}
        self.sent = {"ok": False, "ng": False}  # what came before the question does not answer it
        timer = rospy.Rate(20)
        while not rospy.is_shutdown():
            if hold is not None and not hold():
                return "aborted"
            for outcome in ("ok", "ng"):
                if self.sent[outcome]:
                    return outcome
            for button, outcome in ((self.ok, "ok"), (self.ng, "ng")):
                if not self.pressed[button]:
                    released[button] = True
                elif released[button]:
                    return outcome
            timer.sleep()
        return "preempted"


class Contacts:
    """The contacts of the whole-body controller, switched through its services."""

    def __init__(self, robot_ns, enabled):
        self.robot_ns, self.enabled = robot_ns, bool(enabled)
        self.active = set()

    def switch(self, name, on):
        if not self.enabled:
            return True
        service = "%s/whole_body/contacts/%s/activate" % (self.robot_ns, name)
        try:
            rospy.wait_for_service(service, timeout=1.0)
            response = rospy.ServiceProxy(service, SetBool)(on)
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn(
                "cannot switch the contact %s (%s): is the whole-body controller running "
                "(bringup whole_body:=true)?",
                name,
                e,
            )
            return False
        rospy.loginfo("contact %s", response.message)
        (self.active.add if on else self.active.discard)(name)
        return response.success

    def release_all(self):
        for name in sorted(self.active):
            self.switch(name, False)


class Task:
    """What the states share: the parameters, the targets, the player, the buttons, the contacts and
    the times."""

    def __init__(self):
        self.p, self.targets, self.player = setup()
        self.p.update(load_params(TASK_CONFIG, TASK_KEYS, "~task_config"))
        if self.p["after_hook"] not in ("trajectory", "stop", "unhook"):
            raise RuntimeError('after_hook is "trajectory", "stop" or "unhook", not %r' % self.p["after_hook"])
        self.robot = self.player.robot
        self.buttons = Buttons(self.robot.robot_ns, self.p["ok_button"], self.p["ng_button"])
        self.contacts = Contacts(self.robot.robot_ns, self.p["switch_contacts"])
        self.speed = float(self.p["speed"])
        if self.targets.first_contact >= len(self.targets):
            raise RuntimeError("the trajectory has no contact")
        self.contact_time = self.targets.time_of(self.targets.first_contact)
        self.window_start = max(self.contact_time - float(self.p["approach_window"]), 0.0)
        last = self.targets.last_frame(self.p["end_frame"])
        if last < self.targets.first_contact:
            raise RuntimeError("perching task end_frame must include the first contact")
        self.end_time = self.targets.time_of(last)
        # where the demo stops for the operator before the release (the end without the event)
        self.release_time = min(self.targets.events.get("release", self.end_time), self.end_time)
        rospy.loginfo(
            "contact at %.2f s, approach from %.2f s, end at %.2f s (file time)",
            self.contact_time,
            self.window_start,
            self.end_time,
        )


    def contact_ramp(self):
        """[s] the longest ramp_time of the active contacts (the whole-body controller's parameters)."""
        return max(
            [
                float(rospy.get_param("%s/controller/whole_body/contacts/%s/ramp_time" % (self.robot.robot_ns, n), 2.0))
                for n in self.contacts.active
            ]
            or [0.0]
        )


class TaskState(smach.State):
    def __init__(self, task, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        self.task = task


class Takeoff(TaskState):
    def __init__(self, task):
        TaskState.__init__(self, task, ["succeeded", "aborted"])

    def execute(self, userdata):
        return "succeeded" if take_off(self.task.p, self.task.player) else "aborted"


class GoToStart(TaskState):
    def __init__(self, task):
        TaskState.__init__(self, task, ["succeeded", "aborted"])

    def execute(self, userdata):
        p, player = self.task.p, self.task.player
        ok = player.move_to(0.0, float(p["approach_speed"]), float(p["joint_speed"])) and player.hold(
            0.0, float(p["settle_time"])
        )
        return "succeeded" if ok else "aborted"


class Flight(TaskState):
    def __init__(self, task):
        TaskState.__init__(self, task, ["succeeded", "aborted"])

    def execute(self, userdata):
        return "succeeded" if self.task.player.play(0.0, self.task.window_start, self.task.speed) else "aborted"


class PreHookCheck(TaskState):
    """Stop at the approach start after checking that the flight target has settled."""

    def __init__(self, task):
        TaskState.__init__(self, task, ["succeeded", "not_converged", "aborted"])

    def execute(self, userdata):
        t, p = self.task, self.task.p
        settled = t.player.converge(
            t.window_start,
            float(p["converge_pos"]),
            float(p["converge_vel"]),
            float(p["converge_joint"]),
            float(p["converge_timeout"]),
        )
        if not settled:
            return "aborted" if t.robot.getFlightState() != t.robot.HOVER_STATE else "not_converged"
        rospy.loginfo("pre-hook motion complete at %.2f s: holding before the hook approach", t.window_start)
        return "succeeded"


class Hook(TaskState):
    def __init__(self, task):
        TaskState.__init__(self, task, ["succeeded", "not_converged", "aborted"])

    def execute(self, userdata):
        t, p, player = self.task, self.task.p, self.task.player
        if not player.converge(
            t.window_start,
            float(p["converge_pos"]),
            float(p["converge_vel"]),
            float(p["converge_joint"]),
            float(p["converge_timeout"]),
        ):
            return "aborted" if t.robot.getFlightState() != t.robot.HOVER_STATE else "not_converged"
        ok = player.play(t.window_start, t.contact_time, t.speed * float(p["approach_speed_scale"]))
        return "succeeded" if ok else "aborted"


class JudgeHook(TaskState):
    def __init__(self, task):
        TaskState.__init__(self, task, ["hooked", "failed", "aborted", "preempted"])

    def execute(self, userdata):
        t = self.task
        hold = t.targets.still(t.contact_time)
        outcome = t.buttons.wait(
            "Hooked on the bar? OK (R1): yes, go on / NG (R2): no, teleoperate", hold=lambda: t.player.send(hold)
        )
        return {"ok": "hooked", "ng": "failed"}.get(outcome, outcome)


class Teleop(TaskState):
    def __init__(self, task):
        TaskState.__init__(self, task, ["retry", "finish", "aborted", "preempted"])

    def execute(self, userdata):
        t = self.task

        def hovering():
            return t.robot.getFlightState() == t.robot.HOVER_STATE

        outcome = t.buttons.wait(
            "Teleoperation: fly with the sticks (no target is sent; the joints stay). "
            "OK (R1): back to the approach and retry the hooking / NG (R2): finish here",
            hold=hovering,
        )
        return {"ok": "retry", "ng": "finish"}.get(outcome, outcome)


class Return(TaskState):
    def __init__(self, task):
        TaskState.__init__(self, task, ["succeeded", "aborted"])

    def execute(self, userdata):
        t, p = self.task, self.task.p
        t.contacts.release_all()
        ok = t.player.move_to(t.window_start, float(p["approach_speed"]), float(p["joint_speed"]))
        return "succeeded" if ok else "aborted"


class Release(TaskState):
    def __init__(self, task):
        TaskState.__init__(self, task, ["released", "kept", "aborted", "preempted"])

    def execute(self, userdata):
        t = self.task
        here = t.release_time
        last = t.targets.still(here)
        outcome = t.buttons.wait(
            "Done. OK (R1): switch the contacts off, the rotors taking the load back%s / NG (R2): "
            "finish with the contacts on" % (", then the hook out of the bar" if here < t.end_time else ""),
            hold=lambda: t.player.send(last),
        )
        if outcome != "ok":
            return {"ng": "kept"}.get(outcome, outcome)
        # the longest ramp of the contacts switched off (the parameters of the whole-body controller)
        names = sorted(t.contacts.active)
        ramp = t.contact_ramp()
        t.contacts.release_all()
        if not t.player.hold(here, ramp + 0.5):
            return "aborted"
        if here < t.end_time:  # the release, the unhook in the file
            rospy.loginfo("contacts %s off: playing the release and the unhook", names)
            if not t.player.play(here, t.end_time, t.speed):
                return "aborted"
            t.player.send(t.targets.still(t.end_time))
        else:
            rospy.loginfo("contacts %s off: holding while the load moves back onto the rotors", names)
        rospy.loginfo("released: away from the bar and land by teleoperation; the navigation holds the last target")
        return "released"


class Hooked(TaskState):
    """after_hook "unhook": the hook contact (the first contact of the file) switched on, the hooked pose
    held while the load moves onto the bar; the operator: OK (R1) -> UNHOOK; NG (R2): the task ends with
    the contact on, holding ("kept")."""

    def __init__(self, task):
        TaskState.__init__(self, task, ["unhook", "kept", "aborted", "preempted"])

    def execute(self, userdata):
        t = self.task
        for _, name, on in (e for e in t.targets.contact_events if e[0] == t.contact_time and e[2]):
            t.contacts.switch(name, on)
        hold = t.targets.still(t.contact_time)
        outcome = t.buttons.wait(
            "Hooked, contacts %s on. OK (R1): unhook (contacts off, the hook out of the bar) / NG (R2): "
            "finish here with the contacts on" % sorted(t.contacts.active),
            hold=lambda: t.player.send(hold),
        )
        return {"ok": "unhook", "ng": "kept"}.get(outcome, outcome)


class Unhook(TaskState):
    """Every contact switched off and held through its ramp (the load back onto the rotors), then the
    approach played backwards at ~approach_speed_scale: the hook out through its opening, to the start of
    the approach, held there. The task ends ("unhooked"): the rest by teleoperation."""

    def __init__(self, task):
        TaskState.__init__(self, task, ["succeeded", "aborted"])

    def execute(self, userdata):
        t, p = self.task, self.task.p
        ramp = t.contact_ramp()
        t.contacts.release_all()
        if not t.player.hold(t.contact_time, ramp + float(p["unhook_wait"])):
            return "aborted"
        if not t.player.play_back(t.contact_time, t.window_start, t.speed * float(p["approach_speed_scale"])):
            return "aborted"
        t.player.send(t.targets.still(t.window_start))
        rospy.loginfo("unhooked: the rest by teleoperation; the navigation holds the start of the approach")
        return "succeeded"


class AfterHook(TaskState):
    def __init__(self, task):
        TaskState.__init__(self, task, ["succeeded", "stopped", "aborted"])

    def execute(self, userdata):
        # the trajectory from the contact, cut where contacts begin or end, switched there
        t = self.task
        events = [e for e in t.targets.contact_events if t.contact_time - 1e-9 <= e[0] <= t.release_time + 1e-9]
        if t.p["after_hook"] == "stop":
            for _, name, on in (e for e in events if e[0] == t.contact_time and e[2]):
                t.contacts.switch(name, on)
            if not t.player.send(t.targets.still(t.contact_time)):
                return "aborted"
            rospy.loginfo("hooked: stopped there (after_hook stop), the hooked pose held by the navigation")
            return "stopped"
        times = sorted({e[0] for e in events})
        cursor = t.contact_time
        for time in times:
            if time > cursor and not t.player.play(cursor, time, t.speed):
                return "aborted"
            cursor = max(cursor, time)
            for _, name, on in (e for e in events if e[0] == time):
                t.contacts.switch(name, on)
        if not t.player.play(cursor, t.release_time, t.speed):
            return "aborted"
        t.player.send(t.targets.still(t.release_time))
        rospy.loginfo("done: holding the frame before the release")
        return "succeeded"


def build(task, stop_before_hook=False):
    outcomes = ["succeeded", "released", "stopped", "unhooked", "finished", "aborted", "preempted"]
    unhook = task.p["after_hook"] == "unhook"
    if stop_before_hook:
        outcomes.append("not_converged")
    sm = smach.StateMachine(outcomes=outcomes)
    with sm:
        smach.StateMachine.add("TAKEOFF", Takeoff(task), transitions={"succeeded": "GO_TO_START", "aborted": "aborted"})
        smach.StateMachine.add(
            "GO_TO_START", GoToStart(task), transitions={"succeeded": "FLIGHT", "aborted": "aborted"}
        )
        smach.StateMachine.add(
            "FLIGHT",
            Flight(task),
            transitions={"succeeded": "PRE_HOOK_CHECK" if stop_before_hook else "HOOK", "aborted": "aborted"},
        )
        if stop_before_hook:
            smach.StateMachine.add(
                "PRE_HOOK_CHECK",
                PreHookCheck(task),
                transitions={"succeeded": "succeeded", "not_converged": "not_converged", "aborted": "aborted"},
            )
        smach.StateMachine.add(
            "HOOK", Hook(task), transitions={"succeeded": "JUDGE_HOOK", "not_converged": "TELEOP", "aborted": "aborted"}
        )
        smach.StateMachine.add(
            "JUDGE_HOOK",
            JudgeHook(task),
            transitions={
                "hooked": "HOOKED" if unhook else "AFTER_HOOK",
                "failed": "TELEOP",
                "aborted": "aborted",
                "preempted": "preempted",
            },
        )
        if unhook:
            smach.StateMachine.add(
                "HOOKED",
                Hooked(task),
                transitions={"unhook": "UNHOOK", "kept": "succeeded", "aborted": "aborted", "preempted": "preempted"},
            )
            smach.StateMachine.add("UNHOOK", Unhook(task), transitions={"succeeded": "unhooked", "aborted": "aborted"})
        smach.StateMachine.add(
            "TELEOP",
            Teleop(task),
            transitions={"retry": "RETURN", "finish": "finished", "aborted": "aborted", "preempted": "preempted"},
        )
        smach.StateMachine.add("RETURN", Return(task), transitions={"succeeded": "HOOK", "aborted": "aborted"})
        smach.StateMachine.add(
            "AFTER_HOOK",
            AfterHook(task),
            transitions={"succeeded": "RELEASE", "stopped": "stopped", "aborted": "aborted"},
        )
        smach.StateMachine.add(
            "RELEASE",
            Release(task),
            transitions={"released": "released", "kept": "succeeded", "aborted": "aborted", "preempted": "preempted"},
        )
    return sm


def main():
    rospy.init_node("perching_task")
    task = Task()
    sm = build(task)
    viewer = smach_ros.IntrospectionServer("perching_task", sm, "/PERCHING_TASK")
    viewer.start()
    outcome = sm.execute()
    rospy.loginfo("perching task: %s", outcome)
    viewer.stop()


if __name__ == "__main__":
    main()

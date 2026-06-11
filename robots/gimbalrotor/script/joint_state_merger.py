#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState

class SpecificJointMerger:
    def __init__(self):
        rospy.init_node('specific_joint_merger', anonymous=True)

        # --- 設定: ユーザーのログに基づいたトピック名 ---
        self.arm_topic = '/arm/joint_states_arm'
        self.gimbal_topic = '/gimbalrotor/joint_states_gimbal'
        self.output_topic = '/gimbalrotor/joint_states'

        # 配信レート (Hz)
        self.rate = 30.0

        # --- データを保持する辞書 ---
        # { 'joint_name': {'pos': float, 'vel': float, 'eff': float} }
        self.merged_data = {}

        # --- Publisher ---
        self.pub = rospy.Publisher(self.output_topic, JointState, queue_size=10)

        # --- Subscribers ---
        rospy.Subscriber(self.arm_topic, JointState, self.callback)
        rospy.Subscriber(self.gimbal_topic, JointState, self.callback)

        rospy.loginfo("Joint Merger Started")
        rospy.loginfo("Input 1: %s", self.arm_topic)
        rospy.loginfo("Input 2: %s", self.gimbal_topic)
        rospy.loginfo("Output : %s", self.output_topic)

    def callback(self, msg):
        """
        受信したJointStateメッセージを辞書に格納する。
        ログにあった「velocity: []」のような空配列に対応。
        """
        for i, name in enumerate(msg.name):
            # 位置 (Position)
            # 配列の長さを確認してから取得
            pos = msg.position[i] if i < len(msg.position) else 0.0

            # 速度 (Velocity)
            # ログでは [] になっていたため、データがない場合は 0.0 とする
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0

            # 力/トルク (Effort)
            eff = msg.effort[i] if i < len(msg.effort) else 0.0

            # 辞書を更新 (以前の値を上書き)
            self.merged_data[name] = {
                'pos': pos,
                'vel': vel,
                'eff': eff
            }

    def loop(self):
        r = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            if not self.merged_data:
                # まだデータが来ていなければ待機
                r.sleep()
                continue

            # 出力メッセージの作成
            out_msg = JointState()
            out_msg.header.stamp = rospy.Time.now() # 常に最新時刻

            # 辞書の中身を全てリストに展開
            for name, data in self.merged_data.items():
                out_msg.name.append(name)
                out_msg.position.append(data['pos'])
                out_msg.velocity.append(data['vel'])
                out_msg.effort.append(data['eff'])

            self.pub.publish(out_msg)
            r.sleep()

if __name__ == '__main__':
    try:
        node = SpecificJointMerger()
        node.loop()
    except rospy.ROSInterruptException:
        pass
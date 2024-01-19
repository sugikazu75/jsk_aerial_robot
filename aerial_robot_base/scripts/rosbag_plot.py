#!/usr/bin/env python

import rosbag
import numpy as np
import sys
import os
import matplotlib.pyplot as plt
import yaml

class bagPlotter:
    def __init__(self, obj):
        self.bag_filename = os.path.join(os.getcwd(), obj["bag_filename"])
        self.bag = rosbag.Bag(self.bag_filename)
        self.robot_ns = obj["robot_ns"]
        self.plot_topic_names = obj["plot_topic_names"]
        for i in range(len(self.plot_topic_names)):
            self.plot_topic_names[i] = "/" + self.robot_ns + "/" + self.plot_topic_names[i]

        self.plot_fields = obj["plot_fields"]
        self.plot_colors = obj["plot_colors"]
        self.start_time = obj["start_time"]
        self.end_time = obj["end_time"]
        self.show_legend = obj["show_legend"]
        self.legends = []
        if(self.show_legend):
            self.legends = obj["legends"]
        self.x_label = obj["x_label"]
        self.y_label = obj["y_label"]
        self.y_range = obj["y_range"]

        print("bag_filename: ", self.bag_filename)
        print("robot_ns: ", self.robot_ns)
        print("plot_topic_names", self.plot_topic_names)
        print("plot_fields: ", self.plot_fields)
        print("start_time: ", self.start_time)
        print("end_time: ", self.end_time)

        # get initial time in rosbag
        self.zero_time = 0
        for i, (topic, msg, t) in enumerate(self.bag.read_messages("/" + self.robot_ns + "/flight_state")):
            if(i == 0):
                self.zero_time = t.secs + t.nsecs / 1000000000.0
        print("zero time: ", self.zero_time)

        # get data from rosbag
        self.datas = [np.empty(0)] * len(self.plot_topic_names)
        self.times = [np.empty(0)] * len(self.plot_topic_names)

        for i, (topic, msg, t) in enumerate(self.bag.read_messages(topics = self.plot_topic_names)):
            for j in range(len(self.plot_topic_names)):
                if(self.plot_topic_names[j] == topic):
                    plot_field = self.plot_fields[j]
                    plot_field_attribs = plot_field.split("/")
                    data = msg
                    for plot_field_attrib in plot_field_attribs:
                        data = getattr(data, plot_field_attrib)

                    rosbag_time = t.secs + t.nsecs / 1000000000.0 - self.zero_time
                    if(self.start_time <= rosbag_time and rosbag_time <= self.end_time):
                        self.datas[j] = np.append(self.datas[j], data)
                        self.times[j] = np.append(self.times[j], rosbag_time - self.start_time)

        # plot data
        plt.rcParams['font.family'] = 'Times New Roman' # https://kenbo.hatenablog.com/entry/2018/11/28/111639
        plt.rcParams['figure.subplot.bottom'] = 0.2     # https://qiita.com/78910jqk/items/e8bce993081c384afdba
        fig = plt.figure(figsize=obj["fig_size"])
        for i in range(len(self.plot_topic_names)):
            plt.plot(self.times[i], self.datas[i], color=self.plot_colors[i], label=self.legends[i])
        if(self.show_legend):
            plt.legend(frameon=False, fontsize=obj["legend_fontsize"])
        plt.xlabel(self.x_label, fontsize=obj["x_label_fontsize"])
        plt.ylabel(self.y_label, fontsize=obj["y_label_fontsize"])
        plt.ylim(self.y_range)
        plt.tick_params(labelsize=obj["label_fontsize"])
        if(obj["save_fig"]):
            plt.savefig(obj["save_fig_name"])
        else:
            plt.show()


if __name__ == "__main__":
    config_path = ""
    if(len(sys.argv) == 2):
        config_path = sys.argv[1]
    else:
        print("Please specify the config file name!")
        sys.exit()

    with open(config_path) as file:
        obj = yaml.safe_load(file)

        bagPlotter(obj)

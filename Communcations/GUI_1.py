# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'OneDrive - Linköpings universitet/Programmering/TSEA56/Designer_gui_1.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.

import sys
import paho.mqtt.client as mqtt
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtGui import QIcon, QPixmap, QPainter, QPen
from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsTextItem
from PyQt5.QtCore import QPointF, Qt
import time
import multiprocessing
import keyboard
import matplotlib.pyplot as plt


class Ui_Dialog(object):
    # Class for the GUI

    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(986, 672)

        self.graphicsView = QGraphicsView(Dialog)
        self.graphicsView.setGeometry(QtCore.QRect(550, 390, 392, 280))
        self.graphicsView.setObjectName("graphicsView")
        self.graphicsView.setScene(QGraphicsScene())
        self.graphicsView.setRenderHint(QPainter.Antialiasing)
        # starts time
        self.update_time_bool = False
        self.current_time = "0"
        self.start = time.time()
        self.type_of_mode = ""
        self.qData = multiprocessing.Queue()

        # bool for starting car
        self.is_driving = False
        self.distance_to_obj = "0"
        self.car_distance_driven = 0
        self.car_speed_data = 0
        self.delta_t1_speed = 0
        self.delta_t2_speed = 0
        # initate straight ahead and zero speed
        self.speed = 0
        self.steering = 50
        self.breaking = 0
        self.obs_det_bool = False
        self.crs_data = "A to B"
        self.lat_pos_data = 0
        self.route_plan_data = "A to B to D to F"
        self.map_node_dict = {"A": [799, 440], "B": [570, 500], "C": [720, 517], "D": [
            570, 573], "E": [720, 555], "F": [638, 642], "G": [807, 642], "H": [862, 502]}
        self.previous_rs = "A"
        self.nodes = list()
        self.setup_nodes(self.map_node_dict)
        # Log data lists
        self.save_car_speed_data = []
        self.save_distance_to_obj = []
        self.save_car_distance_driven = []
        self.save_speed = []
        self.save_steering = []
        self.save_car_breaking = []
        self.save_obs_det_bool = []
        self.save_lat_pos_data = []

        ################################## GUI LABEL AND BUTTONS ################################
        self.obst_det_head = QtWidgets.QLabel(Dialog)
        self.obst_det_head.setGeometry(QtCore.QRect(140, 20, 131, 16))
        self.obst_det_head.setObjectName("obst_det_head")

        self.obst_det_label = QtWidgets.QLabel(Dialog)
        self.obst_det_label.setGeometry(QtCore.QRect(30, 60, 131, 16))
        self.obst_det_label.setObjectName("obst_det_label")

        self.obst_dist_label = QtWidgets.QLabel(Dialog)
        self.obst_dist_label.setGeometry(QtCore.QRect(210, 60, 131, 16))
        self.obst_dist_label.setObjectName("obst_dist_label")

        self.obs_det_display = QtWidgets.QTextBrowser(Dialog)
        self.obs_det_display.setGeometry(QtCore.QRect(30, 90, 151, 31))
        self.obs_det_display.setObjectName("obs_det_display")
        self.obs_det_display.setText(str(self.obs_det_bool))

        self.obs_dist_display = QtWidgets.QTextBrowser(Dialog)
        self.obs_dist_display.setGeometry(QtCore.QRect(210, 90, 151, 31))
        self.obs_dist_display.setObjectName("obs_dist_display")
        self.obs_dist_display.setText(str(self.distance_to_obj))

        self.drive_mode_head = QtWidgets.QLabel(Dialog)
        self.drive_mode_head.setGeometry(QtCore.QRect(20, 180, 91, 16))
        self.drive_mode_head.setObjectName("drive_mode_head")

        # manual mode
        self.manual_mode = QtWidgets.QPushButton(Dialog)
        self.manual_mode.setGeometry(QtCore.QRect(20, 220, 161, 26))
        self.manual_mode.setObjectName("manual_mode")

        # semi-automatic
        self.semi_auto_mode = QtWidgets.QPushButton(Dialog)
        self.semi_auto_mode.setGeometry(QtCore.QRect(20, 270, 161, 31))
        self.semi_auto_mode.setObjectName("semi_auto_mode")

        # automatic
        self.auto_mode = QtWidgets.QPushButton(Dialog)
        self.auto_mode.setGeometry(QtCore.QRect(20, 320, 161, 31))
        self.auto_mode.setObjectName("auto_mode")

        # start button to initate car
        self.start_car = QtWidgets.QPushButton(Dialog)
        self.start_car.setGeometry(QtCore.QRect(240, 230, 141, 51))
        self.start_car.setStyleSheet("background-color: green")
        self.start_car.setObjectName("start_car")

        # stop button
        self.stop_car = QtWidgets.QPushButton(Dialog)
        self.stop_car.setGeometry(QtCore.QRect(240, 300, 141, 51))
        self.stop_car.setStyleSheet("background-color: red")
        self.stop_car.setObjectName("stop_car")

        # SEMI AUTOMATIC COMMANDS
        self.next_stop_label = QtWidgets.QPushButton(Dialog)
        self.next_stop_label.setGeometry(QtCore.QRect(60, 600, 80, 40))
        self.next_stop_label.setObjectName("next_stop_label")

        self.next_node_label = QtWidgets.QPushButton(Dialog)
        self.next_node_label.setGeometry(QtCore.QRect(160, 600, 80, 40))
        self.next_node_label.setObjectName("next_node_label")

        self.keep_right_label = QtWidgets.QPushButton(Dialog)
        self.keep_right_label.setGeometry(QtCore.QRect(160, 550, 80, 40))
        self.keep_right_label.setObjectName("keep_right_label")

        self.keep_left_label = QtWidgets.QPushButton(Dialog)
        self.keep_left_label.setGeometry(QtCore.QRect(60, 550, 80, 40))
        self.keep_left_label.setObjectName("keep_left_label")
        ###

        # textbox which sends commands
        self.send_command = QtWidgets.QPushButton(Dialog)
        self.send_command.setGeometry(QtCore.QRect(280, 425, 120, 30))
        self.send_command.setObjectName("send_command")

        self.semi_control_label = QtWidgets.QLabel(Dialog)
        self.semi_control_label.setGeometry(QtCore.QRect(80, 510, 180, 16))
        self.semi_control_label.setObjectName("semi_control_label")

        self.Command_input_box = QtWidgets.QTextEdit(Dialog)
        self.Command_input_box.setGeometry(QtCore.QRect(30, 410, 241, 64))
        self.Command_input_box.setObjectName("Command_input_box")

        self.command_input_label = QtWidgets.QLabel(Dialog)
        self.command_input_label.setGeometry(QtCore.QRect(100, 380, 131, 16))
        self.command_input_label.setObjectName("command_input_label")

        self.drive_info_label = QtWidgets.QLabel(Dialog)
        self.drive_info_label.setGeometry(QtCore.QRect(650, 20, 81, 16))
        self.drive_info_label.setObjectName("drive_info_label")

        self.time_label = QtWidgets.QLabel(Dialog)
        self.time_label.setGeometry(QtCore.QRect(490, 50, 151, 16))
        self.time_label.setObjectName("time_label")

        self.distance_label = QtWidgets.QLabel(Dialog)
        self.distance_label.setGeometry(QtCore.QRect(490, 90, 151, 16))
        self.distance_label.setObjectName("distance_label")

        self.speed_label = QtWidgets.QLabel(Dialog)
        self.speed_label.setGeometry(QtCore.QRect(490, 130, 151, 16))
        self.speed_label.setObjectName("speed_label")

        self.throttle_label = QtWidgets.QLabel(Dialog)
        self.throttle_label.setGeometry(QtCore.QRect(490, 170, 151, 16))
        self.throttle_label.setObjectName("throttle_label")

        self.bearing_label = QtWidgets.QLabel(Dialog)
        self.bearing_label.setGeometry(QtCore.QRect(490, 210, 151, 16))
        self.bearing_label.setObjectName("bearing_label")

        self.crs_label = QtWidgets.QLabel(Dialog)
        self.crs_label.setGeometry(QtCore.QRect(490, 250, 151, 16))
        self.crs_label.setObjectName("crs_label")

        self.lat_pos_label = QtWidgets.QLabel(Dialog)
        self.lat_pos_label.setGeometry(QtCore.QRect(490, 290, 151, 16))
        self.lat_pos_label.setObjectName("lat_pos_label")

        self.routeplan_label = QtWidgets.QLabel(Dialog)
        self.routeplan_label.setGeometry(QtCore.QRect(490, 330, 151, 16))
        self.routeplan_label.setObjectName("routeplan_label")

        self.map_label = QtWidgets.QLabel(Dialog)
        self.map_label.setGeometry(QtCore.QRect(490, 370, 151, 16))
        self.map_label.setObjectName("map_label")

        self.image_label = QtWidgets.QLabel(Dialog)
        self.image_label.setGeometry(QtCore.QRect(550, 390, 151, 16))
        self.image_label.setObjectName("image_label")
        self.banan_map = QPixmap('banspecifikation.png')
        self.image_label.setPixmap(self.banan_map)
        self.image_label.setScaledContents(True)
        self.banan_map = self.banan_map.scaled(
            392, 280, QtCore.Qt.KeepAspectRatio, QtCore.Qt.FastTransformation)
        self.image_label.resize(self.banan_map.width(),
                                self.banan_map.height())

        self.car_pos_label = QtWidgets.QLabel(Dialog)
        self.car_pos_label.setGeometry(QtCore.QRect(100, 100, 20, 20))
        self.car_pos_label.setObjectName("car_pos_label")
        self.car_pos_label.resize(10, 10)
        self.car_pos_label.move(
            self.map_node_dict["A"][0], self.map_node_dict["A"][1])
        self.car_pos_label.setStyleSheet(
            "border: 5px solid blue; border-radius: 5px;")

        self.time_display = QtWidgets.QTextBrowser(Dialog)
        self.time_display.setGeometry(QtCore.QRect(670, 50, 301, 31))
        self.time_display.setObjectName("time_display")

        self.drive_distance_display = QtWidgets.QTextBrowser(Dialog)
        self.drive_distance_display.setGeometry(QtCore.QRect(670, 90, 301, 31))
        self.drive_distance_display.setObjectName("drive_distance_display")
        self.drive_distance_display.setText(str(int(self.car_distance_driven)))

        self.speed_display = QtWidgets.QTextBrowser(Dialog)
        self.speed_display.setGeometry(QtCore.QRect(670, 130, 301, 31))
        self.speed_display.setObjectName("speed_display")
        self.speed_display.setText(str(self.car_speed_data))

        self.throttle_display = QtWidgets.QTextBrowser(Dialog)
        self.throttle_display.setGeometry(QtCore.QRect(670, 170, 301, 31))
        self.throttle_display.setObjectName("throttle_display")
        self.throttle_display.setText(str(self.speed))

        self.bearing_display = QtWidgets.QTextBrowser(Dialog)
        self.bearing_display.setGeometry(QtCore.QRect(670, 210, 301, 31))
        self.bearing_display.setObjectName("bearing_display")
        self.bearing_display.setText(str(self.steering))

        self.crs_display = QtWidgets.QTextBrowser(Dialog)
        self.crs_display.setGeometry(QtCore.QRect(670, 250, 301, 31))
        self.crs_display.setObjectName("crs_display")
        self.crs_display.setText(str(self.crs_data))

        self.lateral_pos_display = QtWidgets.QTextBrowser(Dialog)
        self.lateral_pos_display.setGeometry(QtCore.QRect(670, 290, 301, 31))
        self.lateral_pos_display.setObjectName("lateral_pos_display")
        self.lateral_pos_display.setText(str(self.lat_pos_data))

        self.routeplan_display = QtWidgets.QTextBrowser(Dialog)
        self.routeplan_display.setGeometry(QtCore.QRect(670, 330, 301, 31))
        self.routeplan_display.setObjectName("routeplan_display")
        self.routeplan_display.setText(str(self.route_plan_data))

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

        self.connect_buttons()
        ################################## END GUI LABEL AND BUTTONS ################################

        ################################## MQTT AND TIMER INITS ################################
        self.mqtt_init()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_time)
        self.timer.start(70)  # update every 1000 milliseconds (1 second)

        self.messagetimer = QtCore.QTimer()
        self.messagetimer.timeout.connect(self.updatedata)
        self.messagetimer.start(100)

        self.drivingtimer = QtCore.QTimer()
        self.drivingtimer.timeout.connect(self.drive_function)
        self.drivingtimer.start(50)

        self.pingtimer = QtCore.QTimer()
        self.pingtimer.timeout.connect(self.ping_raspberry)
        self.pingtimer.start(2000)

        self.log_data_timer = QtCore.QTimer()
        self.log_data_timer.timeout.connect(self.log_data)
        self.log_data_timer.start(1000)  # 1000 = every sec

    def ping_raspberry(self):
        # print("PING")
        self.mqtt_client.publish("ping", "1")

    def setup_nodes(self, node_dictionary):
        for node, position in node_dictionary.items():
            node_item = QGraphicsEllipseItem(-5, -5, 10, 10)
            node_item.setBrush(Qt.red)
            node_item.setPos(position[0]-550, position[1]-390)
            # self.scene().addItem(node_item)
            label_item = QGraphicsTextItem(str(node))
            label_item.setPos(
                QPointF(position[0] + 10 - 550, position[1]-390-12))
            self.nodes.append([node_item, label_item])

        # Add the label to the QGraphicsScene
        for i in self.nodes:
            self.graphicsView.scene().addItem(i[0])
            self.graphicsView.scene().addItem(i[1])
        print(f"Node: {node}, Pos: {position}")

    def drive_function(self):
        hotkey = keyboard.get_hotkey_name()
        hotkey_functions = {
            "uppil": self.set_speed_up,
            "up": self.set_speed_up,
            "nedpil": self.set_speed_down,
            "down": self.set_speed_down,
            "högerpil": self.set_steering_right,
            "right": self.set_steering_right,
            "vänsterpil": self.set_steering_left,
            "left": self.set_steering_left,
            "B": self.set_breaking_on,
            "b": self.set_breaking_on,
            "G": self.set_breaking_off,
            "g": self.set_breaking_off,
            "space": self.set_speed_and_steering_zero
        }
        function = hotkey_functions.get(hotkey)
        if function:
            function()

    def set_speed_up(self):
        if self.type_of_mode == "Manual" and self.is_driving and self.speed <= 251:
            self.speed += 4
            self.throttle_display.setText(str(self.speed))
            self.mqtt_client.publish("speed", self.speed)

    def set_speed_down(self):
        if self.type_of_mode == "Manual" and self.is_driving and self.speed > 4:
            self.speed -= 4
            self.throttle_display.setText(str(self.speed))
            self.mqtt_client.publish("speed", self.speed)

    def set_steering_right(self):
        if self.type_of_mode == "Manual" and self.is_driving and self.steering <= 120:
            self.steering += 4
            self.bearing_display.setText(str(self.steering))
            self.mqtt_client.publish("steering", self.steering)

    def set_steering_left(self):
        if self.type_of_mode == "Manual" and self.is_driving and self.steering >= 4:
            self.steering -= 4
            self.bearing_display.setText(str(self.steering))
            self.mqtt_client.publish("steering", self.steering)

    def set_breaking_on(self):
        print(self.is_driving)
        print(self.type_of_mode)
        print(self.breaking)
        if self.type_of_mode == "Manual" and self.is_driving and self.breaking == 0:
            self.breaking = 1
            print("Breaking")
            # self.speed = 0
            self.mqtt_client.publish("breaking", self.breaking)
            # self.mqtt_client.publish("speed", self.speed)
            self.throttle_display.setText(str(self.speed))
            self.bearing_display.setText(str(self.steering))

    def set_breaking_off(self):
        if self.type_of_mode == "Manual" and self.is_driving and self.breaking == 1:
            self.breaking = 0
            # self.speed = 0
            self.mqtt_client.publish("breaking", self.breaking)
            self.mqtt_client.publish("speed", self.speed)
            self.throttle_display.setText(str(self.speed))
            self.bearing_display.setText(str(self.steering))

    def set_speed_and_steering_zero(self):
        if self.type_of_mode == "Manual" and self.is_driving:
            self.speed = 0
            self.steering = 50
            self.throttle_display.setText(str(self.speed))
            self.bearing_display.setText(str(self.steering))
            self.mqtt_client.publish("speed", self.speed)
            self.mqtt_client.publish("steering", self.steering)

    def update_time(self):
        # get the current time and format it as a string
        if self.update_time_bool:
            self.current_time = (str(round(time.time() - self.start, 2)))
        # set the text of the time_label widget to the current time
        self.time_display.setText(self.current_time)

    def log_data(self):
        if self.update_time_bool:
            self.save_car_speed_data.append(
                [self.car_speed_data, self.current_time])
            self.save_distance_to_obj.append(
                [self.distance_to_obj, self.current_time])
            self.save_car_distance_driven.append(
                [self.car_distance_driven, self.current_time])
            self.save_speed.append([self.speed, self.current_time])
            self.save_steering.append([self.steering, self.current_time])
            self.save_car_breaking.append([self.breaking, self.current_time])
            self.save_obs_det_bool.append(
                [self.obs_det_bool, self.current_time])
            self.save_lat_pos_data.append(
                [self.lat_pos_data, self.current_time])

            # print(self.save_car_speed_data)

    def plot_data(self, data_list):
        i1 = 0
        x_values = []  # time
        y_values = []  # data
        for i1 in data_list:
            x_values.append(i1[1])
            y_values.append(i1[0])

        plt.plot(x_values, y_values)
        plt.gcf().autofmt_xdate()
        plt.show()
        print("plotted")

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate(
            "Dialog", "Very fast taxicar, vroom vroom"))
        self.obst_det_head.setText(_translate("Dialog", "Obstacle detection:"))
        self.obst_det_label.setText(_translate("Dialog", "Obstacle detected"))
        self.obst_dist_label.setText(
            _translate("Dialog", "Distance to obstacle"))
        self.drive_mode_head.setText(_translate("Dialog", "Drive mode:"))
        self.manual_mode.setText(_translate("Dialog", "Manual mode"))
        self.semi_auto_mode.setText(
            _translate("Dialog", "Semi-automatic mode"))
        self.auto_mode.setText(_translate("Dialog", "Automatic mode"))
        self.start_car.setText(_translate("Dialog", "START"))
        self.stop_car.setText(_translate("Dialog", "STOP"))
        self.next_stop_label.setText(_translate("Dialog", "NEXT \nSTOP"))
        self.next_node_label.setText(_translate("Dialog", "NEXT \nNODE"))
        self.keep_right_label.setText(_translate("Dialog", "Right"))
        self.keep_left_label.setText(_translate("Dialog", "Left"))
        self.send_command.setText(_translate("Dialog", "Send command"))

        self.semi_control_label.setText(
            _translate("Dialog", "Semi-automatic Controls"))
        self.command_input_label.setText(_translate("Dialog", "Command input"))
        self.drive_info_label.setText(_translate("Dialog", "Drive info"))
        self.time_label.setText(_translate("Dialog", "Time:"))
        self.distance_label.setText(_translate("Dialog", "Distance (cm):"))
        self.speed_label.setText(_translate("Dialog", "Speed (cm/s):"))
        self.throttle_label.setText(_translate("Dialog", "Throttle:"))
        self.bearing_label.setText(_translate("Dialog", "Bearing:"))
        self.crs_label.setText(_translate("Dialog", "Current road segment:"))
        self.lat_pos_label.setText(_translate("Dialog", "Lateral position:"))
        self.routeplan_label.setText(_translate("Dialog", "Planned route:"))
        self.map_label.setText(_translate("Dialog", "Map:"))

    def connect_buttons(self):
        # hej
        self.manual_mode.clicked.connect(self.on_manual_mode_click)
        self.semi_auto_mode.clicked.connect(self.on_semi_auto_mode_click)
        self.auto_mode.clicked.connect(self.on_auto_mode_click)
        self.send_command.clicked.connect(self.on_send_command)

        # start and stop
        self.start_car.clicked.connect(self.on_start_car_click)
        self.stop_car.clicked.connect(self.on_stop_car_click)

        # Semi-auto controls
        self.next_stop_label.clicked.connect(self.on_next_stop_click)
        self.next_node_label.clicked.connect(self.on_next_node_click)
        self.keep_right_label.clicked.connect(self.on_keep_right_click)
        self.keep_left_label.clicked.connect(self.on_keep_left_click)

    def on_manual_mode_click(self):
        # This method is called when manual_mode is clicked
        if not self.is_driving:
            self.mqtt_client.publish("mode", 1)
            self.type_of_mode = "Manual"
            print(self.type_of_mode)

    def on_semi_auto_mode_click(self):
        # This method is called when semi_auto_mode is clicked
        if not self.is_driving:
            self.mqtt_client.publish("mode", 2)
            self.type_of_mode = "Semi-automatic"
            print(self.type_of_mode)

    def on_auto_mode_click(self):
        # This method is called when auto_mode is clicked

        if not self.is_driving:
            self.mqtt_client.publish("mode", 3)
            self.type_of_mode = "Automatic"
            print(self.type_of_mode)

    def on_send_command(self):
        # This method is called when send command is clicked
        self.mqtt_client.publish("data/command")
        try:
            stringlist = (self.Command_input_box.toPlainText()).rsplit(": ")
            print("Topic: " + stringlist[0])
            print("Command: " + stringlist[1])
        except IndexError:
            print("Error: Wrong input")

    def on_start_car_click(self):
        # starts car
        self.update_time_bool = 1
        # timer counter
        self.start = time.time()
        # starts first measured time for calculating distance
        self.delta_t1_speed = time.time()
        self.is_breaking = 0
        self.breaking = 0
        self.is_driving = True
        print("START")

    def on_stop_car_click(self):
        # stops car
        self.current_time = "0"
        self.update_time_bool = 0
        self.is_driving = False

        self.speed = 0  # reset speed
        self.steering = 50  # reset wheels
        self.is_breaking = 0
        self.throttle_display.setText(str(self.speed))
        self.bearing_display.setText(str(self.steering))

        self.mqtt_client.publish("stop", "1")
        self.mqtt_client.publish("speed", self.speed)
        self.mqtt_client.publish("steering", self.steering)
        self.plot_data(self.save_car_speed_data)
        print("STOP")

    def on_next_stop_click(self):
        print("Next stop")

    def on_next_node_click(self):
        print("Next node")

    def on_keep_right_click(self):
        self.mqtt_client.publish("command/turning", "4")
        print("Keep right")

    def on_keep_left_click(self):
        self.mqtt_client.publish("command/turning", "3")
        print("Keep left")

    def mqtt_init(self):
        # initate connection
        MQTT_TOPIC = [("data/distance", 0), ("data/speed", 0),
                      ("data/crs", 0), ("data/lat_pos", 0), ("data/route_plan", 0), ("data/obstacle", 0)]
        try:
            broker_ip = "10.241.226.165"
            broker_port = 1883
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.username_pw_set("tsea56G09", "mindset")
            self.mqtt_client.connect(broker_ip, broker_port)
            self.mqtt_client.subscribe(MQTT_TOPIC)
            self.mqtt_client.loop_start()
            self.mqtt_client.on_message = self.on_message
        except:
            print("Failed to setup connection")

    def on_message(self, client, userdata, message):
        m = str(message.payload.decode("utf-8"))
        t = message.topic
        self.qData.put((t, m))

    def updatedata(self):
        if not self.qData.empty():
            message = self.qData.get()
            if message[0] == "data/distance":
                self.distance_to_obj = message[1]
                self.obs_dist_display.setText(str(self.distance_to_obj))

            if message[0] == "data/speed":
                # recieves speed and shows it on the gui
                self.car_speed_data = float(message[1])
                self.speed_display.setText(str(self.car_speed_data))
                # checks if car is driving and then prints distance based on a difference in time
                if self.is_driving:
                    self.delta_t2_speed = time.time()
                    delta_t = self.delta_t2_speed - self.delta_t1_speed
                    self.delta_t1_speed = self.delta_t2_speed
                    self.car_distance_driven += self.car_speed_data*delta_t
                    self.drive_distance_display.setText(
                        str(int(self.car_distance_driven)))

            ################### Throttle and Bearing data given from gui #####################

            if message[0] == "data/crs":
                print("crs recieved")
                self.crs_data = str(message[1])
                # print(self.crs_data)
                self.crs_display.setText(
                    str(self.previous_rs + " -> " + self.crs_data))
                self.previous_rs = self.crs_data
                ########################################### SENSOR GÄNGET MÅSTE SÄGA HUR DOM SKA SKICKA DATAT ############
                ########################################### SENSOR GÄNGET MÅSTE SÄGA HUR DOM SKA SKICKA DATAT ############
                self.car_pos_label.move(
                    self.map_node_dict[self.crs_data][0], self.map_node_dict[self.crs_data][1])
                ########################################### SENSOR GÄNGET MÅSTE SÄGA HUR DOM SKA SKICKA DATAT ############
                ########################################### SENSOR GÄNGET MÅSTE SÄGA HUR DOM SKA SKICKA DATAT ############

            if message[0] == "data/lat_pos":
                self.lat_pos_data = message[1]
                self.lateral_pos_display.setText(str(self.lat_pos_data))

            if message[0] == "data/route_plan":
                print("route_plan recieved")
                # self.route_plan_data = message[1]
                # print(self.route_plan_data)
                self.routeplan_display.setText(str(self.route_plan_data))

            if message[0] == "data/obstacle":
                self.obs_det_bool = message[1]
                print(self.obs_det_bool)
                self.obs_det_display.setText(str(self.obs_det_bool))

        else:
            pass

            # print(self.qData.get()[1])


def randomspeed():
    return 10


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_())

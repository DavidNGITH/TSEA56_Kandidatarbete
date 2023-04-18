# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'OneDrive - Linköpings universitet/Programmering/TSEA56/Designer_gui_1.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.

import paho.mqtt.client as mqtt
from PyQt5 import QtCore, QtGui, QtWidgets
import time
import multiprocessing
import keyboard
import random

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(986, 672)

        #starts time
        self.update_time_bool = False
        self.current_time = "0"
        self.start = time.time()

        self.distance_to_obj = "0"

        self.type_of_mode = ""
        self.qData = multiprocessing.Queue()

        #bool for starting car
        self.is_driving = False

        self.car_distance_driven = 0
        self.car_speed_data = list()
        self.delta_t1_speed = 0
        self.delta_t2_speed = 0
        #initate straight ahead and zero speed
        self.speed = 0
        self.steering = 50
        self.obs_det_bool = False
        self.crs_data = "A to B"
        self.lat_pos_data = 0
        self.route_plan_data = "A to B to D to F"
        





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

        #manual mode
        self.manual_mode = QtWidgets.QPushButton(Dialog)
        self.manual_mode.setGeometry(QtCore.QRect(20, 220, 161, 26))
        self.manual_mode.setObjectName("manual_mode")

        #semi-automatic
        self.semi_auto_mode = QtWidgets.QPushButton(Dialog)
        self.semi_auto_mode.setGeometry(QtCore.QRect(20, 270, 161, 31))
        self.semi_auto_mode.setObjectName("semi_auto_mode")

        #automatic
        self.auto_mode = QtWidgets.QPushButton(Dialog)
        self.auto_mode.setGeometry(QtCore.QRect(20, 320, 161, 31))
        self.auto_mode.setObjectName("auto_mode")

        #start button to initate car
        self.start_car = QtWidgets.QPushButton(Dialog)
        self.start_car.setGeometry(QtCore.QRect(240, 230, 141, 51))
        self.start_car.setStyleSheet("background-color: green")
        self.start_car.setObjectName("start_car")

        #stop button
        self.stop_car = QtWidgets.QPushButton(Dialog)
        self.stop_car.setGeometry(QtCore.QRect(240, 300, 141, 51))
        self.stop_car.setStyleSheet("background-color: red")
        self.stop_car.setObjectName("stop_car")

        #KEYBOARD
        self.upbutton = QtWidgets.QPushButton(Dialog)
        self.upbutton.setGeometry(QtCore.QRect(110, 550, 81, 26))
        self.upbutton.setObjectName("upbutton")

        self.downbutton = QtWidgets.QPushButton(Dialog)
        self.downbutton.setGeometry(QtCore.QRect(110, 610, 81, 26))
        self.downbutton.setObjectName("downbutton")

        self.leftbutton = QtWidgets.QPushButton(Dialog)
        self.leftbutton.setGeometry(QtCore.QRect(20, 580, 81, 26))
        self.leftbutton.setObjectName("leftbutton")

        self.rightbutton = QtWidgets.QPushButton(Dialog)
        self.rightbutton.setGeometry(QtCore.QRect(200, 580, 81, 26))
        self.rightbutton.setObjectName("rightbutton")
        ###

        #textbox which sends commands
        self.send_command = QtWidgets.QPushButton(Dialog)
        self.send_command.setGeometry(QtCore.QRect(280, 425, 120, 30))
        self.send_command.setObjectName("send_command")

        self.manual_control_label = QtWidgets.QLabel(Dialog)
        self.manual_control_label.setGeometry(QtCore.QRect(110, 510, 101, 16))
        self.manual_control_label.setObjectName("manual_control_label")

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

        self.time_display = QtWidgets.QTextBrowser(Dialog)
        self.time_display.setGeometry(QtCore.QRect(670, 50, 301, 31))
        self.time_display.setObjectName("time_display")

        self.drive_distance_display = QtWidgets.QTextBrowser(Dialog)
        self.drive_distance_display.setGeometry(QtCore.QRect(670, 90, 301, 31))
        self.drive_distance_display.setObjectName("drive_distance_display")

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

        self.mqtt_init()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_time)
        self.timer.start(70)  # update every 1000 milliseconds (1 second)

        self.messagetimer = QtCore.QTimer()
        self.messagetimer.timeout.connect(self.updatedata)
        self.messagetimer.start(100)

        self.drivingtimer = QtCore.QTimer()
        self.drivingtimer.timeout.connect(self.drive_function)
        self.drivingtimer.start(25)

        self.distanceupdate_timer = QtCore.QTimer()
        #self.distanceupdate_timer.timeout.connect(self.updatedistancedriven)
        #self.drivingtimer.start(1000)


    def drive_function(self):
        if ((self.type_of_mode == "Manual") & (self.is_driving)):
            hotkey = keyboard.get_hotkey_name()
            #print(hotkey)
            if (hotkey == "uppil") | (hotkey == "up"):
                if self.speed <= 251:
                    self.speed += 4
                print(self.speed)
                self.throttle_display.setText(str(self.speed))
            if (hotkey == "nedpil") | (hotkey == "down"):
                if self.speed > 4:
                    self.speed -= 4
                print(self.speed)
                self.throttle_display.setText(str(self.speed))
            if (hotkey == "högerpil") | (hotkey == "right"):
                if self.steering <= 96:
                    self.steering += 4
                self.bearing_display.setText(str(self.steering))
            if (hotkey == "vänsterpil") | (hotkey == "left"):
                if self.steering >= 4:
                    self.steering -= 4
                self.bearing_display.setText(str(self.steering))
            if hotkey == "space":
                self.speed = 0
            if hotkey != "":
                self.mqtt_client.publish("speed", self.speed)
                self.mqtt_client.publish("steering", self.steering)

    def update_time(self):
        # get the current time and format it as a string
        if self.update_time_bool:
            self.current_time = (str(round(time.time() - self.start,2)))
        # set the text of the time_label widget to the current time
        else:
            #start = time.time()
            pass
        self.time_display.setText(self.current_time)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Very fast taxicar, vroom vroom"))

        self.obst_det_head.setText(_translate("Dialog", "Obstacle detection:"))

        self.obst_det_label.setText(_translate("Dialog", "Obstacle detected"))

        self.obst_dist_label.setText(_translate("Dialog", "Distance to obstacle"))

        self.drive_mode_head.setText(_translate("Dialog", "Drive mode:"))

        self.manual_mode.setText(_translate("Dialog", "Manual mode"))
        self.semi_auto_mode.setText(_translate("Dialog", "Semi-automatic mode"))
        self.auto_mode.setText(_translate("Dialog", "Automatic mode"))
        self.start_car.setText(_translate("Dialog", "START"))
        self.stop_car.setText(_translate("Dialog", "STOP"))
        self.upbutton.setText(_translate("Dialog", "Up"))
        self.downbutton.setText(_translate("Dialog", "Down"))
        self.leftbutton.setText(_translate("Dialog", "Left"))
        self.rightbutton.setText(_translate("Dialog", "Right"))
        self.send_command.setText(_translate("Dialog", "Send command"))

        self.manual_control_label.setText(_translate("Dialog", "Manual control"))
        self.command_input_label.setText(_translate("Dialog", "Command input"))
        self.drive_info_label.setText(_translate("Dialog", "Drive info"))
        self.time_label.setText(_translate("Dialog", "Time:"))
        self.distance_label.setText(_translate("Dialog", "Distance:"))
        self.speed_label.setText(_translate("Dialog", "Speed:"))
        self.throttle_label.setText(_translate("Dialog", "Throttle:"))
        self.bearing_label.setText(_translate("Dialog", "Bearing:"))
        self.crs_label.setText(_translate("Dialog", "Current road segment:"))
        self.lat_pos_label.setText(_translate("Dialog", "Lateral position:"))
        self.routeplan_label.setText(_translate("Dialog", "Planned route:"))
        self.map_label.setText(_translate("Dialog", "Map:"))


    def connect_buttons(self):
        self.manual_mode.clicked.connect(self.on_manual_mode_click)
        self.semi_auto_mode.clicked.connect(self.on_semi_auto_mode_click)
        self.auto_mode.clicked.connect(self.on_auto_mode_click)
        self.send_command.clicked.connect(self.on_send_command)

        #start and stop
        self.start_car.clicked.connect(self.on_start_car_click)
        self.stop_car.clicked.connect(self.on_stop_car_click)

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

        print(self.Command_input_box.toPlainText())

    def on_start_car_click(self):
        #starts car
        self.update_time_bool = 1
        #timer counter
        self.start = time.time()  
        #starts first measured time for calculating distance 
        self.delta_t1_speed = time.time()
        self.is_driving = True 
        print("START")

    def on_stop_car_click(self):
        #stops car
        self.current_time = "0"
        self.update_time_bool = 0
        self.is_driving = False

        self.speed = 0  #reset speed
        self.steering = 50 #reset wheels
        
        self.mqtt_client.publish("stop", "1")
        self.mqtt_client.publish("speed", self.speed)
        self.mqtt_client.publish("steering", self.steering)
        print("STOP")

    def mqtt_init(self):
    #initate connection
        try:
            broker_ip = "10.241.242.186"
            broker_port = 1883
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.username_pw_set("tsea56G09", "mindset")
            self.mqtt_client.connect(broker_ip, broker_port)
            self.mqtt_client.subscribe("data/distance")
            self.mqtt_client.subscribe("data/speed")
            self.mqtt_client.loop_start()
            self.mqtt_client.on_message = self.on_message
        except: 
            print("Failed to setup connection")

    def on_message(self, client, userdata, message):
        m = str(message.payload.decode("utf-8"))
        t = message.topic
        self.qData.put((t,m))
        print("Recieved message")
        
    def updatedata(self):
        if not self.qData.empty():
            print("updating data")
            message = self.qData.get()
            if message[0] == "data/distance":
                #print ("Distance recieved")
                self.distance_to_obj = message[1]
                #print(self.distance_to_obj)
                self.obs_dist_display.setText(str(self.distance_to_obj))
            
            if message[0] == "data/speed":
                print ("Speed recieved")
                #self.num_of_speeds += 1
                #recieves speed and shows it on the gui
                self.car_speed_data = message[1]
                print(self.car_speed_data)
                self.speed_display.setText(str(self.car_speed_data))
                #checks if car is driving and then prints distance based on a difference in time
                if self.is_driving:
                    self.delta_t2_speed = time.time()
                    delta_t = self.delta_t2_speed - self.delta_t1_speed
                    self.delta_t1_speed = self.delta_t2_speed
                    self.car_distance_driven += self.car_speed_data*delta_t
                    self.drive_distance_display.setText(str(int(self.car_distance_driven)))


            #Throttle and Bearing data given from gui
            
            if message[0] == "data/crs":
                print ("crs recieved")
                #self.num_of_speeds += 1
                #self.crs_data = message[1]
                #print(self.crs_data)
                self.crs_display.setText(str(self.crs_data))
            
            if message[0] == "data/lat_pos":
                print ("lat_pos recieved")
                #self.num_of_speeds += 1
                #self.lat_pos_data = message[1]
                #print(self.lat_pos_data)
                self.lateral_pos_display.setText(str(self.lat_pos_data))

            if message[0] == "data/route_plan":
                print ("route_plan recieved")
                #self.num_of_speeds += 1
                #self.route_plan_data = message[1]
                #print(self.route_plan_data)
                self.routeplan_display.setText(str(self.route_plan_data))
        else:
            pass            
            

                #print(self.qData.get()[1])
           
        
        
    #def time_printer():
    #    self.time_display.

def randomspeed():
    return 10

if __name__ == "__main__":
    import sys
    #start = time.time()
    app = QtWidgets.QApplication(sys.argv)

    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_())

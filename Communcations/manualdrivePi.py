import paho.mqtt.client as mqtt
import socket
import io
import fcntl
import time
import string
import smbus
from smbus2 import SMBus, i2c_msg

bus = SMBus(1)
gas = 0
styr = 50

def main():
    hostname = socket.gethostname()

    ip_adress = socket.gethostbyname(hostname)

    print(hostname) 
    print(ip_adress)

    topic = "manualdrive"
    broker_ip = ip_adress
    broker_port = 1883

    mqttclient = mqtt.Client()

    mqttclient.connect(broker_ip, broker_port)
    
    mqttclient.publish(topic, "Hello World ")
    
    def on_message(client, userdata, message):
        
        try:
            typ_av_styr = int(message.payload.decode("utf-8"))
            
        except:
            typ_av_styr = 0
        global gas
        global styr

        
        if typ_av_styr == 1:
            if (gas <=198):
                gas += 2
                bus.write_byte_data(0x4a,0,gas)
            else:
                bus.write_byte_data(0x4a,0,gas)
                
        elif typ_av_styr == 2:
            global styr
            if styr >= 1:
                styr -= 1
                bus.write_byte_data(0x4a,1,styr)
                print("Vänster")
            else:
                bus.write_byte_data(0x4a,1,styr)
            #vänster
                
        elif typ_av_styr == 3:
            if styr <= 99:
                styr += 1
                print("Höger")
                bus.write_byte_data(0x4a,1,styr)
            else:
                bus.write_byte_data(0x4a,1,styr)
                    
            #höger
        #print("Recieved:", str(message.payload.decode("utf-8")))
        elif typ_av_styr == 4:
            if gas >=2:
                
                gas -= 2
                bus.write_byte_data(0x4a,0,gas)
            else:
                bus.write_byte_data(0x4a,0,gas)
        
        elif typ_av_styr == 5:
            styr = 50
            bus.write_byte_data(0x4a,1,styr)
            print("styr fram")
            
        elif typ_av_styr == 6:
            #nödbroms
            gas = 0
            bus.write_byte_data(0x4a,0,gas)
        
    mqttclient.on_message = on_message
        
    mqttclient.subscribe(topic)
        
    mqttclient.loop_forever()
        
        #mqttclient.disconnect()
        
main()



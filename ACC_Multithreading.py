import serial
import time
import math
import struct
import threading
import Queue

queue_lidar = Queue.Queue(1000)
queue_uno32 = Queue.Queue(1000)
port_lidar = "COM3"                   #serial port address for LIDAR
baudrate_lidar = 115200               #baudrate at which LIDAR sends data
port_uno32 = "COM8"                   #serial port address for UNO32
baudrate_uno32 = 115200               #baudrate at which UNO32 sends data
port_arduino = "COM6"                 #serial port address for Arduino
baudrate_arduino = 115200             #baudrate at which Arduino receives data
safe_distance = 2000                  #safe distance to obstacle in mm
set_speed = 1                         #set speed for cruise control
level = 0
last_value = [0,0,0,0,0,0,0,0]
tol_l = set_speed - 0.05              #lower tolerance limit
tol_h = set_speed + 0.05              #higher tolerance limit
init_vol = set_speed * 100            #initial voltage
l_speed = 0                           #last value of speed
angle = [160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,204]
distance = [2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001,2001]
stop = 0                              #stop flag

lidar = serial.Serial(port_lidar,baudrate_lidar)                            #read LIDAR data from serial port
uno32 = serial.Serial(port_uno32,baudrate_uno32)                            #read distance information from UNO32
arduino = serial.Serial(port_arduino,baudrate_arduino)                      #send control signals to arduino
time.sleep(3)                                                               #wait 3 seconds for communication to establish

def checksum(data):
    #checksum for received data - logic @ https://xv11hacking.wikispaces.com/LIDAR+Sensor
    data_list = []
    for t in range(10):
        data_list.append(data[2*t] + (data[2*t+1] << 8))

    chk32 = 0
    for d in data_list:
        chk32 = (chk32 << 1) + d

    checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 )
    checksum = checksum & 0x7FFF
    return int( checksum )

def calculate_data(data):                                                   #find distance data in mm 
    raw_distance_1 = data[0]
    raw_distance_2 = data[1]
    raw_signal_strength_1 = data[2]
    raw_signal_strength_2 = data[3]

    distance = raw_distance_1 | (( raw_distance_2 & 0x3f) << 8)             #distance information is on 13 pins
    return int( distance )

def read_lidar(lidar,level):
    #print("lidar thread")
    #level = 0
    while(True):
        if level == 0:
            start = ord(lidar.read())
            if start == 250:                                                    #look for start of conversation byte - 0xFA - 250
                level = 1
            else:
                level = 0
        elif level == 1:
            incoming_index = ord(lidar.read(1))
            if incoming_index >= 200 and incoming_index <= 210:                                  #look for angles between 160 and 200 degrees
                level = 2
            else:
                level = 0
        elif level == 2:
            level = 0
            rpm = [ord(b) for b in lidar.read(2)]
            data_0 = [ord(b) for b in lidar.read(4)]
            data_1 = [ord(b) for b in lidar.read(4)]
            data_2 = [ord(b) for b in lidar.read(4)]
            data_3 = [ord(b) for b in lidar.read(4)]
            all_data = [250, incoming_index] + rpm + data_0 + data_1 + data_2 + data_3            #collect all data for required angles
            byte_checksum = [ord(b) for b in lidar.read(2)]                                       #collect incoming checksum from serial port
            incoming_checksum = int(byte_checksum[0]) + (int(byte_checksum[1]) << 8)              #extract angle and distance data only if checksum matches
            if checksum(all_data) == incoming_checksum and \
                                                          (all_data[5] != 128 and
                                                           all_data[9] != 128 and
                                                           all_data[13] != 128 and
                                                           all_data[17] != 128):
                required_data = []
                required_data.append(((all_data[1] - 160) * 4) + 0)                               #angle for 1st set in data packet
                required_data.append(calculate_data(data_0))                                      #distance for 1st set in data packet - above angle

                required_data.append(((all_data[1] - 160) * 4) + 1)                               #angle for 2nd set in data packet
                required_data.append(calculate_data(data_1))                                      #distance for 2nd set in data packet - above angle

                required_data.append(((all_data[1] - 160) * 4) + 2)
                required_data.append(calculate_data(data_2))

                required_data.append(((all_data[1] - 160) * 4) + 3)
                required_data.append(calculate_data(data_3))
                queue_lidar.put(required_data)

def read_uno32(uno32):
    while(True):
        speed = float(uno32.readline())
        queue_uno32.put(speed)

thread_lidar = threading.Thread(target = read_lidar,args=(lidar,level),).start()            #thread for lidar
thread_uno32 = threading.Thread(target = read_uno32,args=(uno32,),).start()                 #thread for uno32

while True:
    if(queue_lidar.empty()):                                                            #check if LIDAR data is coming or not
        required_data = last_value
    else:
        required_data = queue_lidar.get(True,1)
        last_value = required_data
    print(required_data)
    try:                                                                                #catch if UNO32 fails to send speed information
        speed = float(queue_uno32.get(False,1))
        l_speed = speed
    except Queue.Empty:
        speed = l_speed
        #print("missed speed reading")
    if (speed != 0 or stop == 1):                                            #if tires are rotating or car has been stopped purposely
        print(speed)
        a1 = required_data[0]
        for i in range(44):                                                  #arrange distance information as per angles in a single list
            if(a1 == angle[i]):
                distance[i] = required_data[1]
        a2 = required_data[2]
        for i in range(44):
            if(a2 == angle[i]):
                distance[i] = required_data[3]
        a3 = required_data[4]
        for i in range(44):
            if(a3 == angle[i]):
                distance[i] = required_data[5]
        a4 = required_data[6]
        for i in range(44):
            if(a4 == angle[i]):
                distance[i] = required_data[7]
        distance.sort()
        closest_obstacle = distance[0]
        print closest_obstacle
        if (speed > tol_l and speed < tol_h and closest_obstacle > safe_distance):
            arduino.writelines(str(init_vol))
            time.sleep(0.005)
        elif (speed < tol_l and closest_obstacle > safe_distance):
            if(init_vol < 250):
                init_vol = init_vol + 1
            else:
                init_vol = 250
            arduino.writelines(str(init_vol))
            time.sleep(0.005)
        elif (speed > tol_h and closest_obstacle > safe_distance):
            if(init_vol > 0):
                init_vol = init_vol - 1
            else:
                init_vol = 0
            arduino.writelines(str(init_vol))
            time.sleep(0.005)
        elif closest_obstacle > 1000 and closest_obstacle < 2000:                   #Proportional control for ACC
            temp = closest_obstacle - 1000
            temp_2 = map(int, str(temp))
            temp_3 = float(temp_2[0] * 10)
            temp_3 = temp_3 / 100
            org = init_vol
            init_vol = init_vol * temp_3
            arduino.writelines(str(init_vol))
            time.sleep(0.005)
            init_vol = org
        elif closest_obstacle < 1000:                                               #stop if critically close
            init_vol = 0
            arduino.writelines(str(init_vol))
            time.sleep(0.005)
            stop = 1
            init_vol = 100
    elif speed == 0 and stop != 1:
        arduino.writelines(str(init_vol))
        time.sleep(0.005)
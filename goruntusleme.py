import numpy as np
import torch
import cv2
#from pathlib import Path
import time
from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

connection_string="127.0.0.1:14550"
#connection_string="tcp:127.0.0.1:5763"

#iha = connect(connection_string, wait_ready=True)

#iha = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)

#iha = connect(connection_string, baud=115200, wait_ready=True, timeout=60)

vehicle=connect(connection_string, wait_ready=False);vehicle.wait_ready(True,timeout=70)

def armOlVeYuksel(irtifa):
    while vehicle.is_armable is not True:
        print("iha arm edilebilir durumda değil.\n")
        time.sleep(1)
    
    print("iha arm edilebilir durumda.\n")

    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(1)

    # iha.airspeed = 1
    # iha.groundspeed = 1
    # iha.velocity[0] =1
    # iha._yawspeed=1
    # iha._pitchspeed=1

    print("iha " +str(vehicle.mode)+ " moduna alindi.\n")

    vehicle.armed = True
    while vehicle.armed is not True:
        print("iha arm ediliyor...\n")
        time.sleep(1)
    print("iha arm edildi.\n")

    vehicle.simple_takeoff(irtifa)

    while vehicle.location.global_relative_frame.alt < irtifa*0.91:
        time.sleep(1)
        print(f"iha hedefe yükseliyor : " + str(vehicle.location.global_relative_frame.alt))
        print("\n")

    print("İha istenilen irtifaya yükseldi.\n")

armOlVeYuksel(10)

def send_local_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def send_global_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

global komut
komut = vehicle.commands
komut.clear()
time.sleep(1)

komut.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 0, 0, 7))

grid_size = 6
grid_center = int(grid_size/2)
grid_spacing = 50 # adjust as needed



# Camera Settings
camera_Width  = 1024 # 1280 # 640
camera_Heigth = 780  # 960  # 480
centerZone    = 100

# GridLine color green and thickness
lineColor = (0, 255, 0) 
lineThickness = 1

# message color and thickness
colorWhite = (255,255,255)
colorBlack = (0,0,0)
colorBlue = (255, 0, 0) 
colorGreen = (0, 255, 0) 
colorRed = (0, 0, 255) #red
messageThickness = 2

dsize = (camera_Width, camera_Heigth)

def displayGrid(frame):
    # Add a 5x5 Grid

    for i in range(grid_size):
        cv2.line(frame, (int(camera_Width/grid_size)*i, 0), (int(camera_Width/grid_size)*i, camera_Heigth), lineColor, lineThickness)
        cv2.line(frame, (0, int(camera_Heigth/grid_size)*i), (camera_Width, int(camera_Heigth/grid_size)*i), lineColor, lineThickness)
    
    # Add center lines
    cv2.line(frame, (int(camera_Width/2), 0), (int(camera_Width/2), camera_Heigth), lineColor, lineThickness)
    cv2.line(frame, (0, int(camera_Heigth/2)), (camera_Width, int(camera_Heigth/2)), lineColor, lineThickness)

#send_global_ned_velocity(0,5,0) yola dik sağa
#kırmızı artı x

#send_global_ned_velocity(0,0,-1) # 2 metre yukarı çıkar

#send_global_ned_velocity(5,0,0) # yolla aynı hizada

def calculatePositionForDetectedPerson(frame, x, y, h, w):

    # calculate direction and relative position of the person
    cx = int(x + (w / 2))  # Center X of the person
    cy = int(y + (h / 2))  # Center Y of the person
    dir_list = []

    if (cx < int(camera_Width/2)-grid_spacing*grid_center):
        if (cx < int(camera_Width/2)-2*grid_spacing*grid_center):
            dir_list.append("GO FAR LEFT ")
            send_global_ned_velocity(-1,0,0)
        else:
            dir_list.append("GO LEFT ")
            send_global_ned_velocity(-0.5,0,0)
    elif (cx > int(camera_Width/2)+grid_spacing*grid_center):
        if (cx > int(camera_Width/2)+2*grid_spacing*grid_center):
            dir_list.append("GO FAR RIGHT ")
            send_global_ned_velocity(1,0,0)
        else:
            dir_list.append("GO RIGHT ")
            send_global_ned_velocity(0.5,0,0)
        
    if (cy < int(camera_Heigth/2)-grid_spacing*grid_center):
        if (cy < int(camera_Heigth/2)-2*grid_spacing*grid_center):
            dir_list.append("GO FAR UP ")
            send_global_ned_velocity(0,0,-0.6)
        else:
            dir_list.append("GO UP ")
            send_global_ned_velocity(0,0,-0.3)
    elif (cy > int(camera_Heigth/2)+grid_spacing*grid_center):
        if (cy > int(camera_Heigth/2)+2*grid_spacing*grid_center):
            dir_list.append("GO FAR DOWN ")
            send_global_ned_velocity(0,0,0.6)
        else:
            dir_list.append("GO DOWN ")
            send_global_ned_velocity(0,0,0.3)
            
    if not dir_list:
        cv2.putText(frame, "CENTER", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, colorGreen, 2)
    else:
        cv2.putText(frame, " ".join(dir_list), (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, colorGreen, 2)
        
    # display detected person frame, line from center and direction to go
    cv2.line(frame, (int(camera_Width/2), int(camera_Heigth/2)), (cx, cy), colorWhite, messageThickness)
    cv2.rectangle(frame, (x, y), (x + w, y + h), colorRed, messageThickness)
    cv2.putText(frame, str(int(x)) + " " + str(int(y)), (x - 20, y - 45), cv2.FONT_HERSHEY_COMPLEX, 0.7, colorRed, messageThickness)



video_capture=cv2.VideoCapture('deneme.mp4')
time.sleep(1.0)

model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
#b=model.names[0] = 'person'


while True:
    
    ret,frameOrig =video_capture.read()
    if not ret:
        break
        

    frame=cv2.resize(frameOrig, dsize)
    displayGrid(frame)

    results = model(frame)

    persons = model.names[0] 

    person_counter = 0

    person_id = 0
   
    df = results.pandas().xyxy[0]  # get the DataFrame of detected objects
    person_boxes = {}  # dictionary to store the box with highest confidence for each person
    for _, row in df.iterrows():
        if row['name'] == persons:  # check if the detected object is a person
            if 'person_id' not in row:  # if person_id is not assigned yet
                row['person_id'] = person_counter  # assign a new person_id
                person_counter += 1
            else:
                person_id = row['person_id']  # use the assigned person_id
            if person_id not in person_boxes or row['confidence'] > person_boxes[person_id]['confidence']:
                person_boxes[person_id] = row

    # draw the box with highest confidence for each person
    person_id = 0  # initialize person ID counter

    for person_id, box in person_boxes.items():
        x, y, w, h = int(box['xmin']), int(box['ymin']), int(box['xmax'] - box['xmin']), int(box['ymax'] - box['ymin'])
        calculatePositionForDetectedPerson(frame, x, y, h, w)
        cv2.putText(frame, f'Person {person_id}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        person_id += 1
            #cv2.putText(frame, f'Persons: {total_persons}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.imshow("Human Detection",frame)
    if cv2.waitKey(1)&0xFF == ord('q'):
        break
video_capture.release()
cv2.destroyAllWindows()
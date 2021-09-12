#BİSMİLLAHİRRAHMANİRRAHİM
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil
import numpy as np
import cv2
import imutils
from collections import deque
import time
import math
import threading as th

connection_string = "/dev/ttyACM0"
baud_rate = 115200
iha = connect(connection_string,baud_rate, wait_ready=True)

webcam = cv2.VideoCapture(0)

Altitude = 1300

#Controller config
UpdateRate = 1
MaxMovementRatePositive = 0.020
MaxMovementRateNegative = -0.020
DivisionValueX = 14400
DivisionValueY = 9450
TargetCircleMultiplayer = 1


font = cv2.FONT_HERSHEY_SIMPLEX

points = deque(maxlen=32)
counter = 0
(dX, dY) = (0, 0)
direction = ""
Run = True
raduis = 0
CircleLostCount = 0
InsideCircle = False
TargetCircleRaduis = 0

#Display variables
DisplayDx = 0.0
DisplayDy = 0.0
DisplayTreshhold = 0

bulundu = False
ustunde = FalseAltitude = 1300

#Controller config
UpdateRate = 1
MaxMovementRatePositive = 0.020
MaxMovementRateNegative = -0.020
DivisionValueX = 14400
DivisionValueY = 9450
TargetCircleMultiplayer = 1

font = cv2.FONT_HERSHEY_SIMPLEX

points = deque(maxlen=32)
counter = 0
(dX, dY) = (0, 0)
direction = ""
Run = True
raduis = 0
CircleLostCount = 0
InsideCircle = False
TargetCircleRaduis = 0

#Display variables
DisplayDx = 0.0
DisplayDy = 0.0
DisplayTreshhold = 0

bulundu = False
ustunde = FalseAltitude = 1300

#Controller config
UpdateRate = 1
MaxMovementRatePositive = 0.020
MaxMovementRateNegative = -0.020
DivisionValueX = 14400
DivisionValueY = 9450
TargetCircleMultiplayer = 1

font = cv2.FONT_HERSHEY_SIMPLEX

points = deque(maxlen=32)
counter = 0
(dX, dY) = (0, 0)
direction = ""
Run = True
raduis = 0
CircleLostCount = 0
InsideCircle = False
TargetCircleRaduis = 0

#Display variables
DisplayDx = 0.0
DisplayDy = 0.0
DisplayTreshhold = 0

bulundu = False
ustunde = False

################

kirmizi_bulundu = False
kx=iha.location.global_relative_frame.lat
ky=iha.location.global_relative_frame.lon

if iha.mode is not VehicleMode("GUIDED"):
	iha.mode = VehicleMode("GUIDED")

cmds = iha.commands
#cmds.download()
#cmds.wait_ready()

def ortala():
    global bulundu
    global InsideCircle
    global counter
    sayac = 0
    while (sayac < 25):
        _, frame = webcam.read()
    
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
        Greenmask = cv2.inRange(hsv, (20, 150, 150), (190,255,255))
        Greenmask = cv2.erode(Greenmask, None, iterations=2)
        Greenmask = cv2.dilate(Greenmask, None, iterations=2)
    
        FoundedContours = cv2.findContours(Greenmask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        FoundedContours = imutils.grab_contours(FoundedContours)
        center = None
    
        if len(FoundedContours) > 0:
            
            Circle = max(FoundedContours, key=cv2.contourArea)
            ((x, y), raduis) = cv2.minEnclosingCircle(Circle)
            Middle = cv2.moments(Circle)
    
            center = (int(Middle["m10"] / Middle["m00"]), int(Middle["m01"] / Middle["m00"]))
    
            if raduis > 10:
                bulundu = True
                cv2.circle(frame, (int(x), int(y)), int(raduis), (0, 0, 255), 2)
                TargetCircleRaduis = raduis * TargetCircleMultiplayer
                cv2.circle(frame, (int(x), int(y)), int(TargetCircleRaduis), (177, 196, 193), 2)
                cv2.circle(frame, center, 5, (0, 255, 255), -1)
                points.appendleft(center)
            else:
                cv2.putText(frame, "Error no circel Found", (10, 700), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
                # CircleLostCount+= 1
    
        else:
            cv2.putText(frame, "Error no circel Found", (10, 700), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
            # CircleLostCount+= 1
    
        DxCount = 0.0
        DyCount = 0.0
    
        for i in np.arange(1, len(points)):
            if points[i - 1] is None or points[i] is None:
                continue
            # check if 400,400 is in surface of circle ((x,y),raduis) if case then no need to do calculations
    
            if i == 1:
    
                
                DxCount = float(points[i][0]) - 320.0
                DyCount = 250 - float(points[i][1])
    
                cv2.line(frame, points[i - 1], (320, 250), (0, 0, 255), 5)
    
               
               
                # cv2.putText(frame,str(DxCount),(10,100), font, 1,(255,0,0),2,cv2.LINE_AA)
                # cv2.putText(frame,str(DyCount),(10,150), font, 1,(255,0,0),2,cv2.LINE_AA)
    
                
    
                if (int(points[i][0]) - 320) ** 2 + (int(points[i][1]) - 250) ** 2 < (TargetCircleRaduis / 2) ** 2:
                    InsideCircle = True
                else:
    
                    InsideCircle = False
    
        #cv2.putText(frame, "Inside circel" + str(InsideCircle), (10, 120), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
    
    
    
        if (counter % UpdateRate) == 0:
    
            DisplayDx = DxCount
            DisplayDy = DyCount
    
    
            Xmovement = DxCount / DivisionValueX  # 6400
            Ymovement = (DyCount / DivisionValueY)  # 4200
    
            if Xmovement > MaxMovementRatePositive:
                Xmovement = MaxMovementRatePositive
            elif Xmovement < MaxMovementRateNegative:
                Xmovement = MaxMovementRateNegative
            if Ymovement > MaxMovementRatePositive:
                Ymovement = MaxMovementRatePositive
            elif Ymovement < MaxMovementRateNegative:
                Ymovement = MaxMovementRateNegative
    
    
            AltitudeCommand = 0.0

        Ymovement = Ymovement * 10
        Xmovement = Xmovement * 10
       
        if(InsideCircle == False):
            if Xmovement != 0.0:
                print("X: " + str(Xmovement) + "      Y:" + str(Ymovement))
                goto_position_target_local_ned(Ymovement, Xmovement, 0)
        else:
            print("hedefe inis yapiliyor")
            iha.mode = VehicleMode("LAND") 
            break

        time.sleep(2)       
        sayac = sayac + 1  
        
    
    
        #cv2.putText(frame, Direction1, (10, 30), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
        #cv2.putText(frame, Direction2, (10, 60), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
        #cv2.putText(frame, "Altitude Treshhold: " + str(DisplayTreshhold), (10, 90), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
    
        cv2.circle(frame, (320, 250), 10, (255, 0, 0), -1)
    
        cv2.imshow("Detected", frame)
        counter += 1
    
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    
    if(InsideCircle == False):
        print("eve dönülüyor")
        iha.mode = VehicleMode("RTL")

enkUzunluk = 100

def HedefTarama():
    global enkUzunluk

    UpdateRate = 1
    MaxMovementRatePositive = 0.020
    MaxMovementRateNegative = -0.020
    DivisionValueX = 14400
    DivisionValueY = 9450
    TargetCircleMultiplayer = 3
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    points = deque(maxlen=32)
    counter = 0
    (dX, dY) = (0, 0)
    direction = ""
    Run = True
    raduis = 0
    CircleLostCount = 0
    InsideCircle = False
    TargetCircleRaduis = 0
    
    #Display variables
    DisplayDx = 0.0
    DisplayDy = 0.0
    DisplayTreshhold = 0
    
    
    while(1):
        _, frame = webcam.read()

        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
        Greenmask = cv2.inRange(hsv, (20, 150, 150), (190,255,255))
        Greenmask = cv2.erode(Greenmask, None, iterations=2)
        Greenmask = cv2.dilate(Greenmask, None, iterations=2)
    
        FoundedContours = cv2.findContours(Greenmask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        FoundedContours = imutils.grab_contours(FoundedContours)
    
        center = None
    
        if len(FoundedContours) > 0:
            Circle = max(FoundedContours, key=cv2.contourArea)
            ((x, y), raduis) = cv2.minEnclosingCircle(Circle)
            Middle = cv2.moments(Circle)
    
            center = (int(Middle["m10"] / Middle["m00"]), int(Middle["m01"] / Middle["m00"]))
    
            if raduis > 10:
                cv2.circle(frame, (int(x), int(y)), int(raduis), (0, 0, 255), 2)
                TargetCircleRaduis = raduis * TargetCircleMultiplayer
                
                cv2.circle(frame, center, 5, (0, 255, 255), -1)
                points.appendleft(center)
            else:
                cv2.putText(frame, "Error no circel Found", (10, 700), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
                # CircleLostCount+= 1
    
        else:
            cv2.putText(frame, "Error no circel Found", (10, 700), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
            # CircleLostCount+= 1
    
        DxCount = 0.0
        DyCount = 0.0
    
        for i in np.arange(1, len(points)):
            if points[i - 1] is None or points[i] is None:
                continue
    
            if i == 1:
                DxCount = float(points[i][0]) - 320.0
                DyCount = 250 - float(points[i][1])
    
                cv2.line(frame, points[i - 1], (320, 250), (0, 0, 255), 5)
    

        if (counter % UpdateRate) == 0:
        
            DisplayDx = DxCount
            DisplayDy = DyCount
        
        
            Xmovement = DxCount / DivisionValueX  # 6400
            Ymovement = (DyCount / DivisionValueY)  # 4200
        
            if Xmovement > MaxMovementRatePositive:
                Xmovement = MaxMovementRatePositive
            elif Xmovement < MaxMovementRateNegative:
                Xmovement = MaxMovementRateNegative
            if Ymovement > MaxMovementRatePositive:
                Ymovement = MaxMovementRatePositive
            elif Ymovement < MaxMovementRateNegative:
                Ymovement = MaxMovementRateNegative
            
           
            hipo = math.sqrt(Xmovement * Xmovement + Ymovement * Ymovement)
            if hipo != 0.0:
                if hipo < enkUzunluk:
                    enkUzunluk = hipo
                    kx=iha.location.global_relative_frame.lat
                    ky=iha.location.global_relative_frame.lon
                    kirmizi_bulundu = True
            
           
        cv2.circle(frame, (320, 250), 10, (255, 0, 0), -1)
        
        cv2.imshow("Detected", frame)
        counter += 1

        if iha.commands.next == len(cmds):
			break
        
        

    cv2.destroyAllWindows()
def arm():
	while(True):

		if iha.is_armable:
			iha.armed = True
			print("iha arm edildi")
			time.sleep(1)
			break
		time.sleep(1)
		print("iha arm edilebilir değil")


def gorevYuk():
	
	cmds.clear()
	time.sleep(1)
	#waypoint
	cmdk = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10)
	cmdw = Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,-35.36322802 ,149.16554143 ,10)
	cmdfake = Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,-35.36322802 ,149.16554143 ,10)
	cmds.add(cmdk)
	cmds.add(cmdw)
    cmds.add(cmdfake)
	
	
	cmds.upload()
	
	print("görev yüklendi.")

def kalkis(irtifa):
	iha.simple_takeoff(irtifa)

	while iha.location.global_relative_frame.alt < (irtifa*0.9):
		print("yükseliyor")
		time.sleep(0.5)
	print("yükselme tamamlandı")
	time.sleep(2)
	
def kırmızıGorev(x,y,alt):
    if iha.mode is not VehicleMode("GUIDED"):
	    iha.mode = VehicleMode("GUIDED")
    cmds.clear()
    time.sleep(1)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, alt))
    cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,x ,y ,alt))
    cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,x ,y ,alt))
    cmds.upload()
    time.sleep(1)
    iha.mode = VehicleMode("AUTO")
    if iha.mode is VehicleMode("AUTO"):
		print("auto")
    while (True):
        if iha.commands.next == len(cmds):
			break
        print("kırmızıya gidiliyor")
        time.sleep(1)

    print("kırmızıya ulaşıldı")
    time.sleep(1)
    ortala()

def kırmızAra():

	iha.mode = VehicleMode("AUTO")
	if iha.mode is VehicleMode("AUTO"):
		print("auto mode")
    ##########
	while True:
        ###görüntü işleme kırmızı_bulundu ve kx ky ler doldurulcak
		
    ##########
	print("görev tamamlandı")
    if kırmızı_bulundu == True:
        print("görev sonucu: kırmızı bulundu")
        kırmızıGorev(kx,ky,10)
    else:
        print("görev sonucu: kırmızı bulunamadı)
	    iha.mode = VehicleMode("RTL")
	    print("eve dönülüyor")


def goto_position_target_local_ned(north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    """
    msg = iha.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    iha.send_mavlink(msg)


gorevYuk()

arm()

kalkis(10)

kırmızıAra()



 kx=iha.location.global_relative_frame.lat
 ky=iha.location.global_relative_frame.lon
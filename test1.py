from dronekit import connect, Command, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil
con_str="127.0.0.1:14550"
iha=connect(con_str,wait_ready=True)

kırmızı_isFind = False
kx=iha.location.global_relative_frame.lat
ky=iha.location.global_relative_frame.lon

if iha.mode is not VehicleMode("GUIDED"):
	iha.mode = VehicleMode("GUIDED")

cmds = iha.commands
#cmds.download()
#cmds.wait_ready()

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
    #konum = LocationGlobalRelative(x ,y,alt)
    #iha.simple_goto(konum)
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
    ###ortala()

def kırmızAra():

	iha.mode = VehicleMode("AUTO")
	#time.sleep(1)
	if iha.mode is VehicleMode("AUTO"):
		print("auto mode")
	while True:
        ###görüntü işleme kırmızı_isFind ve kx ky ler doldurulcak
		if iha.commands.next == len(cmds):
			break
	print("görev tamamlandı")
    if kırmızı_isFind == True:
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



#iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon

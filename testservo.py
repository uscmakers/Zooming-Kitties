from dronekit import connect
import time
connection_string = '/dev/ttyACM0'
vehicle = connect(connection_string, wait_ready=True, baud=115200, timeout=60)
print("Successfully connected to vehicle at " + connection_string + "!")
vehicle.armed = True
time.sleep(1)
vehicle.channels.overrides['3'] = 1500  # out of 2000

pos1=2000
pos2=1000
print("channel 3 (dc motor) is 1500 (stopped)")
while(1):
    vehicle.channels.overrides['1'] = pos1  # out of 2000
    print("Just set channel 1 to "+ str(pos1) + ".  About to sleep for 5 seconds...")
    time.sleep(5)
    print("Setting channel 1 back to " + str(pos2))
    vehicle.channels.overrides['1'] = pos2
    # vehicle.channels.overrides['3'] = 1000  # out of 2000
    print("Sleeping for 5s...")
    time.sleep(5)

vehicle.armed = False
vehicle.close()
print("Closed vehicle.")

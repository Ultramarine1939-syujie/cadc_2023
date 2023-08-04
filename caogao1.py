import functhion
import time

print ("take off ------ 3m")
functhion.arm_and_takeoff(3)
time.sleep(10)

functhion.goto_position_target_local_ned(0,3,-3)
time.sleep(10)


functhion.goto_position_target_local_ned(6,3,-3)
time.sleep(10)


functhion.goto_position_target_local_ned(6,-3,-3)
time.sleep(10)


functhion.goto_position_target_local_ned(0,-3,-3)
time.sleep(10)


functhion.goto_position_target_local_ned(0,0,-3)
time.sleep(10)

print ("mode  RTL")
vehicle.mode = VehicleMode("RTL")
time.sleep(10)
vehicle.close()





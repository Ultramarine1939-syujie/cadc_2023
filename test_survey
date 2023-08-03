import functhion,time

DURATION = 5
HEIGHT = 3
VEL = 0.2
HEADING = 170
#HEADING = functhion.vehicle.heading

print("下面播报飞机基本状态")
print(" %s" % functhion.vehicle.heading)
print("%s" %functhion.vehicle.groundspeed)
print(" %s" % functhion.vehicle.gps_0)
print(" %s" % functhion.vehicle.rangefinder)
print(" Mode: %s" % functhion.vehicle.mode.name)

def time_count(num):
    for i in range(num+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        print("当前速度为：%s" %functhion.vehicle.groundspeed)
        print("当前角度为: %s:" %functhion.vehicle.heading)
        if i == 1:
            print("计时结束")

#起飞
print("初始化完成,%s秒后飞行器自动起飞,目标高度：%s" % (DURATION,HEIGHT))
time_count(DURATION)
functhion.arm_and_takeoff(HEIGHT)

#测量
print("起飞完成，%s秒后飞行器开始测量场地" %DURATION)
time_count(DURATION)

#返回
print("测量完成，%s秒后飞行器返回" %DURATION)
time_count(DURATION)
functhion.vehicle_return()

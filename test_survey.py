import functhion,time
from yoloDetect import global_vars

DURATION = 5
HEIGHT = global_vars.HEIGHT
VEL = global_vars.VEL
HEADING = global_vars.HEADING
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

def investigate():  #侦察函数
    print("前往侦察区")
    investigate_pos = [[55,4],[55,-4],
                       [57.5,4],[57.5,-4],
                       [60,4],[60,-4]]
    
    for pos in investigate_pos:
        x,y = functhion.calculate_absolute_target(HEADING,pos[1],pos[2])
        functhion.goto_position_target_local_ned(x, y, -HEIGHT)
        time_count(20)
    print("侦察完成")
    time.sleep(1)

def attack():   #打击函数
    now_pos = [32.5,0]
    print("前往打击区")
    x,y = functhion.calculate_absolute_target(HEADING,now_pos[0],now_pos[1])
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
    time_count(34)
    print("打击完毕")
    time.sleep(1)

#起飞
print("初始化完成,%s秒后飞行器自动起飞,目标高度：%s" % (DURATION,HEIGHT))
time_count(DURATION)
functhion.arm_and_takeoff(HEIGHT)

#测量
print("起飞完成，%s秒后飞行器开始测量场地" %DURATION)
time_count(DURATION)
attack()
investigate()

#返回
print("测量完成，%s秒后飞行器返回" %DURATION)
time_count(DURATION)
functhion.vehicle_return()

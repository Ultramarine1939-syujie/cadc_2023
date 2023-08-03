import functhion,time
from yoloDetect import global_vars

'''syujie
有bug,默认飞正北，别再问了
远航也有点小问题，飞不回来了
后期加解算和去重
总之不能跑
'''

DURATION = 5
HEIGHT = global_vars.HEIGHT
VEL = global_vars.VEL
HEADING = global_vars.HEADING
#HEADING = functhion.vehicle.heading

def time_count(num):
    for i in range(num+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        print("当前速度为：%s" %functhion.vehicle.groundspeed)
        print("当前角度为: %s:" %functhion.vehicle.heading)
        if i == 1:
            print("计时结束")

def drone_init():
    print("下面播报飞机基本状态")
    print(" %s" % functhion.vehicle.heading)
    print(" %s" % functhion.vehicle.gps_0)
    print(" %s" % functhion.vehicle.rangefinder)
    print(" Mode: %s" % functhion.vehicle.mode.name)
    print(" DURATION: %s" % DURATION)

def investigate_vel(now_pos,time_set):
    print("前往侦察区")
    investigate_pos = [[55,4],  [55,-4],
                       [57.5,4],[57.5,-4],
                       [60,4],  [60,-4]]
    for pos in investigate_pos:
        now_pos = [pos[0]-now_pos[0],pos[1]-now_pos[1]]
        functhion.send_ned_velocity(now_pos[0]/time_set,now_pos[1]/time_set,0,time_set)
    print("侦察完成")
    time.sleep(1)
    return now_pos

def attack_vel(now_pos,time_set):
    print("前往打击区")
    attack_pos = [32.5,0]
    now_pos = [attack_pos[0]-now_pos[0],attack_pos[1]-now_pos[1]]
    functhion.send_ned_velocity(now_pos[0]/time_set,now_pos[1]/time_set,0,time_set)
    time.sleep(1)
    print("打击完成")
    return now_pos

#初始化
drone_init()

#起飞
print("初始化完成,%s秒后飞行器自动起飞,目标高度：%s" % (DURATION,HEIGHT))
time_count(DURATION)
functhion.arm_and_takeoff(HEIGHT)

#移动
now_pos = [0,0]
print("起飞完成，%s秒后飞行器开始测量场地" %DURATION)
time_count(DURATION)
now_pos = attack_vel(now_pos,20)
now_pos = investigate_vel(now_pos,20)


#返回
print("测量完成，%s秒后飞行器返回" %DURATION)
time_count(DURATION)
functhion.vehicle_return()

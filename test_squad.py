import functhion,time
DURATION = 5
HEIGHT = 3
VEL = 0.2
#HEADING = 170
HEADING = functhion.vehicle.heading

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

#测试函数
#(length2/VEL)无法使用，敬请注意；相关函数已统一为DURATION
def test_squad1(length):
    print("正方形测试")
    functhion.vehicle.groundspeed = 0.1
    print(functhion.vehicle.groundspeed)
    print("前进%sm" %length)
    x,y = functhion.calculate_absolute_target(HEADING,-length,0)
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range(DURATION+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        print(functhion.vehicle.groundspeed)
        if i==1:
            print("完成")

    print("右行%sm" %length)
    x,y = functhion.calculate_absolute_target(HEADING,-length,length)
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range(DURATION+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        if i==1:
            print("完成")

    print("后退%sm" %length)
    x,y = functhion.calculate_absolute_target(HEADING,0,length)
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range(DURATION+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        if i==1:
            print("完成")

    print("左行%sm" %length)
    x,y = functhion.calculate_absolute_target(HEADING,0,0)
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range(DURATION+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        if i==1:
            print("完成")


#初始化
drone_init()

#起飞
print("初始化完成,%s秒后飞行器自动起飞,目标高度：%s" % (DURATION,HEIGHT))
time_count(DURATION)
functhion.arm_and_takeoff(HEIGHT)

#划线
test_squad1(5)
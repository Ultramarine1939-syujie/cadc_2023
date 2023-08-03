import functhion,time

MISSION = ""
DURATION = 5
HEIGHT = 3
VEL = 0.2
HEADING = 170
#HEADING = functhion.vehicle.heading

print("下面播报飞机基本状态")
print(" %s" % functhion.vehicle.heading)
print(" %s" % functhion.vehicle.gps_0)
print(" %s" % functhion.vehicle.rangefinder)
print(" Mode: %s" % functhion.vehicle.mode.name)
print(" DURATION: %s" % DURATION)
#起飞
print("播报完成,%s秒后飞行器自动起飞,目标高度：%s" % (DURATION,HEIGHT))
for i in range(DURATION,0,-1):
    time.sleep(1)
    print("倒计时：%s" % i)
    if i==0:
        print("起飞！")
functhion.arm_and_takeoff(HEIGHT)

#HEADING设定
print("起飞完成\n设定朝向为: %s\n" % HEADING)
#functhion.condition_yaw(HEADING)
time.sleep(5)
print("当前朝向为: %s\n" % functhion.vehicle.heading)

#任务设定
print("当前任务目标为:%s\n 空速设定:%s\n 开始时间：%s秒后\n" % (MISSION, VEL, DURATION))
functhion.vehicle.groundspeed = VEL
for i in range(DURATION+1,0,-1):
    time.sleep(1)
    print("倒计时：%s" % i)
    if i==1:
        print("开始测试")

#测试函数
def test_squad1(length):
    print("正方形测试")
    functhion.vehicle.groundspeed = 0.1
    print(functhion.vehicle.groundspeed)
    print("前进%sm" %length)
    x,y = functhion.calculate_absolute_target(HEADING,-length,0)
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range(30+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        print(functhion.vehicle.groundspeed)
        if i==1:
            print("完成")

    print("右行%sm" %length)
    x,y = functhion.calculate_absolute_target(HEADING,-length,length)
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range(30+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        if i==1:
            print("完成")

    print("后退%sm" %length)
    x,y = functhion.calculate_absolute_target(HEADING,0,length)
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range(30+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        if i==1:
            print("完成")

    print("左行%sm" %length)
    x,y = functhion.calculate_absolute_target(HEADING,0,0)
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range(30+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        if i==1:
            print("完成")

def test_squad2(length1,length2):

    print("长方形")
    print("前进%sm" %length1)
    x,y = functhion.calculate_absolute_target(HEADING,length1,0)
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range((length1/VEL)+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        if i==1:
            print("完成")

    print("右行%sm" %length2)
    x,y = functhion.calculate_absolute_target(HEADING,length1,length2)
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range((length2/VEL)+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        if i==1:
            print("完成")

    print("后退%sm" %length1)
    x,y = functhion.calculate_absolute_target(HEADING,0,length2)
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range((length1/VEL)+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        if i==1:
            print("完成")

    print("左行%sm" %length2)
    x,y = functhion.calculate_absolute_target(HEADING,0,0)
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range((length2/VEL)+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        if i==1:
            print("完成")

def test_race():
    '''
    x轴负数向前、正数向后
    y轴负数向左、正数向右
    飞行角度目前无法确定
    '''
    print("比赛场地测量")
    print("前往打击区")
    x,y = functhion.calculate_absolute_target(HEADING,30,0)
    functhion.goto_position_target_local_ned(-30, 0, -HEIGHT)    #向前行驶
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range(30+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        if i==1:
            print("完成")
    print("到达打击点")
    time.sleep(5)

    print("前往侦察区")
    x,y = functhion.calculate_absolute_target(HEADING,55,-4)
    functhion.goto_position_target_local_ned(55, -4, -HEIGHT)    #向左行驶
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range(30+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        if i==1:
            print("完成")
    print("到达侦察点")
    time.sleep(5)

    print("开始侦察")
    x,y = functhion.calculate_absolute_target(HEADING,60,4)
    functhion.goto_position_target_local_ned(60, 4, -HEIGHT)
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range(20+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        if i==1:
            print("完成")
    print("侦察完成")
    time.sleep(5)

def test_vel():
    print("速度测试")
    time.sleep(1)
    print("前进")
    functhion.send_ned_velocity(1,0,0,5)
    print("右行")
    functhion.send_ned_velocity(0,1,0,5)
    print("global")
    functhion.send_global_velocity(1,0,0,5)
    functhion.send_global_velocity(0,1,0,10)

test_race()

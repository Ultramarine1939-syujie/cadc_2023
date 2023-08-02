"""
本程序用于验证飞行
"""
import functhion,time

MISSION = "验证"
DURATION = 5
HEIGHT = 3
VEL = 1
HEADING = functhion.vehicle.heading

def test_squard():
    print("正方形测试")

    print("前进5m")
    x,y = functhion.calculate_absolute_target(HEADING,1,0)
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range(DURATION+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        if i==1:
            print("完成")

    print("右行5m")
    x,y = functhion.calculate_absolute_target(HEADING,1,1)
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range(DURATION+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        if i==1:
            print("完成")

    print("后退5m")
    x,y = functhion.calculate_absolute_target(HEADING,0,1)
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range(DURATION+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        if i==1:
            print("完成")

    print("左行5m")
    x,y = functhion.calculate_absolute_target(HEADING,0,0)
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range(DURATION+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        if i==1:
            print("完成")

print("下面播报飞机基本状态")
print(" %s" % functhion.vehicle.heading)
print(" %s" % functhion.vehicle.gps_0)
print(" %s" % functhion.vehicle.rangefinder)
print(" Mode: %s" % functhion.vehicle.mode.name)
print(" DURATION: %s" % DURATION)
print("播报完成,5秒后飞行器自动起飞,目标高度：%s" % HEIGHT)
for i in range(5,0,-1):
    time.sleep(1)
    print("倒计时：%s" % i)
    if i==0:
        print("起飞！")
functhion.arm_and_takeoff(HEIGHT)                # 起飞，高度3m

print("起飞完成\n设定朝向为: %s\n" % HEADING)
functhion.condition_yaw(HEADING)
time.sleep(5)
print("当前角度为: %s:" %functhion.vehicle.heading)
print("当前任务目标为:%s\n 空速设定:%s\n 开始时间：%s秒后\n" % (MISSION, VEL, DURATION))
functhion.vehicle.airspeed = VEL
for i in range(DURATION,0,-1):
    time.sleep(1)
    print("倒计时：%s" % i)
    if i==0:
        print("开始测试")

test_squard()

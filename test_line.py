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

'''
vel模式有bug,位置默认为正北
'''

def line(flag,time=None): #   0表示非vel  1表示vel
    print("开始划线")
    line_pos = [[15,0],[-15,0]]
    now_pos = [0,0]
    if flag==0:
        while True:
            i=1
            for pos in line_pos:
                x,y = functhion.calculate_absolute_target(HEADING,pos[0],pos[1])
                #x,y = pos[0],pos[1]
                functhion.goto_position_target_local_ned(x, y, -HEIGHT)
                time_count(15)
            i=i+1
            if i==2:
                break;
    
    elif flag==1 and time != None:
        while True:
            i=1
            for pos in line_pos:
                now_pos = [pos[0]-now_pos[0],pos[1]-now_pos[1]]
                functhion.send_ned_velocity(now_pos[0]/time,now_pos[1]/time,0,time)
            i=i+1
            if i==2:
                break;
    
    elif flag==1 and time ==None:
        print("时间未设置")
 

#初始化
drone_init()

#起飞
print("初始化完成,%s秒后飞行器自动起飞,目标高度：%s" % (DURATION,HEIGHT))
time_count(DURATION)
functhion.arm_and_takeoff(HEIGHT)

#划线
line(0)
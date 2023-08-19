import threading
from functhion import *
import time
from yoloDetect import global_vars,detect_yolov2
from pid import PID


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
#HEADING = vehicle.heading

def time_count(num):
    for i in range(num+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        print("当前速度为：%s" %vehicle.groundspeed)
        print("当前角度为: %s:" %vehicle.heading)
        if i == 1:
            print("计时结束")

def drone_init():
    print("下面播报飞机基本状态")
    print(" %s" % vehicle.heading)
    print(" %s" % vehicle.gps_0)
    print(" %s" % vehicle.rangefinder)
    print(" Mode: %s" % vehicle.mode.name)
    print(" DURATION: %s" % DURATION)
    global_vars.run = True  # 系统开始运行

def investigate_vel(now_pos,time_set):
    print("前往侦察区")
    investigate_pos = [[55,4],  [55,-4],
                       [57.5,-4],[57.5,4],
                       [60,4],  [60,-4]]
    for pos in investigate_pos:
        target_stage  = [pos[0]-now_pos[0],pos[1]-now_pos[1]]
        now_pos = pos
        set_speed_body_offset(target_stage[0]/time_set,target_stage[1]/time_set,0,time_set)
    print("侦察完成")
    time.sleep(1)
    return now_pos

def go_for_attack(now_pos,time_set):
    print("前往打击区")
    # attack_pos = [32.5,0]
    attack_pos = [3,0]
    target_stage = [attack_pos[0]-now_pos[0],attack_pos[1]-now_pos[1]]
    now_pos = attack_pos
    run_with_detect(target_stage[0]/time_set,target_stage[1]/time_set,0,time_set)
    while not global_vars.found_obj:
        run_with_detect(0.1,0,0,20) # 缓慢前进是否识别到筒
    print("识别到筒子")
    return now_pos

def attack():
    fire = False
    detect_num = 0
    start_time = time.time()
    while not (fire or global_vars.finish_task):
        if start_time - time.time() > 30:   # 30秒超时推出
            print("未识别到目标超时，跳过投弹返航")
            break
        if 61 in global_vars.img_type:                      # 如果要识别的白桶在已经识别到的物品中, 桶子61,人0
            start_time = time.time()
            obj_indx = global_vars.img_type.index(61)       # 找到识别的白桶的在已知物中的下标
            location = global_vars.location[obj_indx]
            control_signal,error = pid.calculate_pos_pid(global_vars.center,location)
            if max(error) < 50:
                detect_num += 1
                print("抵达中心,detect_num = ",detect_num)
                control_signal = [0,0]
                if detect_num > 40:
                    do_set_servo(1000, 5)
                    print("投放水瓶！")
                    fire = True
            else:
                detect_num = 0
            set_speed_on_time(control_signal[0],control_signal[1],0,1)
            print("当前控制速度",control_signal)
            global_vars.location = []
            global_vars.img_type = []
            time.sleep(0.1)
            pass
        pass
    print("打击完成")
    return fire


def show_location():
    while not global_vars.finish_taskn:
        # print("Local Location: %s" % vehicle.location.local_frame)    #NED
        time.sleep(1)

if __name__ == '__main__':
    #初始化
    drone_init()
    
    pid = PID(Kp=0.1,Ki=0.001,Kd=0.0)
    #起飞
    print("初始化完成,%s秒后飞行器自动起飞,目标高度：%s" % (DURATION,HEIGHT))
    time_count(DURATION)
    arm_and_takeoff(HEIGHT)
    print("启动位置显示线程")
    threading.Thread(target=detect_yolov2).start()

    # #移动
    now_pos = [0,0]
    print("起飞完成，%s秒后飞行器开始测量场地" %DURATION)
    time_count(DURATION)
    now_pos = go_for_attack(now_pos,3)
    if attack():
        now_pos = investigate_vel(now_pos,20)


    #返回
    print("测量完成，%s秒后飞行器返回" %DURATION)
    time_count(DURATION)
    vehicle_return()
    global_vars.run = False

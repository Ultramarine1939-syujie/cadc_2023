import argparse
import socket
import threading
import time
import cv2
from dronekit import VehicleMode
import functhion
from yoloDetect import yolo_v2, global_vars

#if_searched = False

def search_point():
    global if_searched
    while not if_searched:
        if 61 in global_vars.img_type:   #change to 61
            if_searched = True
    return if_searched  
        

def detect_yolov2():
    HOST = "127.0.0.1"
    PORT = 5000
    cap = cv2.VideoCapture(0)    # 调用默认摄像头
    cap.set(3, global_vars.frame_width)
    cap.set(4, global_vars.frame_height)
    # 参数列表，你看不懂，也不用管，就这样用就对了
    parser = argparse.ArgumentParser()
    parser.add_argument('--objThreshold', default=0.3, type=float, help='object confidence')
    parser.add_argument('--confThreshold', default=0.3, type=float, help='class confidence')
    parser.add_argument('--nmsThreshold', default=0.4, type=float, help='nms iou thresh')
    args = parser.parse_args()
    while not global_vars.ret:
        print("wait for detect")
        global_vars.ret, img = cap.read()  # 读取摄像头
        time.sleep(1)          # 一秒一次
        pass
    if global_vars.send_img:               # 这样用就对了
        s = socket.socket()
        s.bind((HOST, PORT))
        s.listen(5)
        c, addr = s.accept()
        print('Got connection from', addr)
        pass
    # 加载模型，在另一个文件里，要是报错了，大概率是你没有把这个文件和coco.names以及model.onnx文件放到一个文件夹里面
    model = yolo_v2(objThreshold=args.objThreshold, confThreshold=args.confThreshold,
                               nmsThreshold=args.nmsThreshold)  # load model
    if global_vars.show_img:    # 给你调试用的，在自己电脑上就没必要推流去网上看了，用自己电脑看输出
        cv2.namedWindow('video', cv2.WINDOW_AUTOSIZE)
        cv2.resizeWindow('video', 352, 352)  # 设置窗口大小
        pass
    try:
        while not global_vars.finish_task:
            global_vars.ret, img = cap.read()
            # 等待键盘事件
            key = cv2.waitKey(40)  # 25帧-1000/40
            if key & 0xff == ord('q'):
                global_vars.finish_task = True
            detect_img = cv2.resize(img, (352, 352))     # 这个参数不能改，这个模型只能识别大小为352*352的图片，因此图片要变形一下
            outputs = model.detect(detect_img)  # 检测图片里面的物品
            detect_img = model.postprocess(detect_img, outputs)  # 处理图片里面的物品并且画框框
            img = cv2.resize(detect_img, (640, 480))            # 将处理完的图片重新变回我们自己的图片，方便我们看
            # 给傻子画的辅助线，防止看不懂程序还不会调试
            cv2.line(img, (0, int(global_vars.frame_height / 2)), (global_vars.frame_width, int(global_vars.frame_height / 2)), (0, 0, 0), 2)
            cv2.line(img, (int(global_vars.frame_width / 2), 0), (int(global_vars.frame_width / 2), global_vars.frame_height), (0, 0, 0), 2)
            cv2.circle(img, (int(global_vars.frame_width / 2), int(global_vars.frame_height / 2)), global_vars.find_range * 2, (0, 255, 0), 2)
            if global_vars.send_img:
                _, img_encoded = cv2.imencode('.jpg', img)  # 发送图片
                c.sendall(len(img_encoded).to_bytes(4, byteorder='big'))
                c.sendall(img_encoded.tobytes())        # 发送处理完的图片
            if global_vars.show_img:
                cv2.imshow("video", img)            # 在电脑上显示图片
        print("exit the detect")
        # 释放viedcapture，也就是关摄像头
        cap.release()
        # 销毁所有窗口
        if global_vars.show_img:
            cv2.destroyAllWindows()
        if global_vars.send_img:
            c.close()
            print(f"Connection closed.")
    except Exception as e:
        # 如果你用了网络传输图片，退出的时候一般都会报异常，不管他就行，等程序正常结束。
        print(f"出现了未知异常: {e}")
    finally:
        # 不管如何都会执行这个，先把服务器关了
        if global_vars.send_img:
            s.close()
            print("服务器已关闭！")
        print("视觉识别退出。")


if __name__ == '__main__':
   # global if_searched
    DURATION = 5
    HEADING = functhion.vehicle.heading
    HEIGHT = 3
    now_pos = [32.5,0]         # 侦察区域
    functhion.do_set_servo(2000, 5)     # 关闭舵机
    print("close servo")
    print("当前朝向为： %s\n" % HEADING)
    #functhion.condition_yaw(HEADING)
    functhion.vehicle.airspeed = 1
    frame_g = threading.Thread(target=detect_yolov2)
    frame_g.start()                             # 开始识别
    print("start camera")
    while not global_vars.ret:  # 等待摄像头初始化
        print("等待摄像头初始化...")
        time.sleep(1)
        pass
    print("camera begining---")
    print("-----START UP-----")
    functhion.arm_and_takeoff(HEIGHT)                # 起飞，高度3m
    time.sleep(DURATION)                        # 等待
    
    #advance stage 1
    print("前进5m")
    x,y = functhion.calculate_absolute_target(HEADING,now_pos[0],now_pos[1])
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
    print("当前角度为: %s:" %functhion.vehicle.heading)
    print("抵达打击目标点")
    print("start to detect")
    
    while not global_vars.finish_task:                      # 执行设定的任务
        if 61 in global_vars.img_type:                      # 如果要识别的白桶在已经识别到的物品中, 桶子61,人0
            reach = False                       # 还未到达中心
            obj_indx = global_vars.img_type.index(61)       # 找到识别的白桶的在已知物中的下标
            if global_vars.location[obj_indx][0] > global_vars.frame_width / 2 + global_vars.find_range:  # x  坐标在机头后面
                now_pos[0] += 0.1
                print(global_vars.location[obj_indx], "forward!", end= ' ')
                pass
            elif global_vars.location[obj_indx][0] < global_vars.frame_width / 2 - global_vars.find_range:  # x 坐标在机头前面
                now_pos[0] -= 0.1
                print(global_vars.location[obj_indx], "back!", end= ' ')
                pass
            else:
                reach = True
                print("前后位置已经达到")
                break
            if global_vars.location[obj_indx][1] > global_vars.frame_height / 2 + global_vars.find_range:  # x
                now_pos[1] += 0.1
                print( "turn right!")
                pass
            elif global_vars.location[obj_indx][1] < global_vars.frame_height / 2 - global_vars.find_range:
                now_pos[1] -= 0.1
                print( "turn left!")
                pass
            elif reach:
                functhion.do_set_servo(2000, 5)
                print(global_vars.location[obj_indx], "reach the center! ")
                global_vars.finish_task = True
                break
            global_vars.location = []
            global_vars.img_type = []
            x,y = functhion.calculate_absolute_target(HEADING,now_pos[0],now_pos[1])
            print("当前位置为：", now_pos,end=" ")
            functhion.goto_position_target_local_ned(x, y, -HEIGHT)
            print("当前角度为: %s:" %functhion.vehicle.heading)
            time.sleep(1)
            pass
        else :
            print("no target, start circle...........")
            flag = False
            while not flag:  #weijiancedaomubiao
                
                for i in range(4):
                    if i==0:
                        print("forward.....10cm")
                        functhion.goto_position_target_local_ned(0.1, 0, -HEIGHT)
                        time.sleep(1)
                        if search_point():
                            flag = True
                            break
                    if i==1:
                        print("backward.....20cm")
                        functhion.goto_position_target_local_ned(-0.2, 0, -HEIGHT)
                        time.sleep(1)
                        if search_point():
                            flag = True
                            break
                        
                    if i==2:
                        print("left.....10cm")
                        functhion.goto_position_target_local_ned(0.1, 0, -HEIGHT)
                        time.sleep(1)
                        functhion.goto_position_target_local_ned(0, -0.1, -HEIGHT)
                        time.sleep(1)
                        if search_point():
                            flag = True
                            break
                    if i==3:
                        print("riight.....10cm")
                        functhion.goto_position_target_local_ned(0, 0.2, -HEIGHT)
                        time.sleep(1)
                        if search_point():
                            flag = True
                            functhion.goto_position_target_local_ned(0, -0.1, -HEIGHT)
                            time.sleep()
                            break
                break
            
        time.sleep(1)
        pass
    
    print("打击完成")
    # functhion.send_global_velocity(2,0,0,10)    #抵达侦察目标点
    # print("抵达侦察目标点")
    # functhion.send_body_ned_velocity(0,-2,0,10)    #抵达侦察区域左下角

    print("前往侦察区")
    x,y = functhion.calculate_absolute_target(HEADING,55,-4)
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
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
    functhion.goto_position_target_local_ned(x, y, -HEIGHT)
    print("当前角度为: %s:" %functhion.vehicle.heading)
    for i in range(10+1,0,-1):
        time.sleep(1)
        print("倒计时：%s" % i)
        if i==1:
            print("完成")
    print("侦察完成")
    time.sleep(5)

    print("---RETURN-----")
    functhion.vehicle.mode = VehicleMode("RTL")
    time.sleep(DURATION)
    print("-----VEHICLE CLOSE-----")
    functhion.vehicle.close()


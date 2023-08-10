import cv2,argparse,socket
import time,threading
from yoloDetect import yolo_v2, global_vars

def detect_yolov2():
    cap = cv2.VideoCapture(0)    # 调用默认摄像头
    cap.set(3, global_vars.frame_width)
    cap.set(4, global_vars.frame_height)
    # 参数列表，你看不懂，也不用管，就这样用就对了
    parser = argparse.ArgumentParser()
    parser.add_argument('--objThreshold', default=0.3, type=float, help='object confidence')
    parser.add_argument('--confThreshold', default=0.3, type=float, help='class confidence')
    parser.add_argument('--nmsThreshold', default=0.4, type=float, help='nms iou thresh')
    args = parser.parse_args()

    connected = False
    while True:
        for port in global_vars.PORT:
            try:
                if global_vars.send_img:  # 这样用就对了
                    s = socket.socket()
                    s.bind((global_vars.HOST, port))
                    print(f"成功绑定到端口 {port}")
                    # 连接成功后的操作
                    print("正在等待客户端连接ing")
                    s.listen(1)
                    c, addr = s.accept()
                    print(f"成功与客户端 {addr} 建立连接")
                connected = True
                #  可以在这里添加额外的处理逻辑
                break  # 如果端口绑定成功，跳出循环
            except socket.error as e:
                if e.errno == socket.EADDRINUSE:
                    print(f"端口 {port} 被占用，尝试下一个端口...")
            time.sleep(3)  # 等待3秒后重试
        if connected:
            break

    while not global_vars.ret:
        print("wait for detect")
        global_vars.ret, img = cap.read()  # 读取摄像头
        time.sleep(1)          # 一秒一次
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
            if global_vars.send_img:
                _, img_encoded = cv2.imencode('.jpg', img)  # 发送原始图片
                c.sendall(len(img_encoded).to_bytes(4, byteorder='big'))
                c.sendall(img_encoded.tobytes())
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


def main():
    frame_g = threading.Thread(target=detect_yolov2)
    frame_g.start()                             # 开始识别


main()
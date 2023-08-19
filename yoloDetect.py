import argparse
import socket
import time
import cv2,os
import numpy as np
from  functhion import vehicle

class GlobalVariables:      # 全局变量类，用来管理和在文件间传递全局变量
    def __init__(self):
        self.location = []
        self.img_type = []
        self.finish_task = False            # 飞行任务是否完成
        self.frame_width = 720       # 窗口大小
        self.frame_height = 720
        self.frame_width_detect = 640       # 检测窗口大小
        self.frame_height_detect = 480
        self.center = (self.frame_width_detect / 2 , self.frame_height_detect / 2)
        self.ret = False             # 摄像头打开是否成功
        
        #setable_var
        self.show_img = False        # 是否显示图形化界面，用于调试，自己看设为True   
        self.send_img = True        # 是否向互联网发送图片                        
        self.find_range = 50         # 检测边界（中心圆的半径）    
        self.HOST = "192.168.0.100"       #host地址
        self.PORT = [5000, 5025, 5050]    #端口组
        self.CAM = 0        #相机编号

        self.DETECT = 61    #识别物体
        self.VEL = 1        #飞行速度
        self.HEIGHT = 3        #飞行高度
        self.HEADING = vehicle.heading    #飞行朝向
        self.run = False
        self.found_obj = False




# 创建单例对象，全局变量
global_vars = GlobalVariables()


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

def get_model_path(name):
    current_directory = os.getcwd()
    names_path = os.path.join(current_directory,"model",name) 
    return names_path

class yolo_v2():
    def __init__(self, objThreshold=0.3, confThreshold=0.3, nmsThreshold=0.4):
        with open(get_model_path('coco.names'), 'rt') as f:
            self.classes = f.read().rstrip('\n').split(
                '\n')  ###这个是在coco数据集上训练的模型做opencv部署的，如果你在自己的数据集上训练出的模型做opencv部署，那么需要修改self.classes
        self.stride = [16, 32]
        self.anchor_num = 3
        self.anchors = np.array(
            [12.64, 19.39, 37.88, 51.48, 55.71, 138.31, 126.91, 78.23, 131.57, 214.55, 279.92, 258.87],
            dtype=np.float32).reshape(len(self.stride), self.anchor_num, 2)
        self.inpWidth = 352
        self.inpHeight = 352
        self.net = cv2.dnn.readNet(get_model_path('model.onnx'))
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold
        self.objThreshold = objThreshold

    def _make_grid(self, nx=20, ny=20):
        xv, yv = np.meshgrid(np.arange(ny), np.arange(nx))
        return np.stack((xv, yv), 2).reshape((-1, 2)).astype(np.float32)

    def map_range(self, value, in_min, in_max, out_min, out_max):
        # 输入范围的宽度
        in_range = in_max - in_min
        # 输出范围的宽度
        out_range = out_max - out_min
        # 将输入值映射到[0, 1]范围内
        normalized_value = (value - in_min) / in_range
        # 映射到输出范围
        mapped_value = out_min + (normalized_value * out_range)
        # 返回映射后的值
        return int(mapped_value)

    def postprocess(self, frame, outs):
        global global_vars
        frameHeight = frame.shape[0]
        frameWidth = frame.shape[1]
        ratioh, ratiow = frameHeight / self.inpHeight, frameWidth / self.inpWidth
        # Scan through all the bounding boxes output from the network and keep only the
        # ones with high confidence scores. Assign the box's class label as the class with the highest score.
        classIds = []
        confidences = []
        boxes = []
        for detection in outs:
            scores = detection[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > self.confThreshold and detection[4] > self.objThreshold:
                center_x = int(detection[0] * ratiow)
                center_y = int(detection[1] * ratioh)
                # 获取识别物体的中心坐标
                global_vars.location.append([self.map_range(center_x, 0, 352, 0, 640), self.map_range(center_y, 0, 352, 0, 480)])
                width = int(detection[2] * ratiow)
                height = int(detection[3] * ratioh)
                left = int(center_x - width / 2)
                top = int(center_y - height / 2)
                classIds.append(classId)
                # confidences.append(float(confidence))
                confidences.append(float(confidence * detection[4]))
                boxes.append([left, top, width, height])
        global_vars.img_type = classIds  # 获取所有的类别
        if 61 in global_vars.img_type: 
            global_vars.found_obj = True
        # Perform non maximum suppression to eliminate redundant overlapping boxes with
        # lower confidences.
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.confThreshold, self.nmsThreshold)
        for i in indices:
            # i = i[0]
            box = boxes[i]
            left = box[0]
            top = box[1]
            width = box[2]
            height = box[3]
            frame = self.drawPred(frame, classIds[i], confidences[i], left, top, left + width, top + height)
        return frame

    def drawPred(self, frame, classId, conf, left, top, right, bottom):
        # Draw a bounding box.
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), thickness=2)
        # print('conf',conf)
        label = '%.2f' % conf
        label = '%s:%s' % (self.classes[classId], label)

        # Display the label at the top of the bounding box
        labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(top, labelSize[1])
        # cv.rectangle(frame, (left, top - round(1.5 * labelSize[1])), (left + round(1.5 * labelSize[0]), top + baseLine), (255,255,255), cv.FILLED)
        cv2.putText(frame, label, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), thickness=1)
        return frame

    def detect(self, srcimg):
        blob = cv2.dnn.blobFromImage(srcimg, 1 / 255.0, (self.inpWidth, self.inpHeight))
        self.net.setInput(blob)
        outs = self.net.forward(self.net.getUnconnectedOutLayersNames())[0]

        outputs = np.zeros((outs.shape[0] * self.anchor_num, 5 + len(self.classes)))
        row_ind = 0
        for i in range(len(self.stride)):
            h, w = int(self.inpHeight / self.stride[i]), int(self.inpWidth / self.stride[i])
            length = int(h * w)
            grid = self._make_grid(w, h)
            for j in range(self.anchor_num):
                top = row_ind + j * length
                left = 4 * j
                outputs[top:top + length, 0:2] = (outs[row_ind:row_ind + length,
                                                  left:left + 2] * 2. - 0.5 + grid) * int(self.stride[i])
                outputs[top:top + length, 2:4] = (outs[row_ind:row_ind + length,
                                                  left + 2:left + 4] * 2) ** 2 * np.repeat(
                    self.anchors[i, j, :].reshape(1, -1), h * w, axis=0)
                outputs[top:top + length, 4] = outs[row_ind:row_ind + length, 4 * self.anchor_num + j]
                outputs[top:top + length, 5:] = outs[row_ind:row_ind + length, 5 * self.anchor_num:]
            row_ind += length
        return outputs

import cv2
import numpy as np


class GlobalVariables:      # 全局变量类，用来管理和在文件间传递全局变量
    def __init__(self):
        self.location = []
        self.img_type = []
        self.finish_task = False            # 飞行任务是否完成
        self.frame_width = 640       # 窗口大小
        self.frame_height = 480
        self.ret = False             # 摄像头打开是否成功
        self.show_img = False        # 是否显示图形化界面，用于调试，自己看设为True   #setable
        self.send_img = True        # 是否向互联网发送图片                        ##setable
        self.find_range = 50         # 检测边界（中心圆的半径）

# 创建单例对象，全局变量
global_vars = GlobalVariables()


class yolo_v2():
    def __init__(self, objThreshold=0.3, confThreshold=0.3, nmsThreshold=0.4):
        with open('/home/syujie/drone/yoloproject/coco.names', 'rt') as f:
            self.classes = f.read().rstrip('\n').split(
                '\n')  ###这个是在coco数据集上训练的模型做opencv部署的，如果你在自己的数据集上训练出的模型做opencv部署，那么需要修改self.classes
        self.stride = [16, 32]
        self.anchor_num = 3
        self.anchors = np.array(
            [12.64, 19.39, 37.88, 51.48, 55.71, 138.31, 126.91, 78.23, 131.57, 214.55, 279.92, 258.87],
            dtype=np.float32).reshape(len(self.stride), self.anchor_num, 2)
        self.inpWidth = 352
        self.inpHeight = 352
        self.net = cv2.dnn.readNet('/home/syujie/drone/yoloproject/model.onnx')
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

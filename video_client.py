import socket
import time

import cv2
import numpy as np


# 获取当前机器的主机名
#host = socket.gethostbyname("localhost")
host = "127.0.0.1"
# 指定监听的端口号
ports = [5000, 5025, 5050]

connected = False

while True:
    for port in ports:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((host, port))
            print(f"成功连接到服务器的端口 {port}")
            # 连接成功后的操作
            connected = True
            break  # 如果连接成功，跳出循环
        except ConnectionRefusedError:
            print(f"连接到端口 {port} 失败，尝试下一个端口...")
        time.sleep(1)  # 等待1秒后重试
    if connected:
        break

# 在这里可以进行连接成功后的其他操作
img_num = 0


cv2.namedWindow('Raw Frame', cv2.WINDOW_AUTOSIZE)
cv2.resizeWindow('Raw Frame', 1280, 720)  # 设置窗口大小
cv2.namedWindow('Detect Frame', cv2.WINDOW_AUTOSIZE)
cv2.resizeWindow('Detect Frame', 640, 480)  # 设置窗口大小
while True:
    # 接收帧长度
    length = s.recv(4)
    frame_length = int.from_bytes(length, byteorder='big')

    # 接收帧数据
    data = b""
    while len(data) < frame_length:
        packet = s.recv(min(4096, frame_length - len(data)))
        if not packet:
            break
        data += packet

    # 解码帧数据
    frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if (img_num % 2) == 0:
        # 在窗口中显示帧
        cv2.imshow('Raw Frame', frame)
    else:
        # 在窗口中显示帧
        cv2.imshow('Detect Frame', frame)
    img_num += 1
    img_num %= 2
    # 当按下 'q' 键时退出循环
    if cv2.waitKey(1) == ord('q'):
        break

# 关闭窗口和socket连接
cv2.destroyAllWindows()
s.close()

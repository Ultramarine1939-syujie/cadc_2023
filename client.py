import socket
import cv2
import numpy as np

# 创建一个新的 socket 实例
s = socket.socket()

# 获取当前机器的主机名
#host = socket.gethostbyname("localhost")
host = "127.0.0.1"
# 指定监听的端口号
port = 5000

# 连接到服务器
s.connect((host, port))

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

    # 在窗口中显示帧
    cv2.imshow('Received Frame', frame)

    # 当按下 'q' 键时退出循环
    if cv2.waitKey(1) == ord('q'):
        break

# 关闭窗口和socket连接
cv2.destroyAllWindows()
s.close()

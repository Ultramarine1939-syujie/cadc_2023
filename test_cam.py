import cv2

def take_photo(file_path):
    # 打开摄像头
    cap = cv2.VideoCapture(0)

    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    # 读取摄像头画面
    ret, frame = cap.read()

    # 保存图像到文件
    if ret:
        cv2.imwrite(file_path, frame)
        print("已保存照片：", file_path)
    else:
        print("无法读取摄像头画面")

    # 关闭摄像头
    cap.release()

# 调用拍照函数
take_photo("photo.jpg")


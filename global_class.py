

class GlobalVariables:      # 全局变量类，用来管理和在文件间传递全局变量
    def __init__(self):
        self.location = []
        self.img_type = []
        self.finish_task = False            # 飞行任务是否完成
        self.frame_width = 640       # 窗口大小
        self.frame_height = 480
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
        self.HEADING = 0    #飞行朝向
        self.run = False
        self.found_obj = False




# 创建单例对象，全局变量
global_vars = GlobalVariables()
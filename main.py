import uptech
import time
import apriltag
import cv2
import sys
import numpy as np
import signal
import threading
import subprocess

flag = 0
is_tag = 0
mid = 0
tag_width = 0
tags = []
distance = 0
index = 0
frame = 0

di_fang_kuai = 1  # 敌方块
zhong_li_kuai = 0  # 中立块
zha_dan_kuai = 2  # 炸弹块

out = 0
zhuan = 0
io0 = 6
io1 = 6
io2 = 6
io3 = 6
io4 = 6
io5 = 6
adc1 = 0
adc2 = 0
adc0 = 0
adc_average = 0
io_data = [0]
adc_value = [0]

class ApriltagDetect:
    def __init__(self):
        self.target_id = 0
        self.at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11 tag25h9'))

    def update_frame(self, frame):
        h0 = 0  # shi fou you 0 ma
        h1 = 0  # shi fou you 1 ma
        h2 = 0
        m1 = 0  # zui zhong xin 1 ma zhong xin zuo biao
        m0 = 0  # zui zhong xin 0 ma zhong xin zuo biao
        m2 = 0
        mx0 = 0
        mx1 = 0  # ma zhi zhong xin zuo biao
        mx2 = 0
        mid0 = 0
        mid1 = 0
        mid2 = 0
        global is_tag
        global index
        global flag
        global mid
        global tag_width
        global tags
        global distance
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(gray)
        flag = 0
        index = 0
        if tags:
            flag = 1  # 这是个标志位
            for i in range(1, len(tags)):
                # 循环从第二个（results[1]）索引开始，进行冒泡排序。（因为前方index是从零开始的）所以排序没有遗漏
                if tags[i].tag_id == di_fang_kuai or tags[i].tag_id == zhong_li_kuai:
                    # 如果tag码块id是敌方或中立才进行距离比较，炸弹不管
                    if tags[index].tag_id == zha_dan_kuai:
                        # 这一步确保index=0的那个id不是炸弹，若是炸弹，则将索引就改成i（i现在肯定不是炸弹）
                        index = i
                    if tags[i].tag_id == zhong_li_kuai:
                        if tags[index].tag_id == zhong_li_kuai:
                            if (self.get_distance(tags[index].homography, 4300) >
                                    self.get_distance(tags[i].homography, 4300)):  # 冒泡排序
                                index = i
                    elif tags[i].tag_id == di_fang_kuai:
                        if tags[index].tag_id == di_fang_kuai:
                            if (self.get_distance(tags[index].homography, 4300) >
                                    self.get_distance(tags[i].homography, 4300)):  # 冒泡排序
                                index = i
                        elif tags[index].tag_id == zhong_li_kuai:
                            index = i
            if tags[index].tag_id == di_fang_kuai or tags[index].tag_id == zhong_li_kuai:  # 冒泡后如果最近的id是中立或敌方
                is_tag = 1
            else:  # 冒泡后如果的id是炸弹块(侧面证明了没有检测到敌方和中立)
                is_tag = 0
            distance = int(self.get_distance(tags[index].homography, 4300))
            mid = tuple(tags[index].corners[0].astype(int))[0] / 2 + \
                  tuple(tags[index].corners[2].astype(int))[0] / 2  # 计算tag的横向位置
            tag_width = abs(tuple(tags[index].corners[0].astype(int))[0] - tuple(tags[index].corners[2].astype(int))[0])
            # print(taps1, tags[index].tag_id)
        else:
            flag = 0

    def get_distance(self, H, t):
        """
        :param H: homography matrix
        :param t: ???
        :return: distance
        """
        ss = 0.5
        src = np.array([[-ss, -ss, 0],
                        [ss, -ss, 0],
                        [ss, ss, 0],
                        [-ss, ss, 0]])
        Kmat = np.array([[700, 0, 0],
                         [0, 700, 0],
                         [0, 0, 1]]) * 1.0
        disCoeffs = np.zeros([4, 1]) * 1.0
        ipoints = np.array([[-1, -1],
                            [1, -1],
                            [1, 1],
                            [-1, 1]])
        for point in ipoints:
            x = point[0]
            y = point[1]
            z = H[2, 0] * x + H[2, 1] * y + H[2, 2]
            point[0] = (H[0, 0] * x + H[0, 1] * y + H[0, 2]) / z * 1.0
            point[1] = (H[1, 0] * x + H[1, 1] * y + H[1, 2]) / z * 1.0
        campoint = ipoints * 1.0
        opoints = np.array([[-1.0, -1.0, 0.0],
                            [1.0, -1.0, 0.0],
                            [1.0, 1.0, 0.0],
                            [-1.0, 1.0, 0.0]])
        opoints = opoints * 0.5
        rate, rvec, tvec = cv2.solvePnP(opoints, campoint, Kmat, disCoeffs)
        point, jac = cv2.projectPoints(src, np.zeros(rvec.shape), tvec, Kmat, disCoeffs)
        points = np.int32(np.reshape(point, [4, 2]))
        distance = np.abs(t / np.linalg.norm(points[0] - points[1]))
        return distance

def tag_solve1():
    if 100 < distance < 200:
        if mid < 160 - tag_width / 4:
            up.CDS_SetSpeed(1, 350)  # 右轮正转  450
            up.CDS_SetSpeed(2, -350)       # 左轮反转 -450
            time.sleep(0.02)
            print('zuo')
        elif mid > 150 + tag_width / 2:
            up.CDS_SetSpeed(1, -350)       # 右轮正转  450
            up.CDS_SetSpeed(2, 350)  # 左轮反转 -450
            time.sleep(0.02)
            print("you")
        else:
            go_straight()
            print("对准")
        print(distance)
    elif 50 < distance <= 100:
        if mid < 180 - tag_width / 4:
            up.CDS_SetSpeed(1, 360)  # 右轮正转  450
            up.CDS_SetSpeed(2, -360)       # 左轮反转 -450
            time.sleep(0.02)
            print('zuo')
        elif mid > 160 + tag_width / 4:
            up.CDS_SetSpeed(1, -360)  # 右轮正转  450
            up.CDS_SetSpeed(2, 360)  # 左轮反转 -450
            time.sleep(0.02)
            print("you")
        else:
            go_straight()
            print("对准")
        print(distance)
    elif distance <= 90 and out == 1:
        go_straight()

def April_start_detect():
    global frame
    global flag
    global is_tag
    cap = cv2.VideoCapture(0)  #'/dev/video0'
    ad = ApriltagDetect()
    cap.set(3, 320)
    cap.set(4, 240)

    while True:
        ret, frame = cap.read()
        if ret is False:
            cap.release()
            time.sleep(0.1)
            print("reconnect to camera")
            subprocess.check_call("sudo modprobe -rf uvcvideo", shell=True)
            time.sleep(0.4)
            subprocess.check_call("sudo modprobe uvcvideo", shell=True)
            time.sleep(0.2)
            cap = cv2.VideoCapture(0)
        # frame = cv2.rotate(frame, cv2.ROTATE_180)
        ad.update_frame(frame)
        # if tags:
        #     if is_tag == 0:  #炸弹
        #         print("炸弹")
        #         print(f'distance:{distance}')
        #         print(f'mid:{mid}')
        #         print(f'tag_width:{tag_width}')
        #     else:
        #         if tags[index].tag_id == 1:  #敌方
        #             print("敌方")
        #             print(f'distance:{distance}')
        #             print(f'mid:{mid}')
        #             print(f'tag_width:{tag_width}')
        #         elif tags[index].tag_id == 0:  #中立
        #             print("中立")
        #             print(f'distance:{distance}')
        #             print(f'mid:{mid}')
        #             print(f'tag_width:{tag_width}')
        cv2.imshow("img", frame)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break

        up_platform_act()

    cap.release()
    cv2.destroyAllWindows()

def go_straight():
    up.CDS_SetSpeed(1, 400)  #右轮正转
    up.CDS_SetSpeed(2, 400)  #左轮正转

def turn_right():
    up.CDS_SetSpeed(1, -500)       #右轮反转 -450
    up.CDS_SetSpeed(2, 500)  #左轮正转  450

def turn_left():
    up.CDS_SetSpeed(1, 500)  #右轮正转  450
    up.CDS_SetSpeed(2, -500)       #左轮反转 -450

def go_back():
    up.CDS_SetSpeed(1, -500)  # 右轮正转
    up.CDS_SetSpeed(2, -500)  # 左轮正转

def stop():
    up.CDS_SetSpeed(1, 0)
    up.CDS_SetSpeed(2, 0)

def down_platform_detect():
        if io_data[1] == 0 and io_data[2] == 0:
            up.CDS_SetSpeed(1, 0)
            up.CDS_SetSpeed(2, 0)
            up.CDS_SetAngle(3, 535, 712)
            up.CDS_SetAngle(4, 378, 712)
            time.sleep(0.5)
            down_platform_act()
        elif io_data[2] == 0 and io_data[3] == 1:
            up.CDS_SetSpeed(1, 450) #左转
            up.CDS_SetSpeed(2, -450)
            up.CDS_SetAngle(3, 535, 712)
            up.CDS_SetAngle(4, 378, 712)
        elif io_data[2] == 1 and io_data[3] == 0:
            up.CDS_SetSpeed(1, -450) #右转
            up.CDS_SetSpeed(2, 450)
            up.CDS_SetAngle(3, 535, 712)
            up.CDS_SetAngle(4, 378, 712)
        else:
            up.CDS_SetSpeed(1, 450)
            up.CDS_SetSpeed(2, -450)
            up.CDS_SetAngle(3, 535, 712)
            up.CDS_SetAngle(4, 378, 712)

def down_platform_act():
    up.CDS_SetSpeed(1, -800)  #右轮反转
    up.CDS_SetSpeed(2, -800)  #左轮反转
    time.sleep(0.5)
    up.CDS_SetAngle(3, 320, 512)
    up.CDS_SetAngle(4, 587, 512)
    time.sleep(0.3)
    up.CDS_SetAngle(3, 639, 712)
    up.CDS_SetAngle(4, 302, 712)
    time.sleep(0.3)

def up_platform_act():
    up.CDS_SetAngle(3, 535, 512)
    up.CDS_SetAngle(4, 378, 512)
    if zhuan == 0: #最外层if-elif控制边界
        go_straight()

        # if flag == 0:
        #     if io_data[1] == 0 and io_data[2] == 0: #次外层if-elif控制箱子
        #         if io_data[0] != 1 and io_data[3] != 1: #最内层if-elif控制侧面
        #             go_straight()
        if flag == 1:
            if is_tag == 1:
                if zhuan != 0:
                    tag_solve1()
            elif is_tag == 0:
                stop()

        # elif io_data[1] == 1 and io_data[2] == 1:
        #     if io_data[4] == 0 and io_data[5] == 1:
        #         turn_left()
        #         time.sleep(0.3)
        #     elif io_data[4] == 1 and io_data[5] == 0:
        #         turn_right()
        #         time.sleep(0.3)
        #     elif io_data[4] == 0 and io_data[5] == 0:
        #         turn_right()
        #         time.sleep(0.3)
        #     elif io_data[4] == 1 and io_data[5] == 1:
        #         go_straight()
        # elif io_data[1] == 0 and io_data[2] == 1:
        #     turn_left()
        # elif io_data[1] == 1 and io_data[2] == 0:
        #     turn_right()
    elif zhuan == 2:
        go_back()
        time.sleep(0.1)
        turn_left()
        time.sleep(0.1)
    elif zhuan == 1:
        go_back()
        time.sleep(0.1)
        turn_right()
        time.sleep(0.1)
    elif zhuan == 3:
        go_back()
        time.sleep(0.3)
        turn_left()
        time.sleep(0.1)

def adio_start_detect():
    global out
    global adc_value
    global io5
    global io6
    global io3
    global io2
    global io1
    global io0
    global adc_0
    global adc_1
    global adc_2
    global adc_average
    global io_data
    global flag
    global zhuan

    while True:
        adc_value = up.ADC_Get_All_Channle()
        adc0 = adc_value[0]
        adc1 = adc_value[1]
        adc2 = adc_value[2]
        adc_average = (adc0 + adc1 + adc2) / 3

        io_all_input = up.ADC_IO_GetAllInputLevel()
        io_array = '{:08b}'.format(io_all_input)
        io_data.clear()
        for index, value in enumerate(io_array):
            io = int(value)
            io_data.insert(0, io)

        io0 = io_data[0]
        io1 = io_data[1]
        io2 = io_data[2]
        io3 = io_data[3]
        io4 = io_data[4]
        io5 = io_data[5]

        if io1 == 0 and io2 == 0:
            out = 1
        elif io1 == 1 and io2 == 0:
            out = 2
        elif io1 == 0 and io2 == 1:
            out = 3
        elif io1 == 1 and io2 == 1:
            out = 4

        if io4 == 0 and io5 == 0:
            out = 5

        if io0 == 0 and io3 == 0:
            zhuan = 0  # 前进
        elif io0 == 1 and io3 == 0:
            zhuan = 1  # 右转
        elif io0 == 0 and io3 == 1:
            zhuan = 2  # 左转
        elif io0 == 1 and io3 == 1:
            zhuan = 3  # 后退

def signal_handler(handler_signal, handler_frame):
    stop()
    exit(0)

if __name__ == "__main__":
    up = uptech.UpTech()
    up.LCD_Open(2)
    up.ADC_IO_Open()
    up.ADC_Led_SetColor(0, 0x2F0000) #2F0000
    up.ADC_Led_SetColor(1, 0x002F00) #002F00
    up.CDS_Open()
    up.CDS_SetMode(1, 1)
    up.CDS_SetMode(2, 1)
    up.CDS_SetMode(3, 0)
    up.CDS_SetMode(4, 0)
    up.LCD_PutString(40, 0, 'formal')
    up.LCD_Refresh()
    up.LCD_SetFont(up.FONT_6X10)
    signal.signal(signal.SIGINT, signal_handler)
    target = threading.Thread(target=adio_start_detect)
    target.start()
    cap = cv2.VideoCapture('/dev/video0')
    ad = ApriltagDetect()
    cap.set(3, 320)
    cap.set(4, 240)

    while True:
        if out == 5:
            break

    while True:
        ret, frame = cap.read()
        if ret is False:
            cap.release()
            time.sleep(0.1)
            print("reconnect to camera")
            subprocess.check_call("sudo modprobe -rf uvcvideo", shell=True)
            time.sleep(0.4)
            subprocess.check_call("sudo modprobe uvcvideo", shell=True)
            time.sleep(0.2)
            cap = cv2.VideoCapture('/dev/video0')
        # frame = cv2.rotate(frame, cv2.ROTATE_180)
        ad.update_frame(frame)
        # if tags:
        #     if is_tag == 0:  #炸弹
        #         print("炸弹")
        #         print(f'distance:{distance}')
        #         print(f'mid:{mid}')
        #         print(f'tag_width:{tag_width}')
        #     else:
        #         if tags[index].tag_id == 1:  #敌方
        #             print("敌方")
        #             print(f'distance:{distance}')
        #             print(f'mid:{mid}')
        #             print(f'tag_width:{tag_width}')
        #         elif tags[index].tag_id == 0:  #中立
        #             print("中立")
        #             print(f'distance:{distance}')
        #             print(f'mid:{mid}')
        #             print(f'tag_width:{tag_width}')
        cv2.imshow("img", frame)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break

        # up_platform_act()
        tag_solve1()

    cap.release()
    cv2.destroyAllWindows()


    # while True:
    #     adc_value = up.ADC_Get_All_Channle()
    #     io_all_input = up.ADC_IO_GetAllInputLevel()
    #     io_array = '{:08b}'.format(io_all_input)
    #     io_data.clear()
    #
    #     for index, value in enumerate(io_array):
    #         io = (int)(value)
    #         io_data.insert(0, io)
    #     if io_data[4] == 0 and io_data[5] == 0:
    #         break
    #
    # while True:
    #     adc_value = up.ADC_Get_All_Channle()
    #     io_all_input = up.ADC_IO_GetAllInputLevel()
    #     io_array = '{:08b}'.format(io_all_input)
    #     io_data.clear()
    #
    #     for index, value in enumerate(io_array):
    #         io = (int)(value)
    #         io_data.insert(0, io)
    #
    #     average_adc = (adc_value[0] + adc_value[1] + adc_value[2]) / 3




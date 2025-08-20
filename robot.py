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

#蓝方
di_fang_kuai = 2             # 敌方块2
zhong_li_kuai = 0            # 中立块0
zha_dan_kuai = 1             # 炸弹块1

#黄方
# di_fang_kuai = 1             # 敌方块1
# zhong_li_kuai = 0            # 中立块0
# zha_dan_kuai = 2             # 炸弹块2

adc_value = [0]
adc_average3 = 0
adc_average4 = 0
adc_average2 = 0
io_data = [0]

camera_reset = 0
camera_reload = 1
camera_safe = 0

first_flag = 0
io_flag = 0
adc_flag = 0

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

def April_start_detect():
    global frame
    global camera_reset
    global camera_reload
    global camera_safe
    cap = cv2.VideoCapture(0)
    cap.set(3, 320)
    cap.set(4, 240)
    cap.set(cv2.CAP_PROP_FPS, 60)
    ad = ApriltagDetect()

    while True:
        ret, frame = cap.read()
        if camera_reset:
            cap.set(3, 320)
            cap.set(4, 240)
            cap.set(cv2.CAP_PROP_FPS, 60)
            time.sleep(0.5)
            camera_reset = 0
        # if camera_reload:
        #     camera_reload = 0
        #     ret = 0
        if not ret or frame is None:
            print("摄像头断开连接")
            camera_safe = 0
            cap.release()
            time.sleep(0.1)
            print("正在尝试重连")
            subprocess.check_call("sudo modprobe -rf uvcvideo", shell=True)
            time.sleep(0.4)
            subprocess.check_call("sudo modprobe uvcvideo", shell=True)
            time.sleep(0.2)
            cap = cv2.VideoCapture(0)
            camera_reset = 1
            continue
        else:
            camera_safe = 1
        # frame = cv2.rotate(frame, cv2.ROTATE_180)
        ad.update_frame(frame)
        # cv2.imshow("img", frame)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break
        # print("camera succeed")
    cap.release()
    cv2.destroyAllWindows()

def get_adio_data():
    global io_data
    global adc_value
    global adc_average3
    global adc_average4
    global adc_average2
    adc_value = up.ADC_Get_All_Channle()
    adc_average3 = (adc_value[0] + adc_value[1] + adc_value[2]) / 3
    adc_average4 = (adc_value[0] + adc_value[1] + adc_value[2] + adc_value[3]) / 4
    adc_average2 = (adc_value[0] + adc_value[1]) / 2


    io_all_input = up.ADC_IO_GetAllInputLevel()
    io_array = '{:08b}'.format(io_all_input)
    io_data.clear()
    for index, value in enumerate(io_array):
        io = (int)(value)
        io_data.insert(0, io)

#左边往前为+ 右边往前为-

def go_straight():
    global adc_average2
    global adc_average3
    global adc_average4
    if adc_average4 > 1100: #2100
        up.CDS_SetSpeed(1, 1000)  #600
        up.CDS_SetSpeed(2, -1000)
    # elif adc_average > 1600:
    #     up.CDS_SetSpeed(1, 800)  # 600
    #     up.CDS_SetSpeed(2, -800)  #
    else:
        up.CDS_SetSpeed(1, 500)  #500
        up.CDS_SetSpeed(2, -500)

def go_straight_slow():
    up.CDS_SetSpeed(1, 500)  #右前左后
    up.CDS_SetSpeed(2, -500)

def turn_left():
    up.CDS_SetSpeed(1, -800)   #700
    up.CDS_SetSpeed(2, -800)   #700

def turn_left_IO():
    up.CDS_SetSpeed(1, -600)   #700
    up.CDS_SetSpeed(2, -600)   #700

def turn_left_left():
    up.CDS_SetSpeed(1, -800)   #700
    up.CDS_SetSpeed(2, -800)        #700

def turn_left_tag():
    up.CDS_SetSpeed(1, -400)
    up.CDS_SetSpeed(2, -400)

def turn_left_bug():
    up.CDS_SetSpeed(1, -700)   #700
    up.CDS_SetSpeed(2, -700)        #700

def turn_left_down():
    up.CDS_SetSpeed(1, -800)   #700
    up.CDS_SetSpeed(2, -800)        #700

def turn_right():
    up.CDS_SetSpeed(1, 800)          # 700
    up.CDS_SetSpeed(2, 800)     # 700

def turn_right_IO():
    up.CDS_SetSpeed(1, 600)          # 700
    up.CDS_SetSpeed(2, 600)     # 700

def turn_right_right():
    up.CDS_SetSpeed(1, 800)          # 600
    up.CDS_SetSpeed(2, 800)     # 600

def turn_right_tag():
    up.CDS_SetSpeed(1, 400)
    up.CDS_SetSpeed(2, 400)

def turn_right_down():
    up.CDS_SetSpeed(1, 800)          # 700
    up.CDS_SetSpeed(2, 800)     # 700

def go_back():
    up.CDS_SetSpeed(1, -700)
    up.CDS_SetSpeed(2, 700)

def go_back_platform():
    up.CDS_SetSpeed(1, -500)
    up.CDS_SetSpeed(2, 500)

def go_back_tag():
    up.CDS_SetSpeed(1, -600)
    up.CDS_SetSpeed(2, 600)

def down_back():
    up.CDS_SetSpeed(1, -1000)
    up.CDS_SetSpeed(2, 1000)

def back_demo():
    up.CDS_SetSpeed(1, -800)
    up.CDS_SetSpeed(2, 800)

def stop():
    up.CDS_SetSpeed(1, 0)
    up.CDS_SetSpeed(2, 0)

def set_angle_up():
    up.CDS_SetAngle(3, 698, 512)
    up.CDS_SetAngle(4, 320, 512)

def set_angle_mid():
    up.CDS_SetAngle(3, 436, 512)
    up.CDS_SetAngle(4, 587, 512)

def set_angle_down():
    up.CDS_SetAngle(3, 326, 512)
    up.CDS_SetAngle(4, 703, 512)

def set_angle_platform():
    up.CDS_SetAngle(3, 849, 512)  # 3号 上面698 平地442 底下326  台下抬高849
    up.CDS_SetAngle(4, 198, 512)  # 4号 上面320 平地552 底下703  台下抬高198


def down_platform_detect():
    global io_data
    global first_flag
    if io_data[1] == 0 and io_data[7] == 0:
        # if first_flag == 0:
        #     stop()
        #     time.sleep(0.5)
        #     # go_back_platform()
        #     # time.sleep(0.2)  # 0.2
        #     # stop()
        #     # time.sleep(0.5)
        #     down_back()
        #     time.sleep(2)  # 1.0
        #     turn_left()
        #     time.sleep(0.2)
        #     first_flag = 1
        # else:
        #     stop()
        #     time.sleep(0.5)
        #     go_back_platform()
        #     time.sleep(1) #0.2
        #     stop()
        #     time.sleep(0.5)
        #     go_straight_slow()
        #     time.sleep(0.3)
        #     stop()
        #     time.sleep(0.5)
        #     down_back()
        #     time.sleep(1)  #1.0
        #     turn_left()
        #     time.sleep(0.2)

        stop()
        time.sleep(0.5)
        go_back_platform()
        time.sleep(1.5)    # 0.6 0.8 1
        stop()
        time.sleep(0.5)
        go_straight_slow()
        time.sleep(0.4)  # 0.45
        stop()
        time.sleep(0.5)
        back_demo()
        time.sleep(1)    # 1.0
        turn_left()
        time.sleep(0.2)
    elif io_data[1] == 0 and io_data[7] == 1:
        turn_left_down()
    elif io_data[1] == 1 and io_data[7] == 0:
        turn_right_down()
    else:
        turn_right_down()

def tag_solve():
    global is_tag
    global distance
    global mid
    global tag_width
    global io_data
    # print(distance)
    if is_tag == 1:
        if distance > 150:
            if mid < 150 - tag_width / 3:
                turn_left_tag()
                # time.sleep(0.01)
                # print('zuo')
            elif mid > 170 + tag_width / 3:  #
                turn_right_tag()
                # time.sleep(0.01)
                # print("you")
            else:
                go_straight()
                # print("对准")
        elif distance <= 160:
            if io_data[1] == 0 and io_data[7] == 0:
                go_straight()
            elif io_data[1] == 0 and io_data[7] == 1:
                turn_left_tag()
            elif io_data[1] == 1 and io_data[7] == 0:
                turn_right_tag()
    if is_tag == 0:
        if distance < 200:  #160
            go_back_tag()   #go_back()
            time.sleep(0.4)
            # turn_left_bug()
            # time.sleep(0.5) #0.2
            turn_left()
            time.sleep(0.5) #0.3

def up_platform_act():
    global io_data
    global is_tag
    global flag
    global adc_average
    if io_data[0] == 0 and io_data[3] == 0: #最外层if-elif控制边界
        go_straight()
        # if adc_value[6] < 10 and io_data[7] == 0:
        #     go_straight()
        # elif adc_value[6] < 10 and io_data[7] == 1:
        #     go_back()
        #     time.sleep(0.2)
        #     turn_left()
        #     time.sleep(0.3)
        # elif adc_value[6] > 10 and io_data[7] == 0:
        #     go_back()
        #     time.sleep(0.2)
        #     turn_right()
        #     time.sleep(0.3)
        # elif adc_value[6] > 10 and io_data[7] == 1:
        #     go_back()
        #     time.sleep(0.2)
        #     turn_left()
        #     time.sleep(0.3)
        if flag == 1:
            tag_solve()
        elif flag == 0:  # flag == 0 or is_tag == 1
            if io_data[1] == 0 and io_data[7] == 0:  # 次外层if-elif控制箱子
                if io_data[0] != 1 and io_data[3] != 1:  #最内层if-elif控制侧面
                    go_straight()
            elif io_data[1] == 1 and io_data[7] == 1:
                if io_data[4] == 0 and io_data[5] == 1:
                    turn_left_left()
                    time.sleep(0.4)
                elif io_data[4] == 1 and io_data[5] == 0:
                    turn_right_right()
                    time.sleep(0.4)
                elif io_data[4] == 0 and io_data[5] == 0:
                    turn_right_right()
                    time.sleep(0.4)
                elif io_data[4] == 1 and io_data[5] == 1:
                    go_straight()
            elif io_data[1] == 0 and io_data[7] == 1:
                turn_left_IO()
            elif io_data[1] == 1 and io_data[7] == 0:
                turn_right_IO()
    elif io_data[0] == 0 and io_data[3] == 1:
        if flag == 0:
            go_back()
            time.sleep(0.2)
            turn_left()
            time.sleep(0.3)  #0.3
        elif flag == 1:
            if is_tag == 0:
                go_back()
                time.sleep(0.2)
                turn_left()
                time.sleep(0.3)  # 0.3
            elif is_tag == 1:
                go_back()
                time.sleep(0.2)
                turn_left()
                time.sleep(0.1)  # 0.3
    elif io_data[0] == 1 and io_data[3] == 0:
        if flag == 0:
            go_back()
            time.sleep(0.2)
            turn_right()
            time.sleep(0.3)
        elif flag == 1:
            if is_tag == 0:
                go_back()
                time.sleep(0.2)
                turn_right()
                time.sleep(0.3)
            elif is_tag == 1:
                go_back()
                time.sleep(0.2)
                turn_right()
                time.sleep(0.1)
    elif io_data[0] == 1 and io_data[3] == 1:
        go_back()
        time.sleep(0.2)
        turn_left()
        time.sleep(0.3)


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
    target = threading.Thread(target=April_start_detect)
    target.start()

    # while True:
    #     get_adio_data()
    #     if io_data[4] == 0 and io_data[5] == 0:
    #         break

    # stop()
    # time.sleep(0.5)
    # go_back_platform()
    # time.sleep(0.2)  # 0.8 1
    # stop()
    # time.sleep(0.5)
    # # # go_straight_slow()
    # # # time.sleep(0.2)  # 0.3
    # # # stop()
    # # # time.sleep(0.5)
    # down_back()
    # time.sleep(1.2)  # 1.0
    # turn_left()
    # time.sleep(0.2)

    # stop()
    # time.sleep(0.5)
    # back_demo()
    # time.sleep(1.4)
    # turn_left()
    # time.sleep(0.2)

    while True:
        get_adio_data()

        if camera_safe:
            if adc_value[5] > 1000 and adc_value[7] > 1000:
                down_platform_detect()
            else:
                up_platform_act()
        else:
            stop()

        # go_straight_slow()

        # print('adc_average2:', adc_average2)
        # print('adc_average4:', adc_average4)
        # print('adc_average3:', adc_average3)
        # print(adc_value)
        # print(adc_value[5], adc_value[7])
        # print(io_data)
        # print('distance:', distance)

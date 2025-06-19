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

blue_flag = 0
cx = 0
cy = 0

di_fang_kuai = 1             # 敌方块1
zhong_li_kuai = 2            # 中立块0
zha_dan_kuai = 0             # 炸弹块2

adc_value = [0]
adc_average = 0
adc_average2 = 0
io_data = [0]

ir_history = [[1]*5, [1]*5]  # 用于存储左右红外传感器的历史值
ir_index = 0

down_platform = 1
back = 0

IO_DISABLE_DURATION = 2000  # 失效持续时间，可根据需要调整
bomb_io_disable_until = 0  # 失效结束时间戳
current_time = time.time() * 1000

def ir_filter(new_left, new_right):
    global ir_history, ir_index

    # 更新历史数据
    ir_history[0][ir_index] = new_left
    ir_history[1][ir_index] = new_right
    ir_index = (ir_index + 1) % 5

    # 计算平均值
    avg_left = sum(ir_history[0]) / 5
    avg_right = sum(ir_history[1]) / 5

    # 返回滤波后的值(0或1)
    return 1 if avg_left > 0.5 else 0, 1 if avg_right > 0.5 else 0

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
    global cx
    global cy
    global blue_flag
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
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # 定义蓝色的HSV范围（示例值，需根据实际调整）
        # lower_blue = np.array([100, 150, 50])
        # upper_blue = np.array([140, 255, 255])
        lower_blue = np.array([97, 115, 72])
        upper_blue = np.array([140, 255, 255])
        # 创建掩膜
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        # 形态学操作（可选，用于降噪）
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_flag = 0
        # 标记坐标的列表
        coordinates = []
        if contours:
            # 找到最大轮廓
            largest_contour = max(contours, key=cv2.contourArea)
            # 计算轮廓中心
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                blue_flag = 1  # 1表示检测到蓝色物体
                coordinates.append((cx, cy))
                # 在画面中标记中心点
                cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
                cv2.putText(frame, f"({cx}, {cy})", (cx - 50, cy - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        # frame = cv2.rotate(frame, cv2.ROTATE_180)
        ad.update_frame(frame)
        # cv2.imshow("img", frame)
        # cv2.imshow('Mask', mask)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

def get_adio_data():
    global io_data
    global adc_value
    global adc_average
    global adc_average2
    adc_value = up.ADC_Get_All_Channle()
    adc_average = (adc_value[0] + adc_value[1] + adc_value[2]) / 3
    adc_average2 = (adc_value[0] + adc_value[1] + adc_value[2] + adc_value[3]) / 4

    io_all_input = up.ADC_IO_GetAllInputLevel()
    io_array = '{:08b}'.format(io_all_input)
    io_data.clear()
    for index, value in enumerate(io_array):
        io = (int)(value)
        io_data.insert(0, io)

def go_straight():
    global adc_average
    global adc_average2
    if adc_average > 650: #700
        up.CDS_SetSpeed(1, -500)  # 600
        up.CDS_SetSpeed(2, -600)  #
    else:
        up.CDS_SetSpeed(1, -400)  #500
        up.CDS_SetSpeed(2, -500)

def go_straight_slow():
    up.CDS_SetSpeed(1, -350)
    up.CDS_SetSpeed(2, -450)

def turn_left():
    up.CDS_SetSpeed(1, 600)   #700
    up.CDS_SetSpeed(2, -600)        #700

def turn_left_left():
    up.CDS_SetSpeed(1, 600)   #700
    up.CDS_SetSpeed(2, -600)        #700

def turn_left_tag():
    up.CDS_SetSpeed(1, 400)
    up.CDS_SetSpeed(2, -400)

def turn_left_bug():
    up.CDS_SetSpeed(1, 700)   #700
    up.CDS_SetSpeed(2, -700)        #700

def turn_left_down():
    up.CDS_SetSpeed(1, 450)   #700
    up.CDS_SetSpeed(2, -450)        #700

def turn_right():
    up.CDS_SetSpeed(1, -600)          # 700
    up.CDS_SetSpeed(2, 600)     # 700

def turn_right_right():
    up.CDS_SetSpeed(1, -600)          # 700
    up.CDS_SetSpeed(2, 600)     # 700

def turn_right_tag():
    up.CDS_SetSpeed(1, -400)
    up.CDS_SetSpeed(2, 400)

def turn_right_down():
    up.CDS_SetSpeed(1, -450)          # 700
    up.CDS_SetSpeed(2, 450)     # 700

def go_back():
    up.CDS_SetSpeed(1, 400)
    up.CDS_SetSpeed(2, 400)

def go_back_tag():
    up.CDS_SetSpeed(1, 600)
    up.CDS_SetSpeed(2, 600)

def down_back():
    up.CDS_SetSpeed(1, 700)
    up.CDS_SetSpeed(2, 700)

def stop():
    up.CDS_SetSpeed(1, 0)
    up.CDS_SetSpeed(2, 0)

def set_angle_up():
    up.CDS_SetAngle(3, 744, 512)
    up.CDS_SetAngle(4, 320, 512)

def set_angle_mid():
    up.CDS_SetAngle(3, 456, 512) #436
    up.CDS_SetAngle(4, 567, 512) #587

def set_angle_down():
    up.CDS_SetAngle(3, 337, 512)
    up.CDS_SetAngle(4, 686, 512)

def set_angle_platform():
    up.CDS_SetAngle(3, 820, 512)  # 3号 上面698 平地442 底下326  台下抬高849
    up.CDS_SetAngle(4, 198, 512)  # 4号 上面320 平地552 底下703  台下抬高198


def down_platform_detect():
    global io_data
    if io_data[1] == 0 and io_data[2] == 0:
        stop()
        set_angle_mid()
        time.sleep(0.5)
        down_platform_act()
    elif io_data[2] == 0 and io_data[3] == 1:
        turn_left_down()
    elif io_data[2] == 1 and io_data[3] == 0:
        turn_right_down()
    else:
        turn_right_down()

def down_platform_act():
    global down_platform
    down_back()
    time.sleep(0.5)
    set_angle_up()
    time.sleep(0.3)
    set_angle_down()
    time.sleep(0.5)
    down_platform = 0

def tag_solve():
    global is_tag
    global distance
    global mid
    global tag_width
    global io_data
    global current_time
    global bomb_io_disable_until
    global IO_DISABLE_DURATION
    # print(distance)

    if is_tag == 1:
        if distance > 190:
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
        elif distance <= 190:
            if io_data[1] == 0 and io_data[2] == 0:
                go_straight()
            elif io_data[1] == 0 and io_data[2] == 1:
                turn_left()
            elif io_data[1] == 1 and io_data[2] == 0:
                turn_right()
    if is_tag == 0:
        if distance < 200 and 120 < mid < 200:  #90 240
            go_back()   #go_back()
            time.sleep(0.1)
            turn_left_bug()
            time.sleep(0.5) #0.2
            bomb_io_disable_until = current_time + IO_DISABLE_DURATION

def blue_solve():
    if cx < 160 - 20:
        turn_left_tag()
    elif cx > 160 + 20:
        turn_right_tag()
    else:
        go_straight()

def up_platform_act():
    global io_data
    global is_tag
    global flag
    global adc_average
    global adc_average2
    set_angle_mid()
    if io_data[0] ==0 and io_data[3] == 0: #最外层if-elif控制边界
        go_straight()
        if flag == 1:
            tag_solve()
        elif blue_flag == 1:
            blue_solve()
        elif flag == 0 or blue_flag == 0: #flag == 0 or is_tag == 1
            if io_data[1] == 0 and io_data[2] == 0:  # 次外层if-elif控制箱子
                if io_data[0] != 1 and io_data[3] != 1:  # 最内层if-elif控制侧面
                    go_straight()
            elif io_data[1] == 1 and io_data[2] == 1:
                if io_data[4] == 0 and io_data[5] == 1:
                    turn_left_left()
                    time.sleep(0.5)
                elif io_data[4] == 1 and io_data[5] == 0:
                    turn_right_right()
                    time.sleep(0.5)
                elif io_data[4] == 0 and io_data[5] == 0:
                    turn_right_right()
                    time.sleep(0.5)
                elif io_data[4] == 1 and io_data[5] == 1:
                    go_straight()
            elif io_data[1] == 0 and io_data[2] == 1:
                turn_left()
            elif io_data[1] == 1 and io_data[2] == 0:
                turn_right()
    elif io_data[0] == 0 and io_data[3] == 1:
        go_back()
        time.sleep(0.3)
        turn_left()
        time.sleep(0.3)
    elif io_data[0] == 1 and io_data[3] == 0:
        go_back()
        time.sleep(0.3)   #0.2
        turn_right()
        time.sleep(0.3)
    elif io_data[0] == 1 and io_data[3] == 1:
        go_back()
        time.sleep(0.2)
        turn_left()
        time.sleep(0.3)

def up_platform_act_edge():
    global io_data
    set_angle_mid()
    if io_data[0] ==0 and io_data[3] == 0: #最外层if-elif控制边界
        go_straight_slow()
        # if io_data[1] == 0 and io_data[2] == 0:  # 次外层if-elif控制箱子
        #     if io_data[0] != 1 and io_data[3] != 1:  # 最内层if-elif控制侧面
        #         go_straight()
        # elif io_data[1] == 1 and io_data[2] == 1:
        #         # if io_data[4] == 0 and io_data[5] == 1:
        #         #     turn_left_left()
        #         #     time.sleep(0.5)
        #         # elif io_data[4] == 1 and io_data[5] == 0:
        #         #     turn_right_right()
        #         #     time.sleep(0.5)
        #         # elif io_data[4] == 0 and io_data[5] == 0:
        #         #     turn_right_right()
        #         #     time.sleep(0.5)
        #         # elif io_data[4] == 1 and io_data[5] == 1:
        #         #     go_straight()
        #     go_straight()
        # elif io_data[1] == 0 and io_data[2] == 1:
        #     turn_left()
        # elif io_data[1] == 1 and io_data[2] == 0:
        #     turn_right()
    elif io_data[0] == 0 and io_data[3] == 1:
        stop()
        time.sleep(0.15)
        turn_left()
        time.sleep(0.1)
    elif io_data[0] == 1 and io_data[3] == 0:
        stop()
        time.sleep(0.15)
        turn_right()
        time.sleep(0.1)
    elif io_data[0] == 1 and io_data[3] == 1:
        go_back()
        # time.sleep(0.2)
        # turn_left()
        # time.sleep(0.25)

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

    while True:
        get_adio_data()

        filtered_left, filtered_right = ir_filter(io_data[0], io_data[3])

        if adc_average2 < 418:
            if down_platform == 0:
                set_angle_platform()
                time.sleep(1.5)
                down_platform = 1
            down_platform_detect()
        else:
            up_platform_act()

        # set_angle_mid()
        # print('adc_average:', adc_average)
        # print('adc_average2:', adc_average2)
        # print(adc_value)
        # print(adc_value)
        # print('distance:', distance)

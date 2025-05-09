import uptech
import time

def go_straight():
    up.CDS_SetSpeed(1, 480)  #右轮正转
    up.CDS_SetSpeed(2, 500)  #左轮正转

def turn_left():
    up.CDS_SetSpeed(1, -400)  #右轮反转
    up.CDS_SetSpeed(2, 400)  #左轮正转

def turn_right():
    up.CDS_SetSpeed(1, 400)  #右轮正转
    up.CDS_SetSpeed(2, -400)  #左轮反转

def go_back():
    up.CDS_SetSpeed(1, -500)  # 右轮正转
    up.CDS_SetSpeed(2, -500)  # 左轮正转

def down_platform_act():
    up.CDS_SetSpeed(1, -730) #右轮反转
    up.CDS_SetSpeed(2, -750) #左轮反转
    time.sleep(1)
    up.CDS_SetAngle(3, 302, 512)
    up.CDS_SetAngle(4, 767, 512)
    time.sleep(0.8)
    up.CDS_SetAngle(3, 773, 712)
    up.CDS_SetAngle(4, 314, 712)
    time.sleep(0.8)

def up_platform_act():
    if io_data[0] == 0 and io_data[3] == 0:
        go_straight()
    elif io_data[0] == 0 and io_data[3] == 1:
        turn_left()
    elif io_data[0] == 1 and io_data[3] == 0:
        turn_right()
    elif io_data[0] == 1 and io_data[3] == 1:
        go_back()
        time.sleep(0.3)
        turn_right()
        time.sleep(0.2)

if __name__ == "__main__":
    up = uptech.UpTech()
    up.LCD_Open(2)
    up.ADC_IO_Open()
    up.ADC_Led_SetColor(0, 0x2F0000)
    up.ADC_Led_SetColor(1, 0x002F00)
    up.CDS_Open()
    up.CDS_SetMode(1, 1)
    up.CDS_SetMode(2, 1)
    up.CDS_SetMode(3, 0)
    up.CDS_SetMode(4, 0)
    up.LCD_PutString(40, 0, 'formal')
    up.LCD_Refresh()
    up.LCD_SetFont(up.FONT_6X10)
    io_data = []

    while True:
        adc_value = up.ADC_Get_All_Channle()
        io_all_input = up.ADC_IO_GetAllInputLevel()
        io_array = '{:08b}'.format(io_all_input)
        io_data.clear()

        for index, value in enumerate(io_array):
            io = (int)(value)
            io_data.insert(0, io)
        if io_data[4] == 0 and io_data[5] == 0:
            break

    while True:
        adc_value = up.ADC_Get_All_Channle()
        io_all_input = up.ADC_IO_GetAllInputLevel()
        io_array = '{:08b}'.format(io_all_input)
        io_data.clear()

        for index, value in enumerate(io_array):
            io = (int)(value)
            io_data.insert(0, io)
        print(io_data)
        up_platform_act()





import uptech
import time

def down_platform_act():
    up.CDS_SetSpeed(1, -730) #730
    up.CDS_SetSpeed(2, -750) #750
    time.sleep(1)
    up.CDS_SetAngle(3, 541, 512) #300  //减小为向上
    up.CDS_SetAngle(4, 750, 512) #730  //增大为向上
    time.sleep(0.8)
    up.CDS_SetAngle(3, 982, 712) #550  664
    up.CDS_SetAngle(4, 308, 712) #512  338
    time.sleep(0.8)

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

while True:
    down_platform_act()





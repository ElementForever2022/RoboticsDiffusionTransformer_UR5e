import serial,time, keyboard

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

motor_open_list =  (0x02,0x00,0x20,0x49,0x20,0X00,0xC8)
motor_close_list =  (0x02,0x01,0x20,0x49,0x20,0X00,0xC8)


ser.write(motor_open_list)
time.sleep(5)
ser.write(motor_close_list)
time.sleep(5)
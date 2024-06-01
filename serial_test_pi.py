import serial

# 设置串口参数
serial_port = '/dev/ttyUSB0'  # 根据实际情况设置
baud_rate = 57600           # 波特率

# 打开串口
ser = serial.Serial(serial_port, baud_rate)

try:
    while True:
        if ser.in_waiting > 0:
            # 读取一行数据
            line = ser.readline().decode('utf-8').rstrip()
            print(line)

except KeyboardInterrupt:
    print("程序结束")

finally:
    # 关闭串口
    ser.close()

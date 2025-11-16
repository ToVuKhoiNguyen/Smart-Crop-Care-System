import serial

try:
    ser = serial.serial('COM11', 9600, timeout=1)  # Thay COMx bằng cổng thật (ví dụ COM3)
    print("Kết nối thành công")
    ser.close()
except Exception as e:
    print("Lỗi:", e)
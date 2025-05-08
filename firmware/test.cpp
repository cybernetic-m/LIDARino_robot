import serial

ser = serial.Serial('COM3', 9600)  # Replace 'COM3' with your Arduino's serial port

int1 = 123
int2 = 456

ser.write(f"{int1},{int2}\n".encode())

ser.close()

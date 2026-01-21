import serial

ser = (serial.Serial('COM4', 115200))
with open("fuzzy_log.csv", "w") as f:
    f.write("e,de,Kp,Kd\n")
    while True:
        f.write(ser.readline().decode())

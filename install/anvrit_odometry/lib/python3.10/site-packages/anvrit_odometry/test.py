# import serial
# import time

# def main():
#     # Adjust the serial port name as needed, e.g. '/dev/ttyUSB0' or '/dev/ttyACM0'
#     ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
#     # time.sleep(1)  # Adjust wait time if needed

#     # Send RPM values of 120 for both wheels
#     data_to_send = "0 0\n"
#     ser.write(data_to_send.encode())
#     print(f"Sent: {data_to_send.strip()}")

#     ser.close()

# if __name__ == "__main__":
#     main()



import serial
import time

# Establish serial connection
ser = serial.Serial('/dev/ttyACM0', 9600)

# Wait for the serial connection to be established
time.sleep(2)

# Send a message to the Arduino
message = "Hello, Arduino!"
ser.write(message.encode())

import serial
import time

try:
    ser = serial.Serial(port='/dev/cu.usbmodem14201', baudrate=57600)
    
    # Send command
    command = 'E100\r\n'
    ser.write(command.encode())
    time.sleep(1)
    
    # Read feedback
    feedback = ser.readline().decode().strip()
    print("Feedback:", feedback)
    
except serial.SerialException as e:
    print("Serial port error:", str(e))
except Exception as e:
    print("Error:", str(e))
finally:
    ser.close()
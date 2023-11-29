import serial
import time

try:
    ser = serial.Serial(port='/dev/cu.usbmodem14201', baudrate=9600)
    
    # Send command
    # time.sleep(1)
    # ser.setDTR(level=0)
    time.sleep(1)
    command = '-100\n'
    ser.write(command.encode())
    time.sleep(1)
    
    # # Read feedback
    # feedback = ser.readline().decode().strip()
    # print("Feedback:", feedback)
    
except serial.SerialException as e:
    print("Serial port error:", str(e))
except Exception as e:
    print("Error:", str(e))
finally:
    ser.close()
# import arduinoserial

# arduino = arduinoserial.SerialPort('/dev/cu.usbmodem14201', 9600)
# # print arduino.read_until('\n')
# command = '100\n\r'
# # ser.write(command.encode())

# arduino.write(command.encode())
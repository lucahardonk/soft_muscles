import sys
import serial
import time
def send_command_to_arduino(command):
    try:
        # Connect to the Arduino
        arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        arduino.flush()
        
        # Send the command to the Arduino
        arduino.write((command + '\n').encode('utf-8'))
        
        # Optionally, read the response from the Arduino
        if arduino.in_waiting > 0:
            response = arduino.readline().decode('utf-8').strip()
            print(f"Arduino response: {response}")
        
        # Close the connection
        arduino.close()
    except Exception as e:
        print(f"Failed to send command to Arduino: {e}")

def close_all():
    send_command_to_arduino("CLOSE_A_AIR")
    send_command_to_arduino("CLOSE_B_AIR")
    send_command_to_arduino("CLOSE_A_OUT")
    send_command_to_arduino("CLOSE_B_OUT")
    
def open_all():
    send_command_to_arduino("OPEN_A_AIR")
    send_command_to_arduino("OPEN_B_AIR")
    send_command_to_arduino("OPEN_A_OUT")
    send_command_to_arduino("OPEN_B_OUT")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 send_command.py <command>")
        sys.exit(1)
    
    command = sys.argv[1]
    if(command == "OPEN_ALL"):
        open_all()
    	
    elif(command == "CLOSE_ALL"):
        close_all()
    	
    elif(command == "PUMP_TO_100"):
        close_all()
        time.sleep(0.2)
        send_command_to_arduino("OPEN_B_OUT")
        send_command_to_arduino("OPEN_A_AIR")
        time.sleep(0.2)
        send_command_to_arduino("MOVE_MOTOR:100")
    
    elif(command == "PUMP_TO_0"):
        close_all()
        time.sleep(0.2)
        send_command_to_arduino("OPEN_A_OUT")
        send_command_to_arduino("OPEN_B_AIR")
        time.sleep(0.2)
        send_command_to_arduino("MOVE_MOTOR:0")
     
    else:
        send_command_to_arduino(command)


import serial
from time import sleep

def send_warn(id):
    print("warn sent", id)

if __name__ == '__main__':
    ser = serial.Serial ("/dev/ttyS1", 115200)    #Open port with baud rate

    utility_pole_id = 0;
    while True:
        received_data = ser.read()              #read serial port
        data_left = ser.inWaiting()             #check for remaining byte
        received_data += ser.read(data_left)
        received_data = received_data.decode("utf-8")
        print (received_data)
        print (type(received_data))
        if(received_data.find("set ON") != -1):
            send_warn(utility_pole_id)

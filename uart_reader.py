import serial
import requests
from time import sleep

ur = 'https://peaceful-crag-65410.herokuapp.com/todo/api/v1.0/pillars'

def add_utility_pole(idef,data):
    r = requests.post(ur, json = {"data":data, "is_stable": True});
    print(r.text)

def send_warn(idef, data):
    r = requests.put(ur+"/"+str(idef),json = {"data":data, "is_stable": False}, verify=False).status_code;
    print("warn for ", idef, " sent with code: ", r)

def reset_warn(idef, data):
    r = requests.put(ur+"/"+str(idef),json = {"data":data, "is_stable": True}, verify=False).status_code;
    print("warn for ", idef, " reseted with code: ", r)



if __name__ == '__main__':
    ser = serial.Serial ("/dev/ttyS1", 115200)    #Open port with baud rate

    utility_pole_id = 2;
    coord_data = '59.860079, 29.827092'
    while True:
        received_data = ser.read()              #read serial port
        data_left = ser.inWaiting()             #check for remaining byte
        received_data += ser.read(data_left)
        received_data = received_data.decode("utf-8")
        print (received_data)
        print (type(received_data))
        if(received_data.find("set ON") != -1):
            send_warn(utility_pole_id, coord_data)

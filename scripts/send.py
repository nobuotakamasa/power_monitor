#just for testing
import subprocess
import time
import sys

candevice = sys.argv[1]
filename = sys.argv[2]
interval = float(sys.argv[3])
print(candevice, filename, interval)
command = ["cansend", candevice, ""]

with open(filename) as f:
    lines = f.readlines()
while True:
    for line in lines:
        time.sleep(interval)
        line = line[:-1].split(" ")
        canid = line[4]
        data = line[9:]
        #print(canid, data)
        send = canid+"#"
        for d in data:
            send += d
        command[2] = send
        #print(command)
        process = subprocess.Popen(command)


    


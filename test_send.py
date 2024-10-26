#just for testing
import subprocess
import time

currents = [
"8000D69CC8CAB516",
"800011BCC8CAB516",
"80001369C8CAB516",
"7FFFDE3BC8CAB516",
"8000099AC8CAB516",
"80000AB1C8CAB516",
"7FFFFFFFC8CAB516",
"80000004C8CAB516",
"80000A33C8CAB516",
"80000A33C8CAB516",
"80000AB1C8CAB516",
"7FFFFFFFC8CAB516",
]


candata_ids = ["3C2#","3C3#","3C4#","3C5#","3C6#","3C7#","3C8#","3C9#","3CA#","3CB#"]

candata_voltages = ["000#8000B516", "000#8100B516", "000#8000C516"]
index = 0

command = ["cansend", "vcan0", ""]
while True:
    for canid in candata_ids:
        time.sleep(0.1)
        command[2] = canid + currents[index]
        index += 1
        if index >= len(currents):
            index = 0
        process = subprocess.Popen(command)
    command[2] = candata_voltages[index % 3]
    process = subprocess.Popen(command)





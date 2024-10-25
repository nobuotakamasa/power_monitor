import subprocess
import time

candata_current = ["3C9#80000002C8CAB516",
"3C2#8000D69CC8CAB516",
"3C3#800011BCC8CAB516",
"3C4#80001369C8CAB516",
"3C5#7FFFDE3BC8CAB516",
"3C6#8000099AC8CAB516",
"3C7#80000AB1C8CAB516",
"3C8#7FFFFFFFC8CAB516",
"3C9#80000004C8CAB516",
"3Ca#80000A33C8CAB516"]

candata_voltage = "000#8000B516"

command = ["cansend", "vcan0", ""]
while True:
    for data in candata_current:
        time.sleep(0.1)
        command[2] = data
        process = subprocess.Popen(command)
    command[2] = candata_voltage
    process = subprocess.Popen(command)





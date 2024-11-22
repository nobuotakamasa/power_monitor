import sys
import subprocess
import threading
from can_receiver.receiver import CanReceiver
from threading import Thread, Event, Condition, Lock

can_channel = sys.argv[1]
command_send = ["cansend", can_channel, ""]
command_dump = ["candump", can_channel]
CANID_UDS_CLIENT=0x68d
CANID_UDS_SERVER=0x68e
#CANID_UDS_CLIENT=0x68f
#CANID_UDS_SERVER=0x690
#CANID_IP=0x3c3
BUFFER_SIZE = 10000            # リングバッファのサイズ


# CANインターフェースとフィルタ設定
#CAN_INTERFACE = sys.argv[1]  # name of CAN interface


CANID_IP=0x3c2
canids_init = [CANID_IP, CANID_UDS_CLIENT, CANID_UDS_SERVER] #[IP, UDS_CLIENT, UDS_SERVER]
can_num = 9
HW_RESET = f"{CANID_UDS_CLIENT:x}#021101"

ack1 = "3000000000000000"
ack2 = "036ef01000000000"

ack1 = [0x30,0x00,0x00,0x00, 0x00,0x00,0x00,0x00]
ack2 = [0x03,0x6e,0xf0,0x10, 0x00,0x00,0x00,0x00]


#  can0  3C2   [8]  80 00 7B 32 C8 CA B5 16

def can_rec(canid):
    canid = f"{CANID_UDS_CLIENT:X}"
    print(canid)
    while True:
        process = subprocess.Popen(command_dump).split(" ")

def make_send_str(send):
    send_str =  f"{CANID_UDS_CLIENT:x}#"
    for d in send:
        send_str += f"{d:x}"
    return send_str

def make_send_data(canids_target):
    target_ip_hi = (canids_target[0] & 0xf00) >> 8
    target_ip_lo = canids_target[0] & 0xff
    target_client_hi = (canids_target[1] >> 8) & 0xff
    target_client_lo = canids_target[1] & 0xff
    target_server_hi = (canids_target[2] >> 8) & 0xff
    target_server_lo = canids_target[2] & 0xff
    send1 = [0x10,0x09,0x2E,0xF0,0x10,target_ip_hi,target_ip_lo,target_client_hi]
    send2 = [0x21,target_client_lo,target_server_hi,target_server_lo]
    send1_str =  make_send_str(send1)
    send2_str =  make_send_str(send2)
    return send1_str, send2_str
    #print(send1_str)
    #print(send2_str)
    #print(hwreset)
    #process = subprocess.Popen(command_send)


for i in range(can_num):
    canids_target = list(map(lambda n: n+i, canids_init))
    #print(canids_target)
    send(canids_target)


def candump(can_rec, target_can_id):
    while True:
        can_id, data = can_rec.can_receive(target_can_id)
        print(f"CAN ID {hex(target_can_id)} received data: {list(map(lambda n: f'{n:X}', data))}")


def main():
    can_rec = CanReceiver(can_channel, BUFFER_SIZE)
    # スレッドを作成
    thread1 = Thread(target=candump, args=(can_rec, CANID_UDS_SERVER))
    thread1.start()

    try:
        thread1.join()
    except KeyboardInterrupt:
        print("Stopping...")
        can_rec.stop_event.set()
        can_rec.receiver_thread.join()

if __name__ == "__main__":
    main()



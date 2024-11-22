import can
import time
from threading import Thread, Event
import sys

ack = [[0x30,0x00,0x00,0x00, 0x00,0x00,0x00,0x00],[0x03,0x6e,0xf0,0x10, 0x00,0x00,0x00,0x00]]
try:
    START_IP=int(sys.argv[1])
    can_num = int(sys.argv[2])
except:
    print(f"usage python3 {sys.argv[0]} offset_from_0x3c2 number_of_sensors")
    exit(1)
CANID_UDS_CLIENT=0x68d+START_IP*2
CANID_UDS_SERVER=0x68e+START_IP*2
CANID_IP=0x3c2+START_IP
HW_RESET = [0x02,0x11,0x01]
# 使用するCANチャンネルとビットレート
CHANNEL = "can0"
BITRATE = 500000


class BreakLoop(Exception):
    pass

class CANManager:
    def __init__(self, channel, bitrate):
        self.channel = channel
        self.bitrate = bitrate
        self.bus = can.interface.Bus(channel=channel, bustype="socketcan", bitrate=bitrate)
        self.stop_event = Event()
        self.nack_received = Event()  # NACKを受信した場合に通知
        self.received_data = None  # 受信データを保持

    def send_data(self, data, ack_data):
        """CANID_UDS_CLIENTでデータを送信し、ack_dataをACKとして扱う"""
        message = can.Message(arbitration_id=CANID_UDS_CLIENT, data=data, is_extended_id=False)
        self.bus.send(message)
        print(f"Sent: {list(map(hex,data))}")

        # ACK/NACKの待機
        if not self.wait_for_ack(ack_data):
            print("no ACK received. Aborting transmission.")
            self.nack_received.set()

    def wait_for_ack(self, ack_data, timeout=2.0):
        """ACKを待機"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.received_data == ack_data:
                print("ACK received")
                self.received_data = None  # 受信データをリセット
                return True
            time.sleep(0.1)  # 少し待機
        return False

    def receive_ack(self):
        """CANID_UDS_SERVERのデータを監視"""
        while not self.stop_event.is_set():
            message = self.bus.recv(1.0)  # 1秒タイムアウトで待機
            if message and message.arbitration_id == CANID_UDS_SERVER:
                self.received_data = list(message.data)
                print(f"Received: {list(map(hex,self.received_data))}")

    def start_ack_listener(self):
        """ACK/NACK受信スレッドを開始"""
        self.rx_thread = Thread(target=self.receive_ack, daemon=True)
        self.rx_thread.start()

    def stop(self):
        """すべての処理を停止"""
        self.stop_event.set()
        self.rx_thread.join()
        self.bus.shutdown()

def make_send_data(canids_target):
    target_ip_hi = (canids_target[0] & 0xf00) >> 8
    target_ip_lo = canids_target[0] & 0xff
    target_client_hi = (canids_target[1] >> 8) & 0xff
    target_client_lo = canids_target[1] & 0xff
    target_server_hi = (canids_target[2] >> 8) & 0xff
    target_server_lo = canids_target[2] & 0xff
    send1 = [0x10,0x09,0x2E,0xF0,0x10,target_ip_hi,target_ip_lo,target_client_hi]
    send2 = [0x21,target_client_lo,target_server_hi,target_server_lo]
    return send1, send2

def main():
    canids_init = [CANID_IP, CANID_UDS_CLIENT, CANID_UDS_SERVER] #[IP, UDS_CLIENT, UDS_SERVER]
    data_lists = []
    for i in range(can_num):
        canids_target = list(map(lambda n: n+i, canids_init))
        d1,d2 = make_send_data(canids_target)
        data_lists.append([d1,d2])

    print(data_lists)

    manager = CANManager(CHANNEL, BITRATE)
    manager.start_ack_listener()  # ACK/NACK受信スレッドを開始

    try:
        count = 1
        for n,data_list in enumerate(data_lists):
            print("Connect the CAB500 and press Enter.", n)
            input()
            for i in range(len(data_list)):
                # ACK以外が受信されたら送信中断
                if manager.nack_received.is_set():
                    raise BreakLoop("ack not received")
                # データ送信とACK待機
                manager.send_data(data_list[i], ack[i])
                time.sleep(0.01)  # 次の送信までの間隔
    except BreakLoop as e:
        print(e)
    except KeyboardInterrupt:
        print("Stopping CANManager...")
    finally:
        manager.stop()


if __name__ == "__main__":
    main()

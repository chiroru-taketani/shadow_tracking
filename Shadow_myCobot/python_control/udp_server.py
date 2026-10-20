import socket
import time
from pymycobot.mycobot280 import MyCobot280  # myCobot用ライブラリ

# --- 1. 定数の設定（ここの部分が抜けている可能性があります！） ---
# UDP設定
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

# myCobot設定
SERIAL_PORT = "/dev/tty.usbserial-5A491022661"  # ポート番号（Macの環境に合わせて変更）
BAUD_RATE = 115200

# キャリブレーション（座標オフセット）
OFFSET_X = 100.0
OFFSET_Y = 0.0
OFFSET_Z = 150.0

# 安全装置（リミット値）
MIN_X, MAX_X = -200.0, 200.0
MIN_Y, MAX_Y = -200.0, 200.0
MIN_Z, MAX_Z = 100.0, 300.0

# 安全値に丸める関数（クランプ処理）
def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

# --- 2. メイン処理 ---
def main():
    # myCobotの初期化
    print(f"Connecting to myCobot at {SERIAL_PORT}...")
    try:
        mc = MyCobot280(SERIAL_PORT, BAUD_RATE)
        mc.power_on()
        time.sleep(2)  # 起動待ち

        # 最初は安全な姿勢（直立など）で待機させる
        mc.send_angles([0, 0, 0, 0, 90, 0], 30)
        #mc.send_coords([150.0, 0.0, 200.0, -90, 0, 0], 50, 1)
        time.sleep(3)
        print("myCobot Connected and Ready!")
    except Exception as e:
        print(f"myCobot Connection failed: {e}")
        return

    # UDPソケットの初期化
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"UDP Server listening on {UDP_IP}:{UDP_PORT}...")

    last_time = time.time()

    try:
        while True:
            # データの受信
            data, addr = sock.recvfrom(1024)

            try:
                msg = data.decode('utf-8').strip()
                parts = msg.split(',')

                if len(parts) >= 6:
                    raw_x = float(parts[0])
                    raw_y = float(parts[1])
                    raw_z = float(parts[2])
                    rx = float(parts[3])
                    ry = float(parts[4])
                    rz = float(parts[5])

                    # 座標変換（オフセット加算）
                    target_x = raw_x + OFFSET_X
                    target_y = raw_y + OFFSET_Y
                    target_z = raw_z + OFFSET_Z

                    # ソフトウェアリミット（安全装置）
                    safe_x = clamp(target_x, MIN_X, MAX_X)
                    safe_y = clamp(target_y, MIN_Y, MAX_Y)
                    safe_z = clamp(target_z, MIN_Z, MAX_Z)

                    # myCobotへ送信（0.1秒に1回）
                    current_time = time.time()
                    if current_time - last_time >= 0.1:
                        print(f"Move to -> X:{safe_x:.1f}, Y:{safe_y:.1f}, Z:{safe_z:.1f}")
                        # X, Y, Z, Rx, Ry, Rz, 速度, 補間モード
                        mc.send_coords([safe_x, safe_y, safe_z, rx, ry, rz], 100, 0)
                        last_time = current_time

            except ValueError:
                print(f"Parse error. Raw data: {data}")

    except KeyboardInterrupt:
        print("\nStopping UDP Server.")
        print("初期姿勢に戻ります。ぶつからないようご注意ください...")
        try:
            # 初期姿勢（安全な角度）へ移動
            mc.send_angles([0, 0, 0, 0, 90, 0], 50)
            time.sleep(3) # 移動が終わるまで待機
            
            # モーターの力を抜く（手で動かせる状態にする）
            mc.release_all_servos()
            print("アームの力を抜いて安全に終了しました。")
        except Exception as e:
            print(f"終了処理中にエラーが発生しました: {e}")
    finally:
        sock.close()

if __name__ == "__main__":
    main()

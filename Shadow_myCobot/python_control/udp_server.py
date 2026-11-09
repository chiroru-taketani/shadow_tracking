import socket
import time
from pymycobot.mycobot280 import MyCobot280  # myCobot用ライブラリ

# --- 1. 定数の設定---
# C++側の光源初期Z座標（scanH / 2.0 + obj_light_dist = 100.0 + 140.0）
LIGHT_INIT_Z = 240.0

# UDP設定
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

# myCobot設定
SERIAL_PORT = "/dev/tty.usbserial-5A491022661"  # ポート番号（Macの環境に合わせて変更）
BAUD_RATE = 115200

# myCobotが安全に動ける基準位置（mm）
BASE_X = 180.0  #前後
BASE_Y = 0.0  #左右
BASE_Z = 180.0 #高さ

# 安全装置（myCobot 280 の可動限界クランプ）
MIN_X, MAX_X = 100.0, 240.0 #前後
MIN_Y, MAX_Y = -150.0, 150.0 #左右
MIN_Z, MAX_Z = 100.0, 260.0 #高さ

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

                    # 座標変換（C++ -> myCobotベース座標系）
                    #1. 左右: C++の raw_x を myCobot の Y軸 へ（myCobotは左が+のため反転）
                    target_y = BASE_Y - raw_x

                    # 2. 上下: C++の raw_y を myCobot の Z軸 へ（初期値64.0からの差分を加算）
                    target_z = BASE_Z + (raw_y - 64.0)

                    # 3. 前後: C++の raw_z を myCobot の X軸 へ
                    target_x = BASE_X + (raw_z - LIGHT_INIT_Z)


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
        print("初期姿勢に戻ります。")
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

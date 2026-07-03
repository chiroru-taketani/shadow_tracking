import time
from pymycobot.mycobot280 import MyCobot280

# シリアルポートとボーレートの指定
# Macの場合の例（環境に合わせてポート名は変更してください）
PORT = "/dev/tty.usbserial-5A491022661"
BAUD = 115200

def main():
    print(f"Connecting to myCobot at {PORT}...")
    try:
        # myCobot280オブジェクトの作成
        mc = MyCobot280(PORT, BAUD)
        print("Connected successfully! Waiting 3 seconds for the robot to boot...")
        # シリアル接続時にロボットが再起動することがあるため、少し長めに待つ
        time.sleep(3)
    except Exception as e:
        print(f"Connection failed: {e}")
        return

    print("Powering on and reading current angles...")
    mc.power_on()
    time.sleep(1)

    start_angles = mc.get_angles()
    print(f"Current Safe Angles: {start_angles}")

    print("\nSending command to move to [0, 0, 0, 0, 0, 0] at speed 30...")
    # 安全のためゆっくり(スピード30)で0度へ移動
    mc.send_angles([0, 0, 0, 0, 90, 0], 30)

    print("Moving...")
    # 移動が終わるまで5秒待機
    time.sleep(5)

    end_angles = mc.get_angles()
    print(f"Final Angles: {end_angles}")
    print("\n🎉 Done! 今度こそ動いたはずです！！")

if __name__ == "__main__":
    main()

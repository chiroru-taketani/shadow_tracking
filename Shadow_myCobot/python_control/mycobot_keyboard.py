import time
import sys
import tty
import termios
from pymycobot.mycobot280 import MyCobot280

# ==============================================================================
# myCobot 280 キーボードでのXYZ操作テスト
# 自分のPCのキーボードを使って、myCobotの空間座標(XYZ)を直感的に理解するためのツールです。
# ==============================================================================

PORT = "/dev/tty.usbserial-5A491022661"
BAUD = 115200

# 初期位置（安全な場所）
current_x = 0.0
current_y = 0.0
current_z = 0.0

# 手先の姿勢は固定（真下を向くような姿勢）
current_rx = 0.0
current_ry = 0.0
current_rz = 0.0

# 1回のキー入力で動かす距離(mm)
STEP = 10.0

# Macのターミナルで、Enterキーを押さずにキー入力を1文字だけ受け取るための魔法の関数
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    global current_x, current_y, current_z

    print(f"myCobot ({PORT}) に接続中...")
    try:
        mc = MyCobot280(PORT, BAUD)
        mc.power_on()
        time.sleep(2)
    except Exception as e:
        print(f"接続エラー: {e}")
        return

    print("初期位置に移動します。ぶつからないように注意してください...")
    # 一旦安全な角度を経由する
    mc.send_angles([0, -20, -20, 0, 90, 0], 50)
    time.sleep(3)


    # 座標制御モードで初期位置へ
    mc.send_coords([current_x, current_y, current_z, current_rx, current_ry, current_rz], 50, 0)
    time.sleep(2)

    print("\n" + "="*40)
    print("🎮 キーボードコントロール開始！")
    print("W / S : X軸（前後）を移動")
    print("A / D : Y軸（左右）を移動")
    print("E / Q : Z軸（上下）を移動")
    print("SPACE : 現在の実際の座標を読み取って表示")
    print("ESC   : 終了して力を抜く")
    print("="*40 + "\n")

    while True:
        ch = getch()

        # ESCキーで終了 (アスキーコード 27)
        if ord(ch) == 27:
            print("\n終了します。アームの力を抜きます...")
            break

        ch = ch.lower()
        changed = False

        if ch == 'w':
            current_x += STEP
            changed = True
        elif ch == 's':
            current_x -= STEP
            changed = True
        elif ch == 'a':
            current_y += STEP
            changed = True
        elif ch == 'd':
            current_y -= STEP
            changed = True
        elif ch == 'e':
            current_z += STEP
            changed = True
        elif ch == 'q':
            current_z -= STEP
            changed = True
        elif ch == ' ':
            coords = mc.get_coords()
            print(f"\n[現在のアーム実際の座標] -> {coords}")

        # 座標が変更されたら送信
        if changed:
            print(f"\r移動指示 -> X:{current_x:.1f}, Y:{current_y:.1f}, Z:{current_z:.1f}  ", end="")
            # 速度80でサクサク動かす
            mc.send_coords([current_x, current_y, current_z, current_rx, current_ry, current_rz], 80, 0)

    # モーターのトルクを解除
    mc.release_all_servos()

if __name__ == '__main__':
    main()

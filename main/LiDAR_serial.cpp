// コンパイルコマンド: g++ LiDAR_serial.cpp -o LiDAR_serial -std=c++11
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <vector>

// Linux/macOSのシリアル通信用ヘッダー
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// config.hからポート名や待機時間を読み込むためにインクルードします。
// これらの設定が不要な場合は、この行を削除し、main関数内の値を直接指定してください。
#include "config.h"

// 座標データを保持する構造体
struct Point
{
    double x = 0.0;
    double y = 0.0;
};


/**
 * @class SerialPointSender
 * @brief ファイルから読み取った座標データをシリアル通信で送信するクラス
 */
class SerialPointSender
{
public:
    /**
     * @brief コンストラクタ
     * @param port_name シリアルポートのパス (例: "/dev/ttyUSB0")
     */
    SerialPointSender(const std::string &port_name)
        : port_name_(port_name),
          serial_port_fd_(-1) // シリアルポートを無効な値で初期化
    {
        initSerialPort();
    }

    /**
     * @brief デストラクタ
     */
    ~SerialPointSender()
    {
        if (serial_port_fd_ != -1)
        {
            std::cout << "シリアルポートを閉じます。" << std::endl;
            close(serial_port_fd_);
        }
    }
     /**
     * @brief メインの処理ループを実行する関数
     * @param filename 読み込む座標データファイル名
     * @param interval_milliseconds 処理の実行間隔（ミリ秒）
     */
    void run(const std::string &filename, int interval_milliseconds)
    {
        if (serial_port_fd_ < 0)
        {
            std::cerr << "エラー: シリアルポートが初期化されていません。処理を中断します。" << std::endl;
            return;
        }

        while (true)
        {
            Point current_point;
            // ファイルから座標を読み込む
            if (readPointFromFile(filename, current_point))
            {
                // 読み込んだ座標を処理（シリアル送信）する
                processAndSendPoint(current_point);
            }
            else
            {
                // ファイルが読み込めなかった場合
                std::cout << "座標データなし、またはファイルの読み込みに失敗しました。" << std::endl;
            }

            // 指定された時間だけ待機
            std::this_thread::sleep_for(std::chrono::milliseconds(interval_milliseconds));
        }
    }

private:
    std::string port_name_;
    int serial_port_fd_;

    /**
     * @brief シリアルポートの初期化
     */
    void initSerialPort()
    {
        // シリアルポートを開く（読み書き両用）
        serial_port_fd_ = open(port_name_.c_str(), O_RDWR);
        if (serial_port_fd_ < 0)
        {
            std::cerr << "エラー: シリアルポート " << port_name_ << " を開けませんでした。" << std::endl;
            return;
        }

        // シリアルポートの設定を行う
        struct termios tty;
        if (tcgetattr(serial_port_fd_, &tty) != 0)
        {
            std::cerr << "エラー: ポート設定の取得に失敗しました (tcgetattr)。" << std::endl;
            return;
        }
        cfsetispeed(&tty, B115200); // ボーレートを115200に設定
        cfsetospeed(&tty, B115200);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        if (tcsetattr(serial_port_fd_, TCSANOW, &tty) != 0)
        {
            std::cerr << "エラー: ポート設定の適用に失敗しました (tcsetattr)。" << std::endl;
            return;
        }
        std::cout << "シリアルポート " << port_name_ << " を正常に開きました。" << std::endl;
    }

    /**
     * @brief ファイルから最初の座標点を読み込む
     * @param filename 読み込むファイル名
     * @param out_point 読み取った座標を格納するPoint構造体
     * @return 読み込みに成功した場合はtrue、失敗した場合はfalse
     */
    bool readPointFromFile(const std::string &filename, Point &out_point)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            return false;
        }

        int point_count = 0;
        file >> point_count; // 1行目のポイント数を読み込み

        std::string dummy_line;
        std::getline(file, dummy_line); // 1行目の残りの改行を読み飛ばす

        if (point_count > 0)
        {
            std::string line;
            if (std::getline(file, line)) // 2行目の座標データを読み込み
            {
                std::stringstream ss(line);
                char comma;
                if (ss >> out_point.x >> comma >> out_point.y)
                {
                    return true; // 座標の抽出に成功
                }
            }
        }
        return false;
    }

    /**
     * @brief 座標を文字列に変換し、シリアル送信する
     * @param point 送信する座標データ
     */
    void processAndSendPoint(const Point &point)
    {

        std::cout << "変換前の元データ  : X=" << point.x << ", Y=" << point.y << std::endl; 
        // --- 座標のマッピング（スケール変換） ---
        int panel_x = point.x + 8;
        int panel_y = point.y;

        panel_x  = PANEL_RES_Y - ((panel_x / WORLD_MAX_X ) * PANEL_RES_Y);
        panel_y  = PANEL_RES_X -  ((panel_y / WORLD_MAX_Y ) * PANEL_RES_X);

        
        

        // 送信するデータを "x座標,y座標\n" の形式の文字列に変換します。
        std::stringstream ss;
        ss << panel_y << "," << panel_x << "\n";
        std::string data_to_send = ss.str();

        // コンソールに送信するデータを表示
        std::cout << "変換後の送信データ: " << data_to_send;

        // シリアルポートにデータを送信
        writeSerial(data_to_send);
    }

    /**
     * @brief シリアルポートに文字列データを書き込む
     * @param data 送信する文字列
     */
    void writeSerial(const std::string &data)
    {
        if (serial_port_fd_ != -1)
        {
            // 文字列の先頭から、その長さ分のデータを書き込む
            ssize_t bytes_written = write(serial_port_fd_, data.c_str(), data.length());
            if (bytes_written < 0)
            {
                std::cerr << "エラー: シリアルポートへの書き込みに失敗しました。" << std::endl;
            }
        }
    }
};

// --- メインの処理 ---
int main()
{
    try
    {
        // SerialPointSenderオブジェクトを作成
        // ポート名はconfig.hで定義されたPORT_NAMEを使用します
        SerialPointSender sender(PORT_NAME);

        // runメソッドを呼び出して処理を開始
        // 読み込むファイル名として config.h で定義した FILENAME を使用します
        sender.run(FILENAME, WAIT_MILLISECONDS); // ← ここを変更しました
    }
    catch (const std::exception& e)
    {
        std::cerr << "予期せぬエラーが発生しました: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
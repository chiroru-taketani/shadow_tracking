#include <iostream>
#include <opencv2/opencv.hpp>

// ★ ウィンドウサイズに合わせて画像をリシェイプ（リサイズ＋中心線描画）する関数
void reshape(cv::Mat &img, int width, int height)
{
    // 1. 新しいサイズで黒い画像を再生成
    img = cv::Mat::zeros(height, width, CV_8UC3);

    // 2. 新しいサイズを元に中心座標を計算
    int mid_x = img.cols / 2;
    int mid_y = img.rows / 2;
    cv::Scalar color_cyan(255, 255, 0); // シアン色

    // 3. 中心線を描画
    cv::line(img, cv::Point(mid_x, 0), cv::Point(mid_x, img.rows), color_cyan, 2);
    cv::line(img, cv::Point(0, mid_y), cv::Point(img.cols, mid_y), color_cyan, 2);
}

int main()
{
    std::string win_name = "Reshape Window Demo";

    cv::namedWindow(win_name, cv::WINDOW_NORMAL);

    // ★ MacのRetinaディスプレイによるサイズズレ（右側や下の余白）を防ぐ魔法の1行
    cv::setWindowProperty(win_name, cv::WND_PROP_ASPECT_RATIO, cv::WINDOW_FREERATIO);

    cv::resizeWindow(win_name, 640, 480);

    // 初期サイズで画像を用意
    cv::Mat img;
    reshape(img, 640, 480);

    // 前回のサイズを記憶しておく変数（サイズ変更があったかどうかの判定用）
    int last_width = 640;
    int last_height = 480;

    bool is_fullscreen = false;

    std::cout << "【操作方法】" << std::endl;
    std::cout << " F キー : フルスクリーンの切り替え（別ディスプレイ対応）" << std::endl;
    std::cout << " Esc キー: 終了" << std::endl;

    while (true)
    {
        // 現在のウィンドウサイズをチェック
        cv::Rect rect = cv::getWindowImageRect(win_name);
        int current_width = rect.width;
        int current_height = rect.height;

        // ウィンドウが正常なサイズを持っている場合のみ処理
        if (current_width > 0 && current_height > 0)
        {
            // ★ 前回のサイズから変更があった時だけ、リシェイプ関数を実行する
            if (current_width != last_width || current_height != last_height)
            {
                reshape(img, current_width, current_height);

                // 現在のサイズを記録
                last_width = current_width;
                last_height = current_height;

                std::cout << "リシェイプ発動: " << current_width << " x " << current_height << std::endl;
            }
        }

        // 画像を表示
        cv::imshow(win_name, img);

        int key = cv::waitKey(30);

        // Fキーでのフルスクリーン切り替え
        if (key == 'f' || key == 'F')
        {
            is_fullscreen = !is_fullscreen;

            if (is_fullscreen)
            {
                cv::setWindowProperty(win_name, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
            }
            else
            {
                cv::setWindowProperty(win_name, cv::WND_PROP_FULLSCREEN, cv::WINDOW_NORMAL);
                cv::resizeWindow(win_name, 640, 480);
            }
        }
        else if (key == 27)
        { // Esc
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}

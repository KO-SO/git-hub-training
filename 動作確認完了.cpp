#include <iostream>
#include<opencv2/opencv.hpp>
#include <cstdint>
#include <wiringPi.h>
#include <wiringSerial.h>

using namespace cv;
using namespace std;

int main() {

    /* シリアルポートオープン */
int fd = serialOpen("/dev/ttyAMA0",115200);        //引数は自分のに合わせて下さい

wiringPiSetup();
fflush(stdout);

if(fd<0){
    printf("can not open serialport");
}
//   /* const double slope = -60 / 640;*/
    Mat frame;
    //=====================動画の読み込み==========================================
    VideoCapture cam(0);

    if (!cam.isOpened()) {
        return -1;
    }

    while (cam.read(frame)) {

        //======================画像の読み込み=========================================
        //frame=imread("C:\\Users\\papip\\ドキュメント\\農工大学\\サークル\\redcorn.jpg");

        //if (frame.empty()) return -1;

        //======================赤色抽出===============================================
        Mat hsv;
        cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        // inRange によって Hue が特定の範囲にある領域の mask を取得します。
        Mat mask;//この範囲にある色は黒（２５５）、それ以外は0にしてmaskに返す
        inRange(hsv, Scalar(0, 70, 50), Scalar(10, 255, 255), mask);
        inRange(hsv, cv::Scalar(170, 70, 100), cv::Scalar(180, 255, 255), mask);

        //====================輪郭をとる===========================================

        cv::Mat blur;
        cv::blur(mask, blur, cv::Size(3, 3));

        // エッジ検出のアルゴリズムとして cv::Canny を利用します。t
        cv::Mat canny;
        int thresh = 100;
        cv::Canny(blur, canny, thresh, thresh * 2);

        // cv::findContours は第一引数を破壊的に利用するため imshow 用に別変数を用意しておきます。
        cv::Mat canny2 = canny.clone();

        // cv::Point の配列として、輪郭を計算します。
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(canny, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        //============輪郭の可視化=========================================================
         // 輪郭を可視化してみます。分かりやすさのため、乱数を利用して色付けします。
      cv::Mat drawing = cv::Mat::zeros(canny.size(), CV_8UC3);
        cv::RNG rng(12345);

        for (int i = 0; i < contours.size(); i++) {
            cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
            cv::drawContours(drawing, contours, (int)i, color);
        }
        //======輪郭の面積=====================================================================
        int maxArea = 0;
        int maxContourIndex = -1;   //最大の面積を持つ輪郭のカウンター
        for (int i = 0; i < contours.size(); i++) {
            int Area = contourArea(contours[i]);
            if (maxArea < Area) {
                maxArea = Area;
                maxContourIndex = i;
            }
        }
        int8_t Boolean;  //赤コーンを認識したかどうか
        int8_t Angle;  //重心の角度

        if (maxContourIndex > -1) {  //輪郭があります
            vector<Point2f> triangle;
            int minArea = 0;
            minArea = minEnclosingTriangle(contours[maxContourIndex], triangle);
            int AreaRatio = 100 * maxArea / minArea;  //輪郭と外接三角形の面積比
            int Threshold = 65;   //閾値を設定する
            
            if (AreaRatio < Threshold) {   //赤コーンではなかったです
                Boolean = 0;
                Angle = 0;
            }else {                       //赤コーンでした
                    Boolean = 1; 
                    Moments mu = moments(contours[maxContourIndex]);
                    int moment_x = mu.m10 / mu.m00;                   //重心のx座標
                    Angle = (-60 * moment_x) / frame.cols + 30; //重心の角度
            }
        }else {          //輪郭がありませんでした
            Boolean = 0;
            Angle = 0;
        }

        serialPutchar(fd, Boolean);  //データの送信
        serialPutchar(fd, Angle);  //データの送信


        /* cv::imshow("frame", frame);*/
        //cv::imshow("mask", mask);
        ///* cv::imshow("blur", blur);*/
        // /*cv::imshow("canny", canny2);*/
        //cv::imshow("drawing", drawing);
        FILE* fp = fopen("/home/pi/sample.txt", "a");  //ファイルに書き込み
        fprintf(fp, "%d%d\n,", Boolean,Angle);
        fclose(fp);
        
        
        //=========動画の場合=========================================================================
        const int key = waitKey(1);
        if (key == 'q') {
            break;
        }
    }

    destroyAllWindows();
}

//=========騾包ｽｻ陷剃ｸ? ?ｽｮ陜｣?ｽｴ陷ｷ ==========================================================================
//
//waitKey(0);
//destroyAllWindows();
//    }


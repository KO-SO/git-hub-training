#include <iostream>
#include<opencv2/opencv.hpp>
#include <cstdint>
#include <wiringPi.h>
#include <wiringSerial.h>

using namespace cv;
using namespace std;

int main() {

    /* �V���A���|�[�g�I�[�v�� */
int fd = serialOpen("/dev/ttyAMA0",115200);        //�����͎����̂ɍ��킹�ĉ�����

wiringPiSetup();
fflush(stdout);

if(fd<0){
    printf("can not open serialport");
}
//   /* const double slope = -60 / 640;*/
    Mat frame;
    //=====================����̓ǂݍ���==========================================
    VideoCapture cam(0);

    if (!cam.isOpened()) {
        return -1;
    }

    while (cam.read(frame)) {

        //======================�摜�̓ǂݍ���=========================================
        //frame=imread("C:\\Users\\papip\\�h�L�������g\\�_�H��w\\�T�[�N��\\redcorn.jpg");

        //if (frame.empty()) return -1;

        //======================�ԐF���o===============================================
        Mat hsv;
        cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        // inRange �ɂ���� Hue ������͈̔͂ɂ���̈�� mask ���擾���܂��B
        Mat mask;//���͈̔͂ɂ���F�͍��i�Q�T�T�j�A����ȊO��0�ɂ���mask�ɕԂ�
        inRange(hsv, cv::Scalar(150, 100, 100), cv::Scalar(180, 255, 255), mask);

        //====================�֊s���Ƃ�===========================================

        cv::Mat blur;
        cv::blur(mask, blur, cv::Size(3, 3));

        // �G�b�W���o�̃A���S���Y���Ƃ��� cv::Canny �𗘗p���܂��Bt
        cv::Mat canny;
        int thresh = 100;
        cv::Canny(blur, canny, thresh, thresh * 2);

        // cv::findContours �͑�������j��I�ɗ��p���邽�� imshow �p�ɕʕϐ���p�ӂ��Ă����܂��B
        cv::Mat canny2 = canny.clone();

        // cv::Point �̔z��Ƃ��āA�֊s���v�Z���܂��B
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(canny, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        //============�֊s�̉���=========================================================
         // �֊s���������Ă݂܂��B������₷���̂��߁A�����𗘗p���ĐF�t�����܂��B
      cv::Mat drawing = cv::Mat::zeros(canny.size(), CV_8UC3);
        cv::RNG rng(12345);

        for (int i = 0; i < contours.size(); i++) {
            cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
            cv::drawContours(drawing, contours, (int)i, color);
        }
        //======�֊s�̖ʐ�=====================================================================
        int maxArea = 0;
        int maxContourIndex = -1;   //�ő�̖ʐς����֊s�̃J�E���^�[
        for (int i = 0; i < contours.size(); i++) {
            int Area = contourArea(contours[i]);
            if (maxArea < Area) {
                maxArea = Area;
                maxContourIndex = i;
            }
        }
        int8_t Boolean;  //�ԃR�[����F���������ǂ���
        int8_t Angle;  //�d�S�̊p�x

        if (maxContourIndex > -1) {  //�֊s������܂�
            vector<Point2f> triangle;
            int minArea = 0;
            minArea = minEnclosingTriangle(contours[maxContourIndex], triangle);
            int AreaRatio = 100 * maxArea / minArea;  //�֊s�ƊO�ڎO�p�`�̖ʐϔ�
            int Threshold = 65;   //臒l��ݒ肷��
            
            if (AreaRatio < Threshold) {   //�ԃR�[���ł͂Ȃ������ł�
                Boolean = 0;
                Angle = 0;
            }else {                       //�ԃR�[���ł���
                    Boolean = 1; 
                    Moments mu = moments(contours[maxContourIndex]);
                    int moment_x = mu.m10 / mu.m00;                   //�d�S��x���W
                    Angle = (-60 * moment_x) / frame.cols + 30; //�d�S�̊p�x
            }
        }else {          //�֊s������܂���ł���
            Boolean = 0;
            Angle = 0;
        }

        serialPutchar(fd, Boolean);  //�f�[�^�̑��M
        serialPutchar(fd, Angle);  //�f�[�^�̑��M


        /* cv::imshow("frame", frame);*/
        //cv::imshow("mask", mask);
        ///* cv::imshow("blur", blur);*/
        // /*cv::imshow("canny", canny2);*/
        //cv::imshow("drawing", drawing);
        FILE* fp = fopen("/home/pi/sample.txt", "a");  //�t�@�C���ɏ�������
        fprintf(fp, "%d%d\n,", Boolean,Angle);
        fclose(fp);
        
        
        //=========����̏ꍇ=========================================================================
        const int key = waitKey(1);
        if (key == 'q') {
            break;
        }
    }

    destroyAllWindows();
}

//=========逕ｻ蜒�? ?��蝣?��蜷 ==========================================================================
//
//waitKey(0);
//destroyAllWindows();
//    }


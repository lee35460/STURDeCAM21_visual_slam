/******************************************************************************
 *
 * Copyright (C) 2023 - 2028 KETI, All rights reserved.
 *                           (Korea Electronics Technology Institute)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * Use of the Software is limited solely to applications:
 * (a) running for Korean Government Project, or
 * (b) that interact with KETI project/platform.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * KETI BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Except as contained in this notice, the name of the KETI shall not be used
 * in advertising or otherwise to promote the sale, use or other dealings in
 * this Software without prior written authorization from KETI.
 *
 ******************************************************************************/

#include "vo_feature.h"

using namespace cv;
using namespace std;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000
// #define FOCAL_LENGTH 718.8560
#define FOCAL_LENGTH 23.55
#define PRINCIPAL_POINT_X 1920 / 2
#define PRINCIPAL_POINT_Y 1080 / 2

void drawTrajectory(Mat &traj, const Mat &translation)
{
    int x = int(translation.at<double>(0)) + 300;
    int y = int(translation.at<double>(2)) + 100;
    circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);

    char text[100];
    rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0), FILLED);
    sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", translation.at<double>(0), translation.at<double>(1), translation.at<double>(2));
    putText(traj, text, Point(10, 50), FONT_HERSHEY_PLAIN, 1, Scalar::all(255), 1, 8);
}

int main(int argc, char **argv)
{

    Mat img_1, img_2;
    Mat img_1_c, img_2_c;
    Mat prevImage, currImage, currImage_c;

    vector<Point2f> prevFeatures, currFeatures;

    double scale = 0.9;
    char text[100];
    int fontFace = FONT_HERSHEY_PLAIN;
    double fontScale = 1;
    int thickness = 1;
    cv::Point textOrg(10, 50);

    // string pipline = "v4l2src device=/dev/video0 ! videoconvert ! appsink";
    // VideoCapture cap(pipline, CAP_GSTREAMER);
    VideoCapture cap(0);

    if (!cap.isOpened())
    {
        cerr << "Stream is not open." << endl;
        return -1;
    }

    namedWindow("Road facing camera", WINDOW_AUTOSIZE);
    namedWindow("Trajectory", WINDOW_AUTOSIZE);

    Mat frame;
    Mat traj = Mat::zeros(600, 600, CV_8UC3);
    int i = 0;
    double focal = 718.8560;
    cv::Point2d pp(607.1928, 185.2157);
    vector<uchar> cv_status;
    Mat E, R, t, mask;
    clock_t begin = clock();
    Mat R_f = Mat::eye(3, 3, CV_64F), t_f = Mat::zeros(3, 1, CV_64F);

    while (true)
    {
        cap >> frame;

        if (frame.empty())
        {
            cerr << "Receiving an Empty Frame. End stream" << endl;
            break;
        }

        if (i == 0)
        {
            cvtColor(frame.clone(), prevImage, COLOR_BGR2GRAY);
            currImage = prevImage.clone();
            featureDetection(prevImage, prevFeatures);
        }

        else
        {
            cvtColor(frame.clone(), currImage, COLOR_BGR2GRAY);
            featureDetection(currImage, currFeatures);

            if (prevFeatures.size() != 0 && currFeatures.size() != 0)
            {
                if (!currImage.isContinuous() || !prevImage.isContinuous())
                {
                    cout << "Image is not continuous" << endl;
                    return -1;
                }
                featureTracking(prevImage, currImage, prevFeatures, currFeatures, cv_status);
                try
                {
                    if (!prevFeatures.empty() && !currFeatures.empty() && prevFeatures.size() == currFeatures.size())
                    {
                        E = findEssentialMat(currFeatures, prevFeatures, FOCAL_LENGTH, pp, RANSAC, 0.999, 1.0, mask);
                        recoverPose(E, currFeatures, prevFeatures, R, t, FOCAL_LENGTH, pp, mask);
                        if (i > 1)
                        {
                            if (!prevFeatures.empty() && scale > 0.1 && t.at<double>(2) > t.at<double>(0) && t.at<double>(2) > t.at<double>(1))
                            {
                                t_f += scale * (R_f * t);
                                R_f *= R;
                            }
                        }
                    }
                    else
                    {
                        throw std::runtime_error("포인트 배열이 비어 있거나 크기가 서로 다릅니다.");
                    }
                }
                catch (const cv::Exception &e)
                {
                    cerr << "OpenCV Exception: " << e.what() << endl;
                    // 여기에서 추가적인 오류 처리를 수행할 수 있습니다.
                    // 예를 들어, 실패한 경우 대체 로직을 실행하거나 사용자에게 알림 등
                }
                catch (const std::runtime_error &e)
                {
                    cerr << "Runtime Error: " << e.what() << endl;
                    // 비어 있거나 크기가 다른 배열 관련 오류 처리
                }
                catch (...)
                {
                    cerr << "알 수 없는 예외가 발생했습니다." << endl;
                    // 기타 모든 종류의 예외 처리
                }
            }
            if (prevFeatures.size() < MIN_NUM_FEAT)
            {
                featureDetection(prevImage, prevFeatures);
                featureTracking(prevImage, currImage, prevFeatures, currFeatures, cv_status);
            }

            prevImage = currImage.clone();
            prevFeatures = currFeatures;
        }

        drawTrajectory(traj, t_f);
        imshow("Road facing camera", currImage);
        imshow("Trajectory", traj);
        if (waitKey(1) == 'q')
            break;

        ++i;
    }

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "Total time taken: " << elapsed_secs << "s" << endl;
    cap.release();
    destroyAllWindows();
    return 0;
}
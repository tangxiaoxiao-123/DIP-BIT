//myCanny.cpp
#include "myCanny.h"

myCanny::myCanny(int low, int high)
{
    low_threshold = low;
    high_threshold = high;
}

//SobelFilter
//input: imgSrc
//output:gradX,gradY,gradXY,theta
void myCanny::mySobelFilter(const Mat &imgSrc)
{
    gradX = Mat::zeros(imgSrc.size(), CV_32SC1);
    gradY = Mat::zeros(imgSrc.size(), CV_32SC1);
    gradXY = Mat::zeros(imgSrc.size(), CV_32SC1);
    theta = Mat::zeros(imgSrc.size(), CV_32SC1);

    int height = imgSrc.rows;
    int width = imgSrc.cols;
    //gradient operator
    const int SobelX[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
    const int SobelY[9] = {1, 2, 1, 0, 0, 0, -1, -2, -1};

    //Memory occupied by the actual row of the image
    int step = imgSrc.step;
    int stepXY = gradX.step;

    //use ptr to improve speed
    uchar *PX = gradX.data;
    uchar *PY = gradY.data;
    uchar *PXY = gradXY.data;
    uchar *P = imgSrc.data;
    uchar *Ptheta = theta.data;

    for (int i = 1; i < height - 1; i++)
    {
        for (int j = 1; j < width - 1; j++)
        {
            double numX = 0, numY = 0;
            for (int k = 0; k < 9; k++)
            {
                double value = P[(i + X[k]) * step + j + Y[k]];
                numX += value * SobelX[k];
                numY += value * SobelY[k];
            }
            PX[i * stepXY + j * (stepXY / step)] = fabs(numX);
            PY[i * stepXY + j * (stepXY / step)] = fabs(numY);

            if (numX == 0)
                numX = 1e-16;
            PXY[i * stepXY + j * (stepXY / step)] = fabs(numX) + fabs(numY);
            Ptheta[i * stepXY + j * (stepXY / step)] = fabs(atan(numY / numX) * 57.3 + 90);
        }
    }
    //format transform
    gradX.convertTo(gradX, CV_8UC1);
    gradY.convertTo(gradY, CV_8UC1);
    gradXY.convertTo(gradXY, CV_8UC1);
    theta.convertTo(theta, CV_8UC1);
}

/*
    name: void myCanny::nonMaxSuppression
    content: non_max_suppresion
    input: img
    output: img

*/
void myCanny::nonMaxSuppression(Mat &imgDst)
{
    uchar *P = gradXY.data;
    int step = gradXY.step;
    uchar *P_theta = theta.data;

    imgDst = gradXY.clone();
    uchar *P_dst = imgDst.data;

    for (int i = 1; i < gradXY.rows - 1; i++)
    {
        for (int j = 1; j < gradXY.cols - 1; j++)
        {
            //if the value == 0 noprocessing
            if (P[i * step + j] == 0)
                continue;

            //process the neighboor
            uchar value[9] = {0};
            for (int k = 0; k < 9; k++)
                value[k] = P[(i + X[k]) * step + j + Y[k]];

            int angle = P_theta[i * step + j];

            // +-gradient direction value
            double P1 = 0;
            double P2 = 0;

            if (angle >= 0 && angle < 45)
            {
                double a = tan(angle);
                P1 = a * value[2] + (1 - a) * value[5];
                P2 = a * value[6] + (1 - a) * value[3];
            }

            else if (angle >= 45 && angle < 90)
            {
                double a = tan(90 - angle);
                P1 = a * value[2] + (1 - a) * value[1];
                P2 = a * value[6] + (1 - a) * value[7];

            }

            else if (angle == 90)
            {
                P1 = value[1];
                P2 = value[7];
            }

            else if (angle > 90 && angle < 135)
            {
                double a = tan(angle - 90);
                P1 = a * value[0] + (1 - a) * value[1];
                P2 = a * value[8] + (1 - a) * value[7];
            }
            else if (angle >= 135 && angle <= 180)
            {
                double a = tan(180 - angle);
                P1 = a * value[0] + (1 - a) * value[3];
                P2 = a * value[8] + (1 - a) * value[5];
            }
            if (value[4] < P1 || value[4] < P2)
                P_dst[i * step + j] = 0;
        }
    }
    //imshow("img_NMS",P_dst)
}

/*
    name: myCanny::twoThresholdLink
    input: imgSrc -> image source
    output: imgDst
*/
void myCanny::twoThresholdLink(const Mat &imgSrc, Mat &imgDst)
{
    uchar *P = imgSrc.data;
    int step = imgSrc.step;

    imgDst = imgSrc.clone();
    uchar *P_dst = imgDst.data;

    // > high == 255
    // < low == 0
    // low-high propcess
    for (int i = 0; i < imgSrc.rows; i++)
    {
        for (int j = 0; j < imgSrc.cols; j++)
        {
            if (P[i * step + j] > high_threshold)
            {
                P_dst[i * step + j] = 255;
            }
            else if (P[i * step + j] < low_threshold)
            {
                P_dst[i * step + j] = 0;
            }
            else
            {
                for (int k = 0; k < 9; k++)
                {
                    if (i + X[k] < 0 || i + X[k] >= imgSrc.rows || j + Y[k] < 0 || j + Y[k] >= imgSrc.cols)
                        continue;
                    if (P[(i + X[k]) * step + j + Y[k]] > high_threshold)
                    {
                        P_dst[i * step + j] = 255;
                    }
                    else
                    {
                        P_dst[i * step + j] = 0;
                    }
                }
            }
        }
    }
    
    
}

void cannyProcess(const Mat &imgSrc, int low, int high, Mat &imgDst)
{
    myCanny my_canny(low, high);
    //1、Sobel
    my_canny.mySobelFilter(imgSrc);
    //2、no-max-suppresion
    Mat imgDst_NMS;
    my_canny.nonMaxSuppression(imgDst_NMS);
    //imshow("img_NMS",imgDst_NMS);
    //3、two-threshold-link
    my_canny.twoThresholdLink(imgDst_NMS, imgDst);
}
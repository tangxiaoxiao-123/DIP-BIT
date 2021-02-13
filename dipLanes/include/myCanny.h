/*
    author: TangXiaojuan
    date: 2021-02-10
    content: get edge
    process: 1、Sobel->gradientX,gradient,angle
             2、Non-maximum suppression
             3、two-threshold-link
    
*/
#ifndef __MYCANNY_H__
#define __MYCANNY_H__
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

//eight neighbor
const int X[9] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};
const int Y[9] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};

class myCanny
{
public:
    myCanny(int, int);
    void mySobelFilter(const Mat &);
    void nonMaxSuppression(Mat &);
    void twoThresholdLink(const Mat&,Mat&);

private:
    int low_threshold;
    int high_threshold;
    Mat gradX;
    Mat gradY;
    Mat gradXY;
    Mat theta;
};

void cannyProcess(const Mat &imgSrc,int low,int high, Mat &imgDst);

#endif
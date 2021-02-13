
//myGuass.h
/*
    author: TangXiaojuan
    date: 2021-02-10
    content: GuassFilter to fuzzy image

*/
#ifndef __MYGUASS_H__
#define __MYGUASS_H__

#include <opencv2/opencv.hpp>

using namespace cv;


class myGuass
{
public:
    myGuass(int, double);
    void mygetGuassKernel();
    void myFilter(const Mat &, Mat &);
    ~myGuass();

private:
    double **mykernel;
    int size;
    double sigma;
};

#endif

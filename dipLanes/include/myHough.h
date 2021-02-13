//myHough.h
/*
    author: TangXiaojuan
    date: 2021-02-10
    content: Hough Transformer to get lines

*/
#ifndef __MYHOUGH_H__
#define __MYHOUGH_H__
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;



//p and angle
struct TWO{
    int theta;
    int rho;
};

class myHough{
public:
    myHough(int,int);
    void vote(const Mat& img);
    vector<TWO> NMS(int);
    int max_num(TWO&);
    
    ~myHough();

private:
    int **hough_table;   //vote result
    int rho_max;         
    int angle_max;
    int pix_num;        // how many pixes whose value is 255

};

/*
    content: the total hough process
    process: 1、initial rho_max&angle_max to malloc space
             2、vote -> hough_table
             3、NMS -> the Nth max lanes
      input: 1、img -> image source
             2、N -> the Nth max
             3、rh_max -> rho_max
             4、ag_max ->angle_max
     output: 1、indt -> (rho & theta) the Nth max lanes

*/
vector<TWO> houghProcess(const Mat& img,int N,int rh_max,int ag_max);

#endif
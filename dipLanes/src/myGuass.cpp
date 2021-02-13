#include "myGuass.h"


const double PI = 4.0 * atan(1.0);
myGuass::myGuass(int siz, double sig)
{
    size = siz;
    sigma = sig;

    //malloc space
    mykernel = new double *[size];
    for (int i = 0; i < size; i++)
        mykernel[i] = new double[size];

    //zero initial
    for (int i = 0; i < size; i++)
        for (int j = 0; j < size; j++)
            mykernel[i][j] = 0;
    
}

//get guasskernel
//process mykernel
void myGuass::mygetGuassKernel()
{
    int center = size/2;
    double sum = 0;
    for(int i=0;i<size;i++)
    {
        for(int j=0;j<size;j++)
        {
            double dis = (i - center) * (i - center) + (j - center) * (j - center);
            mykernel[i][j] = 1 / (sigma * sigma * 2 * PI) * exp(-dis / (2 * sigma * sigma));
            sum += mykernel[i][j];
        }
    }
    for(int i=0;i<size;i++)
    {
        for(int j=0;j<size;j++)
        {
            mykernel[i][j] /= sum;
        }
    }

/*
    //test print
    for(int i=0;i<size;i++)
    {
        for(int j=0;j<size;j++)
        {
            cout<<mykernel[i][j]<<" ";
        }
        cout<<endl;
    }
*/

}

/*
    content: Filter
    input:imgSrc
    output:imgDst
*/
void myGuass::myFilter(const Mat& imgSrc,Mat& imgDst)
{
    //smooth the kernel
    int index=0;
    double* guasArray = new double[size*size];
    
    for(int i=0;i<size;i++)
        for(int j=0;j<size;j++)
            guasArray[index++] = mykernel[i][j];
    
    int step = imgSrc.step;
    uchar* P = imgSrc.data;
    uchar* P_dst = imgDst.data;
    //filter
    for (int row = 0; row < imgSrc.rows; row++)
    {
        for (int col = 0; col < imgSrc.cols; col++)
        {
            int k = 0;
            double sum = 0;
            for (int i = -size / 2; i <= size / 2; i++)
            {
                for (int j = -size / 2; j <= size / 2; j++)
                {
                    int drow = i + row;
                    drow = drow < 0 ? 0 : drow;
                    drow = drow >= imgSrc.rows ? imgSrc.rows - 1 : drow;

                    int dcol = j + col;
                    dcol = dcol < 0 ? 0 : col;
                    dcol = dcol >= imgSrc.cols ? imgSrc.cols - 1 : dcol;

                    //sum += guasArray[k++] * (double)imgSrc.at<uchar>(drow, dcol);
                    sum += guasArray[k++] * (double)P[drow*step+dcol];
                }
            }
            //process the pixes out of range(0-255)
            if(sum<0)
                P_dst[row*step+col] = 0;
            else if(sum>255)
                P_dst[row*step+col] = 255;
            else
                P_dst[row*step+col] = (uchar)sum;
        }
    }
    
}
myGuass::~myGuass()
{
    //free space
    if(!mykernel)
    {
        for(int i=0;i<size;i++)
            delete[] mykernel[i];
        delete[] mykernel;
    }
}
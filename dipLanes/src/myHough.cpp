//myHough.cpp
#include"myHough.h"

const double PI = 4.0 * atan(1.0);

//choose the max 
int myHough::max_num(TWO& resxy)
{
    int my_max = 0;
    for(int theta=0;theta<angle_max;theta++)
    {
        for(int rho=0;rho<rho_max*2;rho++)
        {
            if(my_max<hough_table[theta][rho])
            {
                my_max = hough_table[theta][rho];
                resxy.theta = theta;
                resxy.rho = rho;
                
            }
        }
    }
    return my_max;
}


myHough::myHough(int rh_max,int ag_max)
{
    angle_max = ag_max;
    pix_num = 0;

    //malloc space
    rho_max = rh_max;
    hough_table = new int* [ag_max];
    for(int i=0;i<angle_max;i++)
        hough_table[i] = new int[2*rho_max];
    
    //zero initial
    for(int i=0;i<angle_max;i++)
    {
        for(int j=0;j<rho_max*2;j++)
        {
            hough_table[i][j] = 0;
        }
    }


}


//vote to rho&theta
/*
    nmae: myHough::vote()
    input:imgSrc
    output:hough_table、pix_num

*/
void myHough::vote(const Mat& imgSrc)
{
    uchar* P = imgSrc.data;
    int step = imgSrc.step;
    for(int y = 0;y<imgSrc.rows;y++)
    {
        for(int x=0;x<imgSrc.cols;x++)
        {
            if(P[y*step+x] == 255)
            {
                pix_num ++;
                for(int theta = 0;theta<angle_max;theta++)
                {
                    double t = PI/180*theta;
                    int rho = x*cos(t) + y*sin(t);
                    hough_table[theta][rho+rho_max] ++;
                }
            }
        }
    }
}

//name: myHough::NMS()
//get the Nth max lanes
//input: N
//output:indt[]->(theta,rho)
vector<TWO> myHough::NMS(int N)
{
    vector<TWO> indt;
    int i=1;
    while(i<=N)
    {
        TWO max_index;
        int max_value = max_num(max_index);
        //if the votes too smaller
        if((max_value<=pix_num/35)&&(max_value <=25))
        {
            break;
        }

        bool ok = 0;
        //some lanes'slope are abnormal
        bool condition3 = abs(90-max_index.theta) <= 10;
        bool condition4 = abs(180-max_index.theta) <= 10;
        bool condition5 = abs(max_index.theta) <= 10;
        
        if(indt.empty())
        {
            i+=1;
            indt.push_back(max_index);
            hough_table[max_index.theta][max_index.rho] = 0;
            continue;
        }
        for(int m=0;m<indt.size();m++)
        {
            int theta = indt[m].theta;
            int rho = indt[m].rho;

            //the close lanes may be the same lane
            bool condition1 = abs(rho-max_index.rho)<=80;
            bool condition2 = abs(theta-max_index.theta) <= 10;

            

            if(condition1||condition2||condition3||condition4||condition5)
            {
                hough_table[max_index.theta][max_index.rho] = 0;
                ok = 1;
                break;
            }

        }
        if(!ok)
        {
            i+=1;
            indt.push_back(max_index);
            hough_table[max_index.theta][max_index.rho] = 0;
        }


    }
    return indt;
}

myHough::~myHough()
{
    for(int i=0;i<angle_max;i++)
    {
        delete[] hough_table[i];
    }
    delete[] hough_table;
}

/*  
    name: houghProcess()
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
vector<TWO> houghProcess(const Mat& img,int N,int rh_max,int ag_max)
{
    
    //1、rho_max、angle_max
    myHough my_hough(rh_max,ag_max);
    //2、vote
    my_hough.vote(img);
    //3、NMS
    vector<TWO> indt = my_hough.NMS(N);
    return indt;
}
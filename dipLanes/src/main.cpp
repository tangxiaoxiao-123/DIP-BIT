//main.cpp

/*  
    author: TangXiaojuan
    date:2021-02-10
    content:lanes detection

*/
#include "../jsoncpp/json.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>
#include "myGuass.h"
#include "myCanny.h"
#include "myHough.h"

using namespace cv;
using namespace Json;
using namespace std;

const char *FILENAME_R = "../accurate_result.json"; //read file
const char *FILENAME_W = "../myresult.json";        //write file
const double PI = 4.0 * atan(1.0);


/*
    content: rho&theta -> y_samples&lanes[y_samples]
    input: 1、imgSrc -> image source
           2、indt -> houghProcess's result
           3、rho_max -> sqrt(H*H+W*W)
           4、imgDst -> to see image effect
           5、data -> read jsonfile's data to get h_samples
   output: 1、lanes
*/
void inverse(Mat &imgSrc, vector<TWO> &indt, int rho_max, Mat &imgDst, Value &data, Value &lanes)
{
    int size = data["h_samples"].size();
    if (indt.empty())
    {
        Value lane;
        for (int i = 0; i < size; i++)
        {
            lane.append(-2);
        }
        lanes.append(lane);
    }
    else
    {
        for (int i = 0; i < indt.size(); i++)
        {
            Value lane;
            double t = PI / 180 * indt[i].theta;

            int rho = indt[i].rho;
            for (int yindex = 0; yindex < size; yindex++)
            {
                int y = data["h_samples"][yindex].asInt();
                if (y < 240)
                {
                    lane.append(-2);
                    continue;
                }
                int x = (int)((rho - rho_max - y * sin(t)) / cos(t));
                if (x >= imgSrc.cols || x < 0)
                {
                    lane.append(-2);
                    continue;
                }
                lane.append(x);
                //circle(imgDst, Point(x, y), 3, CV_RGB(255, 0, 0), 1);
                //imgDst.at<Vec3b>(100, 100) = Vec3b(0, 0, 255);
            }
            lanes.append(lane);
        }
    }
    /*for (int j = 0; j < lanes.size(); j++)
    {
        for (int m = 0; m < myjs.size; m++)
        {
            cout << lanes[j][m].asInt() << ",";
        }
        cout << endl;
    }*/
}



/*
    content: choose the region of interest to improve speed
    a:rows_low
    b:rows_high
    c:cols_low
    d:cols_high

*/
void region_of_interest(Mat &imgSrc, int a, int b, int c, int d)
{
    uchar *P = imgSrc.data;
    int step = imgSrc.step;
    for (int i = a; i < b; i++)
    {
        for (int j = c; j < d; j++)
        {
            imgSrc.at<uchar>(i, j) = 0;
            P[i * step + j] = 0;
        }
    }
}

/*
process:
    1、read image with gray
    2、GuassFilter
    3、CannyEdge
    4、Hough Transformer to get lines
input: image name、guassKernel、root(data)[h_samples]
output: lanes
*/
Value GetLanes(string image_name, myGuass &my_guass, Value &data)
{
    Value lanes;
    //1、read image with gray
    Mat imgSrc = imread(image_name, 0);

    //2、GuassFilter

    Mat imgGuass = imgSrc.clone();
    my_guass.myFilter(imgSrc, imgGuass);

    //3、Canny
    int low_threshold = 100;
    int high_threshold = 160;
    Mat imgCanny;
    cannyProcess(imgGuass,low_threshold,high_threshold,imgCanny);
    //Canny(imgGuass,imgCanny,80,160);

    //4、region_of_interest
    region_of_interest(imgCanny, 0, 7 * imgSrc.rows / 18, 0, imgSrc.cols);

    //5、hough transformer
    //the Nth max lanes
    int H = imgSrc.rows;
    int W = imgSrc.cols;
    int rho_max = int(sqrt(H * H + W * W));
    int angle_max = 180;
    int N = 5;
    vector<TWO> indt = houghProcess(imgCanny, N, rho_max, angle_max);
    

    //6、inverse(the rho&theta -> lanes)
    //主要是为了看效果
    Mat imgDst(imgSrc.rows, imgSrc.cols, CV_8UC3, Scalar::all(0));
    cvtColor(imgSrc, imgDst, COLOR_GRAY2BGR);
    inverse(imgSrc, indt, rho_max, imgDst, data, lanes);

    // imshow("imgSrc", imgSrc);
    // imshow("imgGuass", imgGuass);
    // imshow("imgCanny", imgCanny);
    // imshow("imgDst", imgDst);
    waitKey(0);
    return lanes;
}

int main()
{

    //read accurate_result.json to get image_name

    //open readfile
    Reader reader;
    ifstream ifs(FILENAME_R, ios::binary);
    if (!ifs.is_open())
    {
        cout << "Error opening " << FILENAME_R << " file" << endl;
        return 0;
    }

    //open writefile
    ofstream ofs;
    ofs.open(FILENAME_W, ios::out);
    if (!ofs.is_open())
    {
        cout << "ofs.is_open error" << endl;
        return 0;
    }

    //from jsonfile get image_name
    Value root;
    string data;
    int i = 0;

    //process every image
    int size = 5;
    double sigma = 1.5;
    myGuass my_guass(size, sigma);
    my_guass.mygetGuassKernel();
    double sum_time = 0;
    while (getline(ifs, data, '\n'))
    {

        if (!reader.parse(data, root))
        {
            cout << "reader.parse error" << endl;
            return 0;
        }
        i += 1;
        // if (i >= 2)
        //     return 0;
        string image_name = root["raw_file"].asString();
        cout << i << ": " << image_name << endl;

        //start processing image to get lanes
        double begin_time = clock();
        Value lanes = GetLanes("../"+image_name, my_guass, root);

        //write result to jsonfile
        double end_time = clock();
        double interval = (end_time - begin_time)/1000;
        sum_time += interval;
        //Format processing
        Value root;
        root["raw_file"] = image_name;
        root["lanes"] = lanes;
        root["run_time"] = interval;
        ofs << FastWriter().write(root);
        cout << "picture " << i << " process and save success" << endl;
        cout<<endl;
    }

    //close files
    ifs.close();
    ofs.close();

    cout<<"spend time "<<sum_time/100<<"s every image"<<endl;

    return 0;
}
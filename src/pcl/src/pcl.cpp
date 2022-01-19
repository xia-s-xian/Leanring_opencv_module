
#include <iostream>
#include <image_transport/image_transport.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <visualization_msgs/Marker.h>

//pcl filter
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//sobel edge detection
#include <opencv2/opencv.hpp>
#include <iostream>

//find canny
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "iostream"  

using namespace std;
using namespace cv;

int sobel_edge_detection()
{
  // Reading image
  Mat img = imread("left.jpg");
  // Display original image
  imshow("original Image", img);
  waitKey(0);

  // Convert to graycsale
  Mat img_gray;
  cvtColor(img, img_gray, COLOR_BGR2GRAY);
  // Blur the image for better edge detection
  Mat img_blur;
  GaussianBlur(img_gray, img_blur, Size(3,3), 0);

  // Sobel edge detection
  Mat sobelx, sobely, sobelxy;
  Sobel(img_blur, sobelx, CV_64F, 1, 0, 5);
  Sobel(img_blur, sobely, CV_64F, 0, 1, 5);
  Sobel(img_blur, sobelxy, CV_64F, 1, 1, 5);
  // Display Sobel edge detection images
  imshow("Sobel X", sobelx);
  waitKey(0);
  imshow("Sobel Y", sobely);
  waitKey(0);
  imshow("Sobel XY using Sobel() function", sobelxy);
  waitKey(0);

  // Canny edge detection
  Mat edges;
  Canny(img_blur, edges, 100, 200, 3, false);

  std::cout << edges <<std::endl;

  // Display canny edge detected image
  imshow("Canny edge detection", edges);
  waitKey(0);

  destroyAllWindows();
}

void find_contour()
{
    Mat imageSource=imread("test.jpg",0);  
    imshow("Source Image",imageSource);  
    Mat image;  
    GaussianBlur(imageSource,image,Size(3,3),0);  
    Canny(image,image,100,250);  
    vector<vector<Point>> contours;  
    vector<Vec4i> hierarchy;  
    findContours(image,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point());  
    Mat imageContours=Mat::zeros(image.size(),CV_8UC1);  
    Mat Contours=Mat::zeros(image.size(),CV_8UC1);  //绘制  
    for(int i=0;i<contours.size();i++)  
    {  
        //contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数  
        for(int j=0;j<contours[i].size();j++)   
        {  
            //绘制出contours向量内所有的像素点  
            Point P=Point(contours[i][j].x,contours[i][j].y);  
            Contours.at<uchar>(P)=255;  
        }  
  
        //输出hierarchy向量内容  
        char ch[256];  
        sprintf(ch,"%d",i);  
        string str=ch;  
        cout<<"向量hierarchy的第" <<str<<" 个元素内容为："<<endl<<hierarchy[i]<<endl<<endl;  
  
        //绘制轮廓  
        drawContours(imageContours,contours,i,Scalar(255),1,8,hierarchy);  
    }  
    imshow("Contours Image",imageContours); //轮廓  
    imshow("Point of Contours",Contours);   //向量contours内保存的所有轮廓点集  
    waitKey(0);  

}

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
using namespace cv;
using namespace std;
Mat src_gray;
int thresh = 100;
RNG rng(12345);
void thresh_callback(int, void* );

int base_edge_get_bbox(int argc, char** argv)
{
    CommandLineParser parser( argc, argv, "{@input | stuff.jpg | input image}" );
    Mat src = imread( samples::findFile( parser.get<String>( "@input" ) ) );
    if( src.empty() )
    {
        cout << "Could not open or find the image!\n" << endl;
        cout << "usage: " << argv[0] << " <Input image>" << endl;
        return -1;
    }
    cvtColor( src, src_gray, COLOR_BGR2GRAY );
    GaussianBlur( src_gray, src_gray, Size(3,3),0 );
    const char* source_window = "Source";
    namedWindow( source_window );
    imshow( source_window, src );
    const int max_thresh = 255;
    createTrackbar( "Canny thresh:", source_window, &thresh, max_thresh, thresh_callback );
    thresh_callback( 0, 0 );
    waitKey();
    return 0;
}

void thresh_callback(int, void* )
{
    Mat canny_output;
    Canny( src_gray, canny_output, thresh, thresh*2,3,false );
    imshow("canny_output", canny_output);
    imwrite("canny_output.jpg", canny_output);
    vector<vector<Point> > contours;
    findContours( canny_output, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>centers( contours.size() );
    vector<float>radius( contours.size() );
    for( size_t i = 0; i < contours.size(); i++ )
    {
        approxPolyDP( contours[i], contours_poly[i], 3, true );
        boundRect[i] = boundingRect( contours_poly[i] );
       
        minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
    }
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    std::cout << "width:" <<drawing.cols<< "height:"<< drawing.rows<<std::endl; 
    std::cout<< "contours:"<<contours.size()<<std::endl;

    for( size_t i = 0; i< contours.size(); i++ )
    {
        if((boundRect[i].width>30)&&(boundRect[i].height>30))
        {
          std::cout << "boundRect size" << boundRect[i].width<<" "<<boundRect[i].height <<" "<< boundRect[i].x<<" "<<boundRect[i].y<<std::endl;
          std::cout <<boundRect[i].tl()<< " "<< boundRect[i].br()<<std::endl;
          Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
          drawContours( drawing, contours_poly, (int)i, color );
          rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 1 );
          //circle( drawing, centers[i], (int)radius[i], color, 2 );
        }
    }
    imshow( "Contours", drawing );
    imwrite("Contours.jpg", drawing);
}



//将两张image 按权重进行叠加在一起
int image_fusion()
{
    cv::Mat image = cv::imread("1.jpg");
    cv::Mat image_disparity_conf = cv::imread("2.png");
    cv::Mat tmp = Mat(image_disparity_conf.size(), image_disparity_conf.type());
    cv::Mat fuison = Mat(image_disparity_conf.size(), image_disparity_conf.type());
 
    resize(image_disparity_conf,tmp,Size(),2,2);
    cv::addWeighted(image,0.5,tmp,0.5,0,fuison);
    
    cv::namedWindow("result");
    cv::imshow("result",fuison);
    imwrite("fusion.jpg", fuison);
    cv::waitKey();
    
    return 0;
}


void calDispWithBM(Mat imgL, Mat imgR, Mat &imgDisparity8U)
{
  Mat imgDisparity16S = Mat(imgL.rows, imgL.cols, CV_16S);
 
  //--Call the constructor for StereoBM
  cv::Size imgSize = imgL.size();
  int numberOfDisparities = ((imgSize.width / 8) + 15) & -16;
  cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16, 9);
 
  //--Calculate the disparity image
 
  /*
  左右视图的有效像素区域，一般由双目校正阶段的 cvStereoRectify 函数传递，也可以自行设定。
  一旦在状态参数中设定了 roi1 和 roi2，OpenCV 会通过cvGetValidDisparityROI 函数计算出视差图的有效区域，在有效区域外的视差值将被清零。
  */
  cv::Rect roi1, roi2;
  bm->setROI1(roi1);
  bm->setROI2(roi2);
 
  //==预处理滤波参数
  /*
  预处理滤波器的类型，主要是用于降低亮度失真（photometric distortions）、消除噪声和增强纹理等,
  有两种可选类型：CV_STEREO_BM_NORMALIZED_RESPONSE（归一化响应） 或者 CV_STEREO_BM_XSOBEL（水平方向Sobel算子，默认类型）,
  该参数为 int 型;
  */
  bm->setPreFilterType(1);
 
  /*预处理滤波器窗口大小，容许范围是[5,255]，一般应该在 5x5..21x21 之间，参数必须为奇数值, int 型*/
  bm->setPreFilterSize(5);
 
  /*预处理滤波器的截断值，预处理的输出值仅保留[-preFilterCap, preFilterCap]范围内的值，参数范围：1 - 31,int 型*/
  bm->setPreFilterCap(31);
 
  //==SAD 参数
  /*SAD窗口大小，容许范围是[5,255]，一般应该在 5x5 至 21x21 之间，参数必须是奇数，int 型*/
  bm->setBlockSize(9);
 
  /*最小视差，默认值为 0, 可以是负值，int 型*/
  bm->setMinDisparity(0);
 
  /*视差窗口，即最大视差值与最小视差值之差, 窗口大小必须是 16 的整数倍，int 型*/
  bm->setNumDisparities(64);
 
  //==后处理参数
  /*
  低纹理区域的判断阈值:
  如果当前SAD窗口内所有邻居像素点的x导数绝对值之和小于指定阈值，则该窗口对应的像素点的视差值为 0
  （That is, if the sum of absolute values of x-derivatives computed over SADWindowSize by SADWindowSize
  pixel neighborhood is smaller than the parameter, no disparity is computed at the pixel），
  该参数不能为负值，int 型;
  */
  bm->setTextureThreshold(10);
 
  /*
  视差唯一性百分比:
  视差窗口范围内最低代价是次低代价的(1 + uniquenessRatio/100)倍时，最低代价对应的视差值才是该像素点的视差，
  否则该像素点的视差为 0 （the minimum margin in percents between the best (minimum) cost function value and the second best value to accept
  the computed disparity, that is, accept the computed disparity d^ only if SAD(d) >= SAD(d^) x (1 + uniquenessRatio/100.) for any d != d*+/-1 within the search range ），
  该参数不能为负值，一般5-15左右的值比较合适，int 型*/
  bm->setUniquenessRatio(15);
 
  /*检查视差连通区域变化度的窗口大小, 值为 0 时取消 speckle 检查，int 型*/
  bm->setSpeckleWindowSize(300);
 
  /*视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零，int 型*/
  bm->setSpeckleRange(4);
 
  /*
  左视差图（直接计算得出）和右视差图（通过cvValidateDisparity计算得出）之间的最大容许差异。
  超过该阈值的视差值将被清零。该参数默认为 -1，即不执行左右视差检查。int 型。
  注意在程序调试阶段最好保持该值为 -1，以便查看不同视差窗口生成的视差效果。
  */
  bm->setDisp12MaxDiff(1);
 
  /*计算视差*/
  bm->compute(imgL, imgR, imgDisparity16S);
 
  //-- Check its extreme values
  double minVal; double maxVal;
  minMaxLoc(imgDisparity16S, &minVal, &maxVal);
 
  cout << minVal << "\t" << maxVal << endl;
 
  //--Display it as a CV_8UC1 image：16位有符号转为8位无符号
  imgDisparity16S.convertTo(imgDisparity8U, CV_8U, 255 / (numberOfDisparities*16.));
}
//版权声明：本文为CSDN博主「JoannaJuanCV」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
//原文链接：https://blog.csdn.net/zfjBIT/article/details/91429770

void stereo_bm_proc()
{
//--读取图像
  Mat imgL = imread("./data/image1.jpg", 0);
  Mat imgR = imread("./data/image2.jpg", 0);
  std::cout << "u img width:" << imgL.cols << std::endl;
  std::cout << "v img height:" << imgL.rows << std::endl;
  
  //--And create the image in which we will save our disparities
  Mat imgDisparity8U = Mat(imgL.rows, imgL.cols, CV_8UC1);
 
  calDispWithBM(imgL, imgR, imgDisparity8U);
  
  namedWindow("disparity", 0);
  cv::imshow("disparity",imgDisparity8U);
  cv::waitKey();
  imwrite("./data/image_disparity.jpg", imgDisparity8U);

}

int main( int argc, char** argv )
{
  //image_fusion();
  stereo_bm_proc();
  return 0;
}

#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include<sensor_msgs/image_encodings.h>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  
#include <sstream> 
#include <std_msgs/String.h>
#include <color_detect/BoundingBox.h>

#include<string>
#include <fstream>
#include <unistd.h> // Linux系统中

//cmakelists添加环境配置参数find_package(OpenCV REQUIRED)

using namespace cv;
using namespace std;


bool image_view ;
int video_device;

int width = 800; // 设置图像宽度
int height = 600; // 设置图像高度
int H = 200,S = 150,V = 150;

Mat frame;  //定义opencv形式图像参数
//Mat frame = Mat::zeros(1, 5, CV_32F);   // [0,0,0,0,0]
//Mat frame_copy;
//Mat frame_copy = Mat::ones(1, 5, CV_32F);   // [1,1,1,1,1]

Mat imghsv;//定义hsv画面
vector<Mat> hsvSplit;
Mat mask;
sensor_msgs::ImagePtr ros_msg;  //定义ros形式图像参数
color_detect::BoundingBox detect_msg;

// Scalar lower_red(130, 50, 50);
// Scalar upper_red(255, 245, 245); // 定义红色的HSV范围


void on_mouse(int event, int x, int y, int flags, void* userdata);

int main(int argc, char** argv)  
{  
    ros::init(argc, argv, "opencv_camera");  
    ros::NodeHandle nh;  
    image_transport::ImageTransport it(nh);  
    image_transport::Publisher image_pub = it.advertise("image", 10);  
    ros::Publisher boundingbox_pub = nh.advertise<color_detect::BoundingBox>("color_detect",10);

    nh.getParam("image_view", image_view);
    nh.getParam("video_device", video_device);
    nh.getParam("width", width);
    nh.getParam("height", height);
    nh.getParam("H", H);
    nh.getParam("S", S);
    nh.getParam("V", V);

    ROS_INFO("view:%d",image_view);

    ros::Rate loop_rate(5);  

    namedWindow("video");

    VideoCapture cap(video_device);  //dev/video0
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width); // 设置图像宽度
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height); // 设置图像高度
    if(!cap.isOpened())   
    {  
        ROS_ERROR("Can not opencv video device\n");  
        return 1;  
    }  

    int max = 0;
    while (ros::ok()) 
    {  

        Scalar lower_red(H - 30, S - 30, V - 30);
        Scalar upper_red(H + 30, S + 30, V + 30); // 定义红色的HSV范围

        cap >> frame;  //摄像头画面赋给frame
        if(!frame.empty()) //画面是否正常
        {  
            /*对图片二次处理*/

            cvtColor(frame, imghsv, COLOR_BGR2HSV);// 将图像转换为HSV颜色空间

            split(imghsv, hsvSplit);
		    equalizeHist(hsvSplit[2], hsvSplit[2]);
		    merge(hsvSplit, imghsv);

            inRange(imghsv, lower_red, upper_red, mask);//二值化红色部分

            Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
            morphologyEx(mask, mask, MORPH_OPEN, kernel);//开运算
            morphologyEx(mask, mask, MORPH_CLOSE, kernel);//闭运算
            
            GaussianBlur(mask, mask, Size(5, 5), 0);//高斯滤波
            Canny(mask, mask, 150, 100);//canny算子边缘检测

            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;
            findContours(mask,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,Point());  
            //ROS_INFO("个数为%d",int(contours.size()));
            std::vector<std::vector<cv::Point> >::const_iterator itc = contours.begin();
            std::vector<std::vector<cv::Point> >::const_iterator max_c = contours.begin();
            if(	(!contours.empty() && !hierarchy.empty()))
            {
                        //寻找最大面积的轮廓
                        while (itc != contours.end())
                        {
                           if( cv::contourArea(*itc) >  cv::contourArea(*max_c)) {
                                max_c = itc;
                            }
                            itc++;
                        }
            Rect boundRect = boundingRect(*max_c);
            circle(frame, Point(boundRect.x + boundRect.width/2, boundRect.y + boundRect.height/2), 5, Scalar(0,0,255), -1);
            rectangle(frame, Point(boundRect.x, boundRect.y), Point(boundRect.x + boundRect.width, boundRect.y + boundRect.height), Scalar( 0, 0, 255), 2);

            //ROS_INFO("x:%d,y:%d",boundRect.x+ boundRect.width/2, boundRect.y + boundRect.height/2);
            detect_msg.Class = "red";
            detect_msg.xmin = boundRect.x;
            detect_msg.xmax=boundRect.x + boundRect.width;
            detect_msg.ymin=boundRect.y;
            detect_msg.ymax= boundRect.y + boundRect.height;
            }
            else
            {

            }
        //circle(frame, Point(50, 50), 45, Scalar(0, 0, 220), -1, 8, 0);        

             //opencv转ros
            try
            {
                ros_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();  
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception:%s",e.what());
            }
            if(image_view)
            {
                imshow("video", frame);
                setMouseCallback("video", on_mouse, 0);
               // imshow("mask",mask);
               
                int c = waitKey(1);
		            if (c == 27) {//key:esc
			        ros::shutdown();
		        }
            }
            image_pub.publish(ros_msg);  
            boundingbox_pub.publish(detect_msg);
    	}  
    }
    cap.release();			//释放视频

    ros::spinOnce();  
    loop_rate.sleep();  
}  

void on_mouse(int event, int x, int y, int flags, void* userdata)
{
    if (event == EVENT_LBUTTONDOWN) //定义一个鼠标左键按下去的事件
    {
        H = imghsv.at<Vec3b>(y,x )[0];
        S = imghsv.at<Vec3b>(y, x)[1];
        V = imghsv.at<Vec3b>(y,x)[2];
        cout << "像素点的HSV值为："<< " H: " << H << " S: " << S << " V: " << V<<endl;
        //ROS_INFO("hsv=",imghsv.at(y,x));
        // char path[255];
        // getcwd(path, 255);
        //ROS_INFO("%s,%d",path,strlen(path));
        ofstream file("output.txt");
        if (file.is_open()) {
            file << "H: "<<H<<" S: " << S<< "  V: "<<V<< endl;
            ROS_INFO("成功保存参数");
            file.close();
        } else {
            ROS_ERROR("Failed to open file");
        }

    }
}

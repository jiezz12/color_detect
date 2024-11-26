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

//cmakelists添加环境配置参数find_package(OpenCV REQUIRED)

using namespace cv;
using namespace std;


bool image_view = 1;
int video_device;

int width = 800; // 设置图像宽度
int height = 600; // 设置图像高度

int main(int argc, char** argv)  
{  
    ros::init(argc, argv, "opencv_camera");  
    ros::NodeHandle nh("~");  
    image_transport::ImageTransport it(nh);  
    image_transport::Publisher image_pub = it.advertise("image", 10);  
    ros::Publisher boundingbox_pub = nh.advertise<color_detect::BoundingBox>("color_detect",10);


    nh.getParam("image_view", image_view);
    nh.getParam("video_device", video_device);

    ros::Rate loop_rate(5);  

    VideoCapture cap(video_device);  //dev/video0
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width); // 设置图像宽度
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height); // 设置图像高度
    if(!cap.isOpened())   
    {  
        ROS_ERROR("Can not opencv video device\n");  
        return 1;  
    }  

    Mat frame;  //定义opencv形式图像参数
    Mat imghsv;//定义hsv画面
    vector<Mat> hsvSplit;
    Mat mask;
    sensor_msgs::ImagePtr ros_msg;  //定义ros形式图像参数
    color_detect::BoundingBox detect_msg;
    Mat frame_copy;
    namedWindow("video");

    Scalar lower_red(130, 50, 50);
    Scalar upper_red(255, 245, 245); // 定义红色的HSV范围
    int max = 0;
    while (ros::ok()) 
    {  
        cap >> frame;  //摄像头画面赋给frame
        frame_copy = frame.clone();
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
            vector<double> Area(contours.size());
            if(contours.size() > 0 )
            {
                        //寻找最大面积的轮廓
                        for (int i = 1; i < contours.size(); i++) {
                            Area[i] = contourArea(contours[i]);
                            if (Area[i] > Area[max]) {
                                max = i;
                            }   
                        }
            Rect boundRect = boundingRect(Mat(contours[max]));
            circle(frame, Point(boundRect.x + boundRect.width/2, boundRect.y + boundRect.height/2), 5, Scalar(0,0,255), -1);
            ROS_INFO("x:%d,y:%d",boundRect.x+ boundRect.width/2, boundRect.y + boundRect.height/2);
            rectangle(frame, Point(boundRect.x, boundRect.y), Point(boundRect.x + boundRect.width, boundRect.y + boundRect.height), Scalar( 0, 0, 255), 2);
            detect_msg.Class = "red";
            detect_msg.xmin = boundRect.x;
            detect_msg.xmax=boundRect.x + boundRect.width;
            detect_msg.ymin=boundRect.y;
            detect_msg.ymax= boundRect.y + boundRect.height;
            }
            else{
                frame = frame_copy;
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
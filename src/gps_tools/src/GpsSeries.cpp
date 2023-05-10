
#include <regex>
#include <string>
#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <serial/serial.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

using namespace std;

struct Location
{
  double lat;    // 纬度
  double lon;    // 经度
  double alt;    // 海拔
  double hdop;   // 水平精度因子
  double vdop;   // 高程精度因子，暂时没用
  int fix;       // 固定解标识
};

template <class Type>    
Type stringToNum(const string str)   
{ 
  Type num;
  if(str.empty()) return num;
	istringstream iss(str);       
	iss >> num;       
	return num;       
} 

class GPSMsgForward{
public:
  serial::Serial sp_;
  serial::Timeout timeout_ = serial::Timeout::simpleTimeout(100);
  bool flag_opened;
  std::string setPortName_;      // "/dev/ttyUSB0"
  std::string gpsTopic_;
  std::string gpsOdomTopic_;
  ros::NodeHandle nh_;
  ros::NodeHandle privateNh_;
  ros::Publisher pubGPSRaw_;
  ros::Publisher pubGpsOdom_; 
  nav_msgs::Odometry gpsOdomData_;
  sensor_msgs::NavSatFix gpsdata_;

public:
  GPSMsgForward(ros::NodeHandle& nh, ros::NodeHandle& privateNh): nh_(nh), privateNh_(privateNh)
  {
    privateNh.param<std::string>("gps_topic", gpsTopic_, "/gps/raw_data");
    privateNh.param<std::string>("gps_odom_topic", gpsOdomTopic_, "/global/gps/odom");
    privateNh.param<std::string>("serial_port", setPortName_, "/dev/ttyACM0");
    pubGPSRaw_  = nh_.advertise<sensor_msgs::NavSatFix>(gpsTopic_, 3);
    pubGpsOdom_ = nh_.advertise<nav_msgs::Odometry>(gpsOdomTopic_, 3);
    flag_opened = init();
    gpsOdomData_.header.frame_id = "earth";
    gpsOdomData_.child_frame_id = "gps";
  } 

  ~GPSMsgForward(){}

  bool init(){
    sp_.setPort(setPortName_);
    //设置串口通信的波特率
    sp_.setBaudrate(57600);
    //串口设置timeout
    sp_.setTimeout(timeout_);

    try{
        sp_.open();
    }
    catch(serial::IOException& e)
    {
      ROS_ERROR_STREAM("Unable to open port: " + setPortName_);
      return false;
    }

    //判断串口是否打开成功
    if(sp_.isOpen()) ROS_INFO_STREAM(setPortName_ + "is opened.");
    else return false;

    return true;
  }

  void coordinateTrans(const Location& result){
    double A_EARTH = 6378137.0;
    double flattening = 1.0/298.257223563;
    double NAV_E2 = (2.0-flattening)*flattening;
    double slat = sin(result.lat*M_PI/180);
    double clat = cos(result.lat*M_PI/180);
    double r_n = A_EARTH/sqrt(1.0 - NAV_E2*slat*slat);
    gpsOdomData_.header.stamp = gpsdata_.header.stamp;
    gpsOdomData_.pose.covariance[0] = gpsdata_.position_covariance[0];
    gpsOdomData_.pose.covariance[7] = gpsdata_.position_covariance[0];
    gpsOdomData_.pose.covariance[14] = 1.5 ;
    gpsOdomData_.pose.pose.position.x = (r_n + result.alt)*clat * cos(result.lon*M_PI/180);
    gpsOdomData_.pose.pose.position.y = (r_n + result.alt)*clat * sin(result.lon*M_PI/180);
    gpsOdomData_.pose.pose.position.z = (r_n*(1.0 - NAV_E2) + result.alt)*slat;
  }

  Location getGNGGA(std::string& msg)
  {
    Location result;
    auto position  = msg.find("$GNGGA");
    if( position == msg.npos )
        return result;
    auto endPosition = msg.find("\n", position);
    size_t length = endPosition - position - 1;               // C++ 中 "/n" 的长度很迷惑！所以不要 "/n"
    std::string gngga = msg.substr(position, length) + "\n";  // 一定注意手动添加 "/n" 结尾
    // std::cout << gngga ;
    vector<string> arrs;   
    do 
    {   
      string tmp_s;    
      position = gngga.find(",");   
      tmp_s = gngga.substr(0,position);   
      gngga.erase(0,position+1);    
      arrs.push_back(tmp_s);   
    }while(position != -1); 
    if(arrs[2].empty())
    {
      return result;
    }
    {
      result.lat = stringToNum<double>(arrs[2]) / 100.0;
      result.lon = stringToNum<double>(arrs[4]) / 100.0;
      result.fix = stringToNum<int>(arrs[6]);
      result.hdop = stringToNum<double>(arrs[8]);
      result.alt = stringToNum<double>(arrs[9]);
    }
    return result;
  }

  double NMEA2float(std::string s)
  {
    double d = std::stod(s)/100.0;
    d = floor(d) + (d - floor(d))*10/6;
    return d;
  }

  void fillSatMessage(sensor_msgs::NavSatFix& sat,const Location& loc )
  {
    sat.header.frame_id = "earth";
    sat.header.stamp = ros::Time::now();

    sat.status.status = loc.fix;

    sat.latitude = loc.lat;
    sat.longitude = loc.lon;
    sat.altitude = loc.alt;

    sat.position_covariance[0]  = loc.hdop;
    sat.position_covariance[4]  = loc.hdop;
    sat.position_covariance[8]  = 10;

    sat.position_covariance_type = 2;     // 2 协方差矩阵已知   
  }

  void run(){
    ros::Rate loopRate(15);
    while(ros::ok())
    {
      size_t n = sp_.available();
      if(n!=0)
      {
        std::string data = sp_.read(n);
        Location result = getGNGGA(data);
        if(result.alt == 0)
        {
          ROS_ERROR_ONCE("-----> No GPS Data !");
        }
        else{
          fillSatMessage(gpsdata_, result);
          pubGPSRaw_.publish(gpsdata_);
          if(result.fix == 4)
          {
            coordinateTrans(result);
            pubGpsOdom_.publish(gpsOdomData_);
          }
        }
      }
      loopRate.sleep();
    }
  }
};
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_serial_node");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle nh;
    ros::NodeHandle prNh("~");
    GPSMsgForward gpsMsgReceiver(nh, prNh);
    gpsMsgReceiver.run();
    return 0;
}
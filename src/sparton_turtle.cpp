// This program publishes randomly−generated velocity
// messages for turtlesim.
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h> // For geometry_msgs::Twist

using namespace std;

double sparton_to_linear_accel(const char* data);
double sparton_to_angular_vel(const char* data);
void split(const std::string &s, char delim, std::vector<std::string> &elems);
std::vector<std::string> split(const std::string &s, char delim);

serial::Serial ser;

int main ( int argc, char** argv ) {
   // Initialize the ROS system and become a node .
   ros::init ( argc, argv, "sparton_turtle" );
   ros::NodeHandle nh;

   // Create a publisher object.
   ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

   try
   {
       ser.setPort("/dev/ttyUSB0");
       ser.setBaudrate(115200);
       serial::Timeout to = serial::Timeout::simpleTimeout(1000);
       ser.setTimeout(to);
       ser.open();
   }
   catch (serial::IOException& e)
   {
       ROS_ERROR_STREAM("Unable to open port");
       return -1;
   }

   if(ser.isOpen()){
       ROS_INFO_STREAM("Serial Port initialized");
   }else{
       return -1;
   }

   ser.write("accelGyrop_mask d.on\r\n");

   // Loop at 100Hz until the node is shut down.
   ros::Rate rate(100);
   while(ros::ok()) {
      // Create and fill in the message . The other four
      // fields, which are ignored by turtlesim, default to 0.
      geometry_msgs::Twist msg;
      std::string results;
      //ser.write("accelp &di @ f.\r\n");

      ser.flush();
      std::string result = ser.readline(ser.available());
      if(result.length() > 0){
         std::vector<std::string> my_vec = split(result,',');
         //ROS_INFO_STREAM(result);
         if(my_vec.size() > 7){         

            msg.linear.x = sparton_to_linear_accel(my_vec[2].c_str());
            //msg.linear.y = sparton_to_linear_accel(my_vec[3].c_str());
            //msg.linear.z = sparton_to_linear_accel(my_vec[4].c_str());
            //msg.angular.x = sparton_to_angular_vel(my_vec[5].c_str());
            //msg.angular.y = sparton_to_angular_vel(my_vec[6].c_str());
            msg.angular.z = sparton_to_angular_vel(my_vec[7].c_str());

            ROS_INFO_STREAM(result);
            ROS_INFO_STREAM("Sending Sparton AHRS-M2 velocity command: " << "linear=" << msg.linear.x << " angular=" << msg.angular.z);

            // Publish the message .
            pub.publish(msg);
         }
      }
      

      // Wait until it's time for another iteration .
      rate.sleep();
   }
}

double sparton_to_linear_accel(const char* data){
   double lcl_dbl = atof(data) * 9.8055 / 1000;
   
   // Amateur stabilization
   if(lcl_dbl < 0.9 && lcl_dbl > -0.9)
      lcl_dbl = 0.0;

   return lcl_dbl;
}

double sparton_to_angular_vel(const char* data){
   double lcl_dbl = atof(data);
   
   // Amateur stabilization
   if(lcl_dbl < 0.01 && lcl_dbl > -0.01)
      lcl_dbl = 0.0;

   return lcl_dbl;
}

void split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}


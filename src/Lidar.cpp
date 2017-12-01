#include <ras_group8_lidar/Lidar.hpp>

// STD
#include <string>

namespace ras_group8_lidar {

Lidar::Lidar(ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  
  lidar_point_cloud_subscriber_= node_handle_.subscribe(lidar_point_cloud_topic_, 1, &Lidar::lidarCallback, this);

  lines_publisher_=node_handle_.advertise<nav_msgs::Path>("/lines",1,true);


  ROS_INFO("Successfully launched node.");
}

Lidar::~Lidar()
{
}

bool Lidar::readParameters()
{
  if (!node_handle_.getParam("lidar_point_cloud_topic", lidar_point_cloud_topic_))
    return false;
  if (!node_handle_.getParam("distance_threshold", distance_threshold_))
    return false;
  return true;
}

void Lidar::lidarCallback(const sensor_msgs::PointCloud& msg)
{
//    msg.points.size();
//    msg.channels.size();
//    //msg.channels.name;

//    ROS_INFO("I recive a message points %i, C1 %i, C2 %i", msg.points.size(),msg.channels[0].values.size(),msg.channels[1].values.size());
//    //ROS_INFO("%s ,%s ",msg.channels[0].name.c_str(),msg.channels[1].name.c_str());

     s_left_start_=0;
     s_left_stop_=0;
     s_right_start_=0;
     s_right_stop_=0;
     s_front_start_=0;
     s_front_stop_=0;



      //ROS_INFO("for %d",msg.points.size());
   for (int i=0;i<msg.points.size();i++)
   {
       if (msg.channels[1].values[i]<=40.0)
          { s_front_stop_=i;}
       else if (s_right_start_==0&&msg.channels[1].values[i]>=50.0)
          { s_right_start_=i;}
       else if (msg.channels[1].values[i]<=130.0)
          { s_right_stop_=i;}
       else if (s_left_start_==0&&msg.channels[1].values[i]>=230.0)
          { s_left_start_=i;}
       else if (msg.channels[1].values[i]<=310)
          { s_left_stop_=i;}
       else if (s_front_start_==0&&msg.channels[1].values[i]>=320.0)
          { s_front_start_=i;}
   }
   ROS_ERROR(" %f, %f ;%f, %f; %d %f",msg.channels[1].values[s_left_start_],msg.channels[1].values[s_left_stop_],msg.channels[1].values[s_right_start_],msg.channels[1].values[s_right_stop_],s_front_start_,msg.channels[1].values[s_front_stop_]);

    ROS_INFO(" %d, %d", s_left_start_,s_left_stop_);
    ROS_INFO("%f, %f, %f, %f",msg.points[s_left_start_].x,msg.points[s_left_start_].y,msg.points[s_left_stop_].x,msg.points[s_left_stop_].y);

   //left segment
   if(s_left_start_!=0&&s_left_stop_!=0)
   {
       bool finished=false;
       std::vector<int> i_start_list;
       i_start_list.push_back(s_left_start_);
       int i_stop=s_left_stop_;
       int i_start=s_left_start_;


       while (!finished)
       {
            //ROS_INFO("While");
           //Creat a line between the start and the end y=ax+b --> ax+b-y=0;
           double a=(msg.points[i_stop].y-msg.points[i_start].y)/(msg.points[i_stop].x-msg.points[i_start].x);
           double b=msg.points[i_start].y-a*msg.points[i_start].x;
           double max_distance=0;
           double distance=0;
           int i_max_distance=0;

           for (int j=i_start+1 ;j<i_stop ; j++)
           {
               distance=std::abs((a*msg.points[j].x-msg.points[j].y+b)/sqrt(pow(a,2)+1));
               //ROS_INFO("Distance %f, x:%f, y:%f ",distance,msg.points[j].x,msg.points[j].y);
               if (distance>distance_threshold_&&distance>max_distance)
               {
                   i_max_distance=j;
                   max_distance=distance;
               }
           }
           //ROS_INFO("MaxDistance %f index %d",max_distance, i_max_distance);
          // ros::Duration(10).sleep();

           if(i_max_distance==0)
           {
               i_start=i_stop;
               i_start_list.push_back(i_start);
               //ROS_INFO("I_MaxDistance %d s_left_stop %d",i_max_distance,s_left_stop_);
               if (i_stop==s_left_stop_)
               {
                   finished=true;
                   ROS_INFO("finnished");
               }
               else
               {
                   i_stop=s_left_stop_;
               }

           }
           else
           {
               i_stop=i_max_distance;
           }
       }
       //ros::Duration(5).sleep();
       lines_.poses.resize(i_start_list.size());

       for (int i=0; i<i_start_list.size(); i++)
       {
           lines_.poses[i].pose.position.x=msg.points[i_start_list[i]].x;
           lines_.poses[i].pose.position.y=msg.points[i_start_list[i]].y;
           //ROS_INFO("Element:%d x %f,y %f ",i_start_list[i],msg.points[i_start_list[i]].x,msg.points[i_start_list[i]].y);

       }
       lines_.header.stamp=ros::Time::now();
       lines_.header.frame_id="odom";

       lines_publisher_.publish(lines_);
        ROS_INFO("left Lines are created %i",i_start_list.size());

       //ROS_INFO("left Lines are created");
   }
   else
   {
       ROS_ERROR("Left segements are not possible");
   }
   //ros::Duration(5).sleep();



   //right segment



   //front segment






}


} /* namespace */

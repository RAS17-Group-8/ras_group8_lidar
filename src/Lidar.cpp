#include <ras_group8_lidar/Lidar.hpp>

// STD
#include <string>


namespace ras_group8_lidar {

Lidar::Lidar(ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
  
  lidar_point_cloud_subscriber_= node_handle_.subscribe("/scan_cloud", 1, &Lidar::lidarCallback, this);

  lines_publisher_=node_handle_.advertise<visualization_msgs::MarkerArray>("/map/markers",1,true);
  path_publisher_=node_handle_.advertise<nav_msgs::Path>("/linespath",1,true);
  
  distance_threshold_=0.03;
  continue_threshold_=0.1;
  minsize=5;
  target_frame="base_link";
  laser_frame="laser";


  ROS_INFO("Successfully launched node.");
}

Lidar::~Lidar()
{
}

bool Lidar::readParameters()
{
  ROS_INFO("Successfully launched node.1");
  if (!node_handle_.getParam("distance_threshold", distance_threshold_))
    return false;
  return true;
}

void Lidar::lidarCallback(const sensor_msgs::PointCloud& msg)
{
	walls.markers.clear();
    s_left_start_=0;
    s_left_stop_=msg.points.size();
    int numofpt=msg.points.size();

   //left segment
   if(s_left_stop_!=0)
   {
        ROS_INFO("start");
       bool finished=false;
       std::vector<int> i_start_list;
       i_start_list.push_back(s_left_start_);
       int i_stop=s_left_stop_;
       int i_start=s_left_start_;
       int true_start=s_left_start_;


       while (!finished)
       {
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

       int list_size=i_start_list.size();
       ROS_INFO("list size %d",list_size);
       int wallid=0;

       for (int line_st=0; line_st<list_size-1; line_st++)
       {
        //ROS_INFO("line point %d",line_st);
        int pt=i_start_list[line_st];
        int next=i_start_list[line_st+1];
        if(next<i_start_list[0]) next=next+numofpt;

        if ((next-pt)<minsize) continue;

        //ROS_INFO("good size");
        int st=pt;
        int et=st+1;
        int et2;
        while (et<=next)
        {
          if(et>numofpt) et2=et-numofpt;
          else et2=et;
          double dist=sqrt(pow(msg.points[et2].x-msg.points[st].x,2)+pow(msg.points[et2].y-msg.points[st].y,2));
          if(dist>continue_threshold_)
          {

            if((et-st)>=minsize && -2.5<msg.points[st].x<2.5 && -2.5<msg.points[st].y<2.5
            	&& -2.5<msg.points[et2].x<2.5 && -2.5<msg.points[et2].y<2.5)
            {
              visualization_msgs::Marker new_line=Lidar::makeline(msg.points[st].x,msg.points[st].y,msg.points[et2].x,msg.points[et2].y,wallid++);
            }
            st=et;
            et=et+1;
            continue;
          }
          else et=et+1;
        }
        if(st==pt&& -2.5<msg.points[pt].x<2.5 && -2.5<msg.points[pt].y<2.5
            	&& -2.5<msg.points[next].x<2.5 && -2.5<msg.points[next].y<2.5)
        {
          visualization_msgs::Marker new_line=Lidar::makeline(msg.points[pt].x,msg.points[pt].y,msg.points[next].x,msg.points[next].y,wallid++);        
          walls.markers.push_back(new_line);
          //ROS_INFO("new line, whole line");

        }
       }
       ROS_INFO("list size of array %d",sizeof(walls.markers));

       lines_publisher_.publish(walls);

       //ros::Duration(5).sleep();
       lines_.poses.resize(i_start_list.size());

       for (int i=0; i<i_start_list.size(); i++)
       {
           lines_.poses[i].pose.position.x=msg.points[i_start_list[i]].x;
           lines_.poses[i].pose.position.y=msg.points[i_start_list[i]].y;
           //ROS_INFO("Element:%d x %f,y %f ",i_start_list[i],msg.points[i_start_list[i]].x,msg.points[i_start_list[i]].y);

       }
       lines_.header.stamp=ros::Time::now();
       lines_.header.frame_id="laser";

       path_publisher_.publish(lines_);
       //  ROS_INFO("left Lines are created %i",i_start_list.size());

       //ROS_INFO("left Lines are created");
   }
   else
   {
       ROS_ERROR(" segements are not possible");
   }
}


visualization_msgs::Marker Lidar::makeline(float x1, float y1, float x2, float y2,int id)
{
  //ROS_INFO("make line %f, %f, %f, %f",x1,y1,x2,y2);
    geometry_msgs::PointStamped p1;
    p1.header.frame_id=laser_frame;
    p1.header.stamp =ros::Time::now();
    p1.point.x=x1;
    p1.point.y=y1;
    p1.point.z=0;

    geometry_msgs::PointStamped p2;
    p2.header.frame_id=laser_frame;
    p2.header.stamp =ros::Time::now();
    p2.point.x=x2;
    p2.point.y=y2;
    p2.point.z=0;

    geometry_msgs::PointStamped np1;
    geometry_msgs::PointStamped np2;




    try{
      listener.waitForTransform(target_frame, laser_frame,
                              ros::Time::now(), ros::Duration(3.0));
      listener.transformPoint(target_frame,p1,np1);
      listener.transformPoint(target_frame,p2,np2);

    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    geometry_msgs::Pose pose;
    pose.position.x=(np1.point.x+np2.point.x)/2;

    pose.position.y=(np1.point.y+np2.point.y)/2;



    pose.position.z=0.05;
    double angle=atan((np1.point.y-np2.point.y)/(np1.point.x-np2.point.x));
    double scale=sqrt(pow(np1.point.x-np2.point.x,2)+pow(np1.point.y-np2.point.y,2));
    tf::Quaternion tfq= tf::createQuaternionFromRPY(0,0,angle);
    tf::quaternionTFToMsg(tfq,pose.orientation);

    

    visualization_msgs::Marker marker;
    marker.header.frame_id = target_frame;
    marker.header.stamp = ros::Time::now();
    marker.id = id;
    marker.type = 1;
    marker.action = 0;
    marker.pose=pose;

    marker.scale.x = scale;
    marker.scale.y = 0.01;
    marker.scale.z = 0.1;

    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.5;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    return marker;

}


} /* namespace */

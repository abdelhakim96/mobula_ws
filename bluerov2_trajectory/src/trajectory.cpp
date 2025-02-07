/**
 * @file   trajectory.cpp
 * @author Hakim Amer 
 * @date   
 *  Trajectory generation node, contains 4 modes:
 1- position hold
 2- Go to setposition 
 3- Circular trajectory 
 4- Custom trajectory read from a text file 

 */



#include <trajectory.h>   // Adding header file: contains all variable types definitions and subscribers/publishers 
#include <bluerov2_trajectory/set_trajectoryConfig.h>
double sampleTime = 0.01; 
// Callback functions
//dynamicReconfigureCallback Retreives all the data from rqt_reconfigure GUI 

void dynamicReconfigureCallback(bluerov2_trajectory::set_trajectoryConfig& config, uint32_t level)  

{
    traj_on = config.traj_on;                              // Flag for activation of trajectory script
    pub_on_setpoint_pos = config.pub_on_setpoint_position; // Flag for publishing trajectory on mavros topic
    traj_type = config.traj_type;                          // Flag for publishing trajectory on mavros topic
    wp_x=config.wp_x;                                      // User specified waypoint x-coordinate
    wp_y=config.wp_y;                                      // User spcified waypoint y-coordinate
    wp_z=config.wp_z;                                      // User specified waypoint z-coordinate
    wp_yaw =config.wp_yaw;                                  // User specified yaw angle 
    radius = config.des_radius;                            // User specified radius of turn for circle trajectory
    absvel = config.des_velocity;                          // Velocity of ROV during circle trajectory
    reg_on = config.reg_on;

                         
}




// Callback for current ROV position 
void pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_pos = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};

    
    current_vel =  {msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z};
   // current_att_mat.setRotation(current_att_quat);
    //current_att_mat.getRPY(current_att[0], current_att[1], current_att[2]);
    

}

// Callback for desired ROV position 
void traj_extern_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    desired_pos = *msg;
}










int main(int argc, char** argv)
{
    ros::init(argc, argv, "rov_trajectory");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<bluerov2_trajectory::set_trajectoryConfig> server;
    dynamic_reconfigure::Server<bluerov2_trajectory::set_trajectoryConfig>::CallbackType f;

    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    // Roslaunch parameters
    

    // Publishers
    ref_pos_pub = nh.advertise<geometry_msgs::Vector3>("ref_trajectory/position", 1);   // reference position for ROV in world frame [m]
    ref_vel_pub = nh.advertise<geometry_msgs::Vector3>("ref_trajectory/velocity", 1);    //reference velocity for ROV in ROV body frame  [m/s]
    ref_yaw_pub = nh.advertise<std_msgs::Float64>("ref_trajectory/yaw", 1);              // reference yaw angle for ROV [deg]
    setpoint_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1); //reference position for ROV in world frame (for MAVROS) [m]
    traj_on_pub = nh.advertise<std_msgs::Bool>("trajectory_on", 1);   // Boolean indicating if Traj_on button is clicked (for recording data)
    reg_on_pub = nh.advertise<std_msgs::Bool>("regression_on", 1);   // Boolean indicating if GP regression is activated
    
 
    // Subscribers
    pos_sub = nh.subscribe<nav_msgs::Odometry>("/mobula/rov/odometry", 1, pos_cb);  // get current ROV position (ground truth)
    traj_extern_sub = nh.subscribe<geometry_msgs::PoseStamped>("/trajectory_extern", 1, traj_extern_cb); // get desired trajectory from external source
    


     // Sampling time for ros node
    ros::Rate rate(1 / sampleTime);

    
    current_pos.resize(3);
    current_vel.resize(3);
    bool trajectory_reset = false;
    while (ros::ok())
    {

        
        traj_on_msg.data = traj_on;
        ROS_INFO(" traj_on %f",  traj_on);


        if (traj_on )
        {
           
  
        if (traj_type == 0)  //  Keep current position
           {
                 trajectory_reset = true;

             ROS_INFO("--------Position Hold!--------");
             
             if (init_pos)
             {
             ref_traj_msg.x = current_pos.at(0); 
             ref_traj_msg.y = current_pos.at(1);             
             ref_traj_msg.z = current_pos.at(2);
             ref_yaw_msg.data = current_att[2];
             init_pos=0;
             }

           }
           
        if (traj_type == 1)  // Set point navigation defined by rqt_reconfigure (user)
            {
                 trajectory_reset = true;

               ROS_INFO("--------Waypoint navigation: Go to Setpoint!--------");
               ref_traj_msg.x  = wp_x;
               ref_traj_msg.y  = wp_y;             
               ref_traj_msg.z  = wp_z;
               ref_yaw_msg.data = wp_yaw* ((M_PI)/180.0);
               //init_pos=1;
            }

       

        if (traj_type == 2)  // Circlular trajectory with specified radius,centre and velocity
            {

               
            if (trajectory_reset== true) {
                // Reset trajectory to start from the beginning
                d_theta = 0.0; // Reset angle
                // Reset other variables if needed

                // Set the flag to true to indicate that the trajectory has been reset
                trajectory_reset = false;
            }

               ROS_INFO("--------Circle selected!--------");
               d_theta =  (absvel * sampleTime * radius + d_theta);
               double d_theta_2 = 2 * absvel * sampleTime * radius + d_theta;

               ref_traj_msg.x = radius + wp_x - radius * cos (d_theta);
               ref_traj_msg.y = wp_y + radius * sin (d_theta);
               ref_traj_msg.z = wp_z;      
               

               double p_x1 =  wp_x - radius * cos (d_theta);
               double p_x2 = wp_x - radius * cos (d_theta_2);

               double p_y1 = wp_x - radius * sin (d_theta);
               double p_y2 = wp_x - radius * sin (d_theta_2);

               //Transform to body frame 
                 double u_earth = (p_x2 - p_x1)/sampleTime ;
                 double v_earth = (p_y2 - p_y1)/sampleTime;

                double u_body = u_earth * cos(current_att[2]) + v_earth * sin(current_att[2]);
                double v_body = -u_earth * sin(current_att[2]) + v_earth * cos(current_att[2]);

           
               ref_vel_msg.x = u_body ;
               ref_vel_msg.y = -v_body;
               ref_vel_msg.z = 0.0;
               double yaw_to_center;
               ref_yaw_msg.data = wp_yaw* ((M_PI)/180.0);


             if (!current_pos.empty()) {
                    // Calculate yaw angle to face the center of the circle
                    double yaw_to_center = d_theta;

             }
             else{

                 yaw_to_center =0.0;
             }
               // ref_yaw_msg.data = (d_theta - M_PI)*0.0 ;
              // bool init_pos=1;

              cout <<"Yaw ref"<< ref_yaw_msg.data * (180/M_PI)  <<std::endl;
              cout <<"pos cb "<< current_pos.at(0) <<std::endl;
            }

            
        if (traj_type == 3)  // Read reference trajectory from a textfile
            { 
                            if (trajectory_reset== true) {
                // Reset trajectory to start from the beginning
                d_theta = 0.0; // Reset angle
                // Reset other variables if needed

                // Set the flag to true to indicate that the trajectory has been reset
                trajectory_reset = false;
            }
               d_theta =  (absvel * sampleTime * radius + d_theta);
               double d_theta_2 = 2 * absvel * sampleTime * radius + d_theta;
                               ROS_INFO("--------Figure-8 trajectory!--------");

                double omega = M_PI / 20.0; // Frequency of the figure-8
                double amplitude = 1.5;     // Amplitude of the figure-8

                double x = -amplitude + amplitude * cos(d_theta);
                double y = amplitude * sin(2 *  d_theta)/2;

                ref_traj_msg.x = x + wp_x;
                ref_traj_msg.y = y + wp_y;
                ref_traj_msg.z = wp_z;

                // Adjust yaw if needed
               // ref_yaw_msg.data = wp_yaw * (M_PI / 180.0);
                         
            }

        reg_on_pub.publish(reg_on_msg);

        }
        reg_on_msg.data = reg_on;
        ref_pos_pub.publish(ref_traj_msg);
        //ref_pos_pub_rviz.publish(ref_traj_msg);
        ref_vel_pub.publish(ref_vel_msg);

        ref_yaw_pub.publish(ref_yaw_msg);
        traj_on_pub.publish(traj_on_msg);
        ros::spinOnce();
        rate.sleep();
   
    }

}
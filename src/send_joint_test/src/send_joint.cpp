#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <stdio.h>

int main(int argc, char** argv){
    
    ros::init(argc, argv, "send_joint");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("robot_angle", 2);
    std_msgs::Float32MultiArray pub_msg;
    pub_msg.data.resize(20);

    /***********/
    /*Main Loop*/
    /***********/
    ros::Rate loop_rate(1);//[Hz]
    while(ros::ok()){
        
        printf("input joint angle¥n");
        printf("J1, J2, J3->");
        scanf("%f, %f, %f", &pub_msg.data[0], &pub_msg.data[1], &pub_msg.data[2]);

        pub.publish(pub_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();

    return 0;
}
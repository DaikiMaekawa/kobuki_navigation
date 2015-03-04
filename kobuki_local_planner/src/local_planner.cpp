#include <ros/ros.h>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <mocap_msgs/MoCapPose.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

class LocalPlanner{
public: 
    LocalPlanner(ros::NodeHandle &node) : 
        _path_sub(node.subscribe("/move_group/result", 10, &LocalPlanner::global_path_callback, this)),
        _pose_sub(node.subscribe("/kobuki/mocap_pose", 10, &LocalPlanner::self_pose_callback, this))
    {

    }

    void spin(){
        ros::Rate r(10);

        while(ros::ok()){
            for(int i=0; i < _path_gl.points.size(); i++){
                double x = _path_gl.points[i].transforms[0].translation.x;
                double y = _path_gl.points[i].transforms[1].translation.y;
                double z = _path_gl.points[i].transforms[2].translation.z;
                ROS_INFO_STREAM("path[" << i << "] = (" << x << ", " << y << ", " << z << ")");
            }

            r.sleep();
            ros::spinOnce();
        }
    }

private:
    void self_pose_callback(const mocap_msgs::MoCapPose &pose_msg){
        _self_pose = pose_msg;
        //ROS_INFO_STREAM("pose = " << _self_pose);
    }

    void global_path_callback(const moveit_msgs::MoveGroupActionResult &path_msg){
        _path_gl = path_msg.result.planned_trajectory.multi_dof_joint_trajectory;
        ROS_INFO_STREAM("path = " << _path_gl);
    }
    
    ros::Subscriber _path_sub, _pose_sub;
    //moveit_msgs::MoveGroupActionResult _path_msg_gl;
    mocap_msgs::MoCapPose _self_pose;
    trajectory_msgs::MultiDOFJointTrajectory _path_gl;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "local_planner");

    ros::NodeHandle node;
    LocalPlanner lp(node);
    lp.spin();

    return 0;
}


#include <ros/ros.h>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <mocap_msgs/MoCapPose.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <vector>
#include <math.h>

class LocalPlanner{
public: 
    LocalPlanner(ros::NodeHandle &node) : 
        _path_sub(node.subscribe("/move_group/result", 10, &LocalPlanner::global_path_callback, this)),
        _pose_sub(node.subscribe("/kobuki/mocap_pose", 10, &LocalPlanner::self_pose_callback, this))
    {
        _path_gl_it = _path_gl.points.begin();
    }

    void spin(){
        ros::Rate r(10);

        while(ros::ok()){
            if(_path_gl_it != _path_gl.points.end()){
                double t_pos[3];
                t_pos[0] = _path_gl_it->transforms[0].translation.x;
                t_pos[1] = _path_gl_it->transforms[0].translation.y;
                t_pos[2] = _path_gl_it->transforms[0].translation.z;
                
                double self_pos[3];
                self_pos[0] = _self_pose.pose.position.x;
                self_pos[1] = _self_pose.pose.position.y;
                self_pos[2] = _self_pose.pose.position.z;

                double dist = distance_points(t_pos, self_pos);
                ROS_INFO_STREAM("target_pos = (" << t_pos[0] << ", " << t_pos[1] << ", " << t_pos[2] << ")");
                ROS_INFO_STREAM("self_pos = (" << self_pos[0] << ", " << self_pos[1] << ", " << self_pos[2] << ")");
                ROS_INFO_STREAM("dist = " << dist);

                if(dist < 0.5) {
                    _path_gl_it++;
                }
            }

            r.sleep();
            ros::spinOnce();
        }
    }

private:
    double distance_points(double p1[3], double p2[3]){
        return sqrt(pow(p1[0]-p2[0], 2) + pow(p1[1]-p2[1], 2) + pow(p1[2]-p2[2], 2));
    }

    void self_pose_callback(const mocap_msgs::MoCapPose &pose_msg){
        _self_pose = pose_msg;
        //ROS_INFO_STREAM("pose = " << _self_pose);
    }

    void global_path_callback(const moveit_msgs::MoveGroupActionResult &path_msg){
        _path_gl = path_msg.result.planned_trajectory.multi_dof_joint_trajectory;
        _path_gl_it = _path_gl.points.begin();
        ROS_INFO_STREAM("path = " << _path_gl);
    }
    
    ros::Subscriber _path_sub, _pose_sub;
    //moveit_msgs::MoveGroupActionResult _path_msg_gl;
    mocap_msgs::MoCapPose _self_pose;
    trajectory_msgs::MultiDOFJointTrajectory _path_gl;
    std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint>::iterator _path_gl_it;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "local_planner");

    ros::NodeHandle node;
    LocalPlanner lp(node);
    lp.spin();

    return 0;
}


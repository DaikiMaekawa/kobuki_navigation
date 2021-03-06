#include <ros/ros.h>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <mocap_msgs/MoCapPose.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Twist.h>

#include <vector>
#include <math.h>

/*TODO: Delete functions*/

double distance_points(double p1[3], double p2[3]){
    return sqrt(pow(p1[0]-p2[0], 2) + pow(p1[1]-p2[1], 2));
}

double direction_deg(double p_lc[3]) {
    return atan2(p_lc[1], p_lc[0]) * 180.0 / 3.1415;
}

double normalize_deg(double ang){
    while(ang > 180) ang -= 360;
    while(ang < -180) ang+= 360;

    return ang;
}

void rotation2d(double p_in[3], double dq, double p_out[3]){
    p_out[0] = cos(dq) * p_in[0] - sin(dq) * p_in[1];
    p_out[1] = sin(dq) * p_in[0] + cos(dq) * p_in[1];
    p_out[2] = p_in[2] + dq;
}

double coord_trans_global_to_local(double p1_gl[3], double p2_gl[3], double p_out[3]){
    double p12[3];
    p12[0] = p2_gl[0]-p1_gl[0];
    p12[1] = p2_gl[1]-p1_gl[1];
    p12[2] = p2_gl[2]-p1_gl[2];
    
    rotation2d(p12, -p1_gl[2], p_out);
}

double clamp(double value, double high, double low){
    if(value > high) {
        return high;
    } else {
        if(value < low) return low;
        else            return value;
    }
}

void quaternion_to_rpy(double q[4], double rpy[3]){
    const float sin_beta = 2 * (q[3] * q[1] - q[0] * q[2]);

    if(sin_beta < -1.0f){
        rpy[0] = 2 * atan2(q[0], q[3]);
        rpy[1] = -0.5f * M_PI;
        rpy[2] = 0;
    }else if(sin_beta > 1.0f){
        rpy[0] = 2 * atan2(q[0], q[3]);
        rpy[1] = 0.5f * M_PI;
        rpy[2] = 0;
    }else{
        rpy[0] = atan2(2 * (q[3] * q[0] + q[1] * q[2]),
                    1.0f - 2 * (q[0] * q[0] + q[1] * q[1]));
        rpy[1] = asin(sin_beta);
        rpy[2] = atan2(2 * (q[3] * q[2] + q[1] * q[0]),
                    1.0f - 2 * (q[1] * q[1] + q[2] * q[2]));
    }
}

class LocalPlanner{
public: 
    LocalPlanner(ros::NodeHandle &node) : 
        _path_sub(node.subscribe("/move_group/result", 10, &LocalPlanner::global_path_callback, this)),
        _pose_sub(node.subscribe("/kobuki/mocap_pose", 10, &LocalPlanner::self_pose_callback, this)),
        _cmd_pub(node.advertise<geometry_msgs::Twist>("cmd_vel", 10)),
        _start_follow_path(false)
    {
        _path_gl_it = _path_gl.points.begin();
    }

    void spin(){
        ros::Rate r(10);

        while(ros::ok()){
            geometry_msgs::Twist cmd;
            
            if(_start_follow_path){
                double t_pos_gl[3];
                t_pos_gl[0] = _path_gl_it->transforms[0].translation.x;
                t_pos_gl[1] = _path_gl_it->transforms[0].translation.y;
                t_pos_gl[2] = 0;
                double rpy[3];
                double q[4];
                q[0] = _self_pose.pose.orientation.x;
                q[1] = _self_pose.pose.orientation.y;
                q[2] = _self_pose.pose.orientation.z;
                q[3] = _self_pose.pose.orientation.w;
                quaternion_to_rpy(q, rpy);

                double self_pos[3];
                self_pos[0] = _self_pose.pose.position.x;
                self_pos[1] = _self_pose.pose.position.y;
                self_pos[2] = rpy[2];

                double dist = distance_points(t_pos_gl, self_pos);
                ROS_INFO_STREAM("target_pos = (" << t_pos_gl[0] << ", " << t_pos_gl[1] << ", " << t_pos_gl[2] << ")");
                ROS_INFO_STREAM("self_pos = (" << self_pos[0] << ", " << self_pos[1] << ", " << self_pos[2] << ")");
                ROS_INFO_STREAM("dist = " << dist);
                
                double t_pos_lc[3];
                coord_trans_global_to_local(self_pos, t_pos_gl, t_pos_lc);
                double dir = direction_deg(t_pos_lc);
                ROS_INFO_STREAM("dir = " << dir);
                ROS_INFO_STREAM("normalize_dir = " << normalize_deg(dir));
                
                if(_path_gl_it == --_path_gl.points.end() && dist < 0.3){
                    double finish_rpy[3];
                    double finish_q[4];
                    finish_q[0] = _path_gl_it->transforms[0].rotation.x;
                    finish_q[1] = _path_gl_it->transforms[0].rotation.y;
                    finish_q[2] = _path_gl_it->transforms[0].rotation.z;
                    finish_q[3] = _path_gl_it->transforms[0].rotation.w;

                    quaternion_to_rpy(finish_q, finish_rpy);
                    double diff = normalize_deg(self_pos[2] - finish_rpy[2]);
                    ROS_INFO_STREAM("diff = " << diff);
                    
                    if(diff < 20){
                        cmd.linear.x = 0;
                        cmd.linear.y = 0;
                        cmd.angular.z = clamp(diff * 0.01, 1.0, -1.0);
                    }else{
                        _start_follow_path = false;
                        cmd.linear.x = 0;
                        cmd.linear.y = 0;
                        cmd.angular.z = 0;
                    }
                                    
                }else{
                    if(normalize_deg(dir) > 30){
                        cmd.linear.x = 0;
                        cmd.linear.y = 0;
                        cmd.angular.z = clamp(normalize_deg(dir) * 0.01, 1.0, -1.0);
                    }else{
                        cmd.linear.x = clamp(t_pos_lc[0], 1.0, -1.0);
                        cmd.linear.y = clamp(t_pos_lc[1], 1.0, -1.0);
                        cmd.angular.z = clamp(normalize_deg(dir) * 0.01, 1.0, -1.0);
                    }
                    
                    if(dist < 0.3 && dir < 40) {
                        _path_gl_it++;
                    }
                }
            }
            
            ROS_INFO("loop");
            _cmd_pub.publish(cmd);

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
        if(_path_gl.points.size() > 1){
            _start_follow_path = true;
        }
        _path_gl_it = _path_gl.points.begin();
        ROS_INFO_STREAM("path = " << _path_gl);
    }
    
    ros::Subscriber _path_sub, _pose_sub;
    ros::Publisher _cmd_pub;
    bool _start_follow_path;
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


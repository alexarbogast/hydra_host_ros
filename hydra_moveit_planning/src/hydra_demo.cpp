#include <unordered_map>
#include <algorithm>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

static const std::string NAME = "hydra_planning_demo";

static const std::string PLANNING_GROUP = "hydra_planning_group";
static const std::string ROB1_TOOL = "rob1_flange";
static const std::string ROB2_TOOL = "rob2_flange";

Eigen::Isometry3d pose_interp(double t,
        const Eigen::Isometry3d& aff1, const Eigen::Isometry3d& aff2) {
    t = std::min(1.0, std::max(0.0, t));

    Eigen::Quaterniond rot1(aff1.linear());
    Eigen::Quaterniond rot2(aff2.linear());

    Eigen::Vector3d trans1 = aff1.translation();
    Eigen::Vector3d trans2 = aff2.translation();

    Eigen::Isometry3d result;
    result.translation() = (1.0 - t) * trans1 + t * trans2;
    result.linear() = rot1.slerp(t, rot2).toRotationMatrix();
    return result;
}

class VirtualObject {
public:
    VirtualObject() {
        this->frame_ = Eigen::Isometry3d::Identity();
    }

    inline void setPose(const Eigen::Isometry3d& pose) {this->frame_ = pose;}
    inline Eigen::Isometry3d getPose() const { return this->frame_; }

    inline void addRigidChild(const std::string& name, 
                              const Eigen::Isometry3d& pose) {
        this->children_.emplace(name, pose);
    }

    Eigen::Isometry3d getChildPose(const std::string& name) {
        return frame_ * children_.at(name);
    }

    void animate(const Eigen::Isometry3d& to_pose, std::size_t steps,
                 std::map<std::string, EigenSTL::vector_Isometry3d>& cache) {
        const static std::string base_name = "root";

        cache.clear();
        EigenSTL::vector_Isometry3d test;
        cache.emplace(base_name, test);
        for (auto& child : children_) {
            cache.emplace(child.first, test);
        }

        Eigen::Isometry3d start_pose = frame_;
        for (std::size_t i = 0; i < steps + 1; i++) {
            double t = i / (double)steps; 
            // update object frame
            Eigen::Isometry3d interp = pose_interp(t, start_pose, to_pose);
            cache.at(base_name).push_back(interp);

            // cache child frames
            for (auto& child : children_) {
                cache.at(child.first).push_back(interp * child.second);
            }
        }
        frame_ = to_pose;
    }

    void relanimate(const Eigen::Isometry3d& transform, std::size_t steps,
                 std::map<std::string, EigenSTL::vector_Isometry3d>& cache) {
        Eigen::Isometry3d to_pose = transform * frame_;
        animate(to_pose, steps, cache);
    }

protected:

    Eigen::Isometry3d frame_ = Eigen::Isometry3d::Identity();
    std::map<std::string, Eigen::Isometry3d> children_;
};


class HydraDemo {
public:
    HydraDemo() 
        : move_group_(PLANNING_GROUP), 
          current_state_(*(move_group_.getCurrentState())) {
        this->joint_model_group_ = 
            move_group_.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        this->move_group_.setMaxVelocityScalingFactor(0.8);
        this->move_group_.setMaxAccelerationScalingFactor(0.8);
    }

    bool run() {
        ROS_INFO_NAMED(NAME, "Starting hydra planning demo");
        moveZeros();
        moveHome();

        routine();

        moveHome();
        ROS_INFO_NAMED(NAME, "Finished hydra planning demo successfully");
        return true; 
    }

private:
    void moveHome() {
        move_group_.setStartStateToCurrentState();
        move_group_.setNamedTarget("home");
        auto error_code = move_group_.move();
        
        if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_ERROR_NAMED(NAME, "Failed to plan and execute move to home");
        }
    }

    void moveZeros() {
        move_group_.setStartStateToCurrentState();
        move_group_.setNamedTarget("zeros");
        auto error_code = move_group_.move();
        
        if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_ERROR_NAMED(NAME, "Failed to plan and execute move to zeros");
        }
    }

    void routine() {
        move_group_.setStartStateToCurrentState();        

        VirtualObject object;
        Eigen::Isometry3d start_pose = Eigen::Translation3d(0.0, 0.0, 0.8) 
                                         * Eigen::Isometry3d::Identity();
        object.setPose(start_pose);

        // robots pose relative to virtual object
        Eigen::Isometry3d rob1_rel_pose = Eigen::Translation3d(0.0, -0.025, 0.0) 
            * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());
        Eigen::Isometry3d rob2_rel_pose = Eigen::Translation3d(0.0, 0.025, 0.0) 
            * Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ());

        object.addRigidChild(ROB1_TOOL, rob1_rel_pose);
        object.addRigidChild(ROB2_TOOL, rob2_rel_pose);

        Eigen::Isometry3d rob1_pose1 = object.getChildPose(ROB1_TOOL);
        Eigen::Isometry3d rob2_pose1 = object.getChildPose(ROB2_TOOL);
        move_group_.setPoseTarget(rob1_pose1, ROB1_TOOL);
        move_group_.setPoseTarget(rob2_pose1, ROB2_TOOL);
        move_group_.move();

        // plan ahead
        current_state_ = *(move_group_.getCurrentState());
        auto animate = [this] (const Eigen::Isometry3d& transform,
                               VirtualObject& object,
                               std::size_t steps,
                               double scaling) {
            std::map<std::string, EigenSTL::vector_Isometry3d> anim_cache;
            object.animate(transform, steps, anim_cache);

            moveit_msgs::RobotTrajectory traj_msg;
            planMotion({ROB1_TOOL, ROB2_TOOL}, 
                       {anim_cache.at(ROB1_TOOL), anim_cache.at(ROB2_TOOL)},
                       traj_msg, scaling, 0.8);
            return traj_msg;
        };

        // move object, plan and excecute motion 
        Eigen::Isometry3d transform1 = Eigen::Translation3d(-0.15, -0.15, 0.2) 
                                        * object.getPose();
        Eigen::Isometry3d transform2 = Eigen::Translation3d(0.0, 0.3, 0.0) 
                                        * transform1;
        Eigen::Isometry3d transform3 = Eigen::Translation3d(0.3, -0.3, 0.3) 
                                        * transform2;
        Eigen::Isometry3d transform4 = Eigen::Translation3d(-0.15, 0.15, -0.2) 
                                        * transform3;

        Eigen::Isometry3d transform5 = transform4 *
                Eigen::AngleAxisd(M_PI/3, Eigen::Vector3d::UnitZ());
        Eigen::Isometry3d transform6 =  transform5 *
                Eigen::AngleAxisd(-2*M_PI/3, Eigen::Vector3d::UnitZ());
        Eigen::Isometry3d transform7 = transform6 *
                Eigen::AngleAxisd(M_PI/3, Eigen::Vector3d::UnitZ());
        Eigen::Isometry3d transform8 = transform7 *
                Eigen::AngleAxisd(0.8 * -M_PI/2, Eigen::Vector3d::UnitX());
        Eigen::Isometry3d transform9 = transform8 *
                Eigen::AngleAxisd(1.5 * M_PI/2, Eigen::Vector3d::UnitX());
        Eigen::Isometry3d transform10 = transform9;
        transform10.linear() = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()) * transform9.linear();
        Eigen::Isometry3d transform11 = transform10;
        transform11.linear() = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()) * transform10.linear();
        Eigen::Isometry3d transform12 = transform11 *
                Eigen::AngleAxisd(-0.7 * M_PI/2, Eigen::Vector3d::UnitX());

        std::vector<moveit_msgs::RobotTrajectory> frames;
        frames.push_back(animate(transform1, object, 50,  0.8));
        frames.push_back(animate(transform2, object, 50,  0.8));
        frames.push_back(animate(transform3, object, 50, 0.8));
        frames.push_back(animate(transform4, object, 50,  0.8));

        frames.push_back(animate(transform5,  object, 25, 0.8));
        frames.push_back(animate(transform6,  object, 25, 0.8));
        frames.push_back(animate(transform7,  object, 25, 0.8));
        frames.push_back(animate(transform8,  object, 25, 0.8));
        frames.push_back(animate(transform9,  object, 25, 0.8));
        frames.push_back(animate(transform10, object, 25, 0.8));
        frames.push_back(animate(transform11, object, 25, 0.8));
        frames.push_back(animate(transform12, object, 25, 0.8));
        frames.push_back(animate(start_pose,  object, 25, 0.8));

        // execute trajectories
        for (auto& frame : frames) {
            move_group_.execute(frame);
        }
    }

private:
    // note: bypass collision checking to get continuous multi-robot traj
    bool planMotion(const std::vector<std::string>& tips,
                    const std::vector<EigenSTL::vector_Isometry3d>& paths,
                    moveit_msgs::RobotTrajectory& traj_msg,
                    double max_velocity_scaling_factor = 1.0,
                    double max_acceleration_scaling_factor = 1.0) {
        const size_t n_tips = tips.size();
        if (n_tips != paths.size()) {
            ROS_ERROR_NAMED(NAME, "Mismatch number of tips and paths");
            return false;
        }

        // make sure the paths have the same number of waypoints
        auto const path_size = paths.front().size();
        bool same_size = std::all_of(begin(paths), end(paths),
                            [path_size](const EigenSTL::vector_Isometry3d& x) {
                                return x.size() == path_size; });
        
        // plan
        robot_trajectory::RobotTrajectory traj(move_group_.getRobotModel(), 
                                               joint_model_group_->getName());
        traj.addSuffixWayPoint(current_state_, 0.0);

        // To limit absolute joint-space jumps, we pass consistency limits to the IK solver
        double jump_threshold = 0.1;
        std::vector<std::vector<double>> consistency_limits = 
                                                 {{0.1, 0.1, 0.1, 0.1, 0.1, 0.1}, 
                                                  {0.1, 0.1, 0.1, 0.1, 0.1, 0.1}};
    
        for (std::size_t point_idx = 0; point_idx < path_size; point_idx++) {
            EigenSTL::vector_Isometry3d points;
            for (std::size_t tip_idx = 0; tip_idx < n_tips; tip_idx++) {
                points.push_back(paths[tip_idx][point_idx]);
            }
            
            //current_state.setFromIK(joint_model_group_, points, tips, consistency_limits, 0.0, );
            current_state_.setFromIKSubgroups(joint_model_group_, points, tips,
                                     {consistency_limits}, 5.0);
            current_state_.update();
            traj.addSuffixWayPoint(current_state_, (point_idx + 3) * 1);
        }
        
        // time parameterization
        trajectory_processing::IterativeSplineParameterization time_param;
        //trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
        bool success = time_param.computeTimeStamps(traj, 
                                        max_velocity_scaling_factor,
                                        max_acceleration_scaling_factor);
        if (!success) {
            ROS_ERROR_NAMED(NAME, "Time paramaterization failed");
        }

        // convert to moveit trajectory
        traj.getRobotTrajectoryMsg(traj_msg);
        return true;
    }

    moveit::planning_interface::MoveGroupInterface move_group_;
    const moveit::core::JointModelGroup* joint_model_group_;

    moveit::core::RobotState current_state_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, NAME);
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    HydraDemo demo;
    demo.run();
    return 0;
}
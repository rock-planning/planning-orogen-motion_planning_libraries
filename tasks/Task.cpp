/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#include <base/logging/logging_printf_style.h>

#include <envire/Orocos.hpp>

using namespace motion_planning_libraries;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    
    mpMotionPlanningLibraries = boost::shared_ptr<MotionPlanningLibraries>(
            new MotionPlanningLibraries(_config.get()));
    
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    
    // Set traversability map.
    envire::OrocosEmitter::Ptr binary_event;
    if(_traversability_map.read(binary_event) == RTT::NewData)
    {
        mEnv.applyEvents(*binary_event);   
        mpMotionPlanningLibraries->setTravGrid(&mEnv, _traversability_map_id);
    }
     
    // Set start pose.
    if(_start_pose_samples.read(mStartPose) == RTT::NewData) {
        mpMotionPlanningLibraries->setStartPoseInWorld(mStartPose);
        _debug_start_pose_samples.write(mStartPose);
    }
    
    // Set goal pose.
    if(_goal_pose_samples.read(mGoalPose) == RTT::NewData) {
        mpMotionPlanningLibraries->setGoalPoseInWorld(mGoalPose);
        base::samples::RigidBodyState new_goal;
        new_goal.position = mGoalPose.position;
        double yaw = mGoalPose.getYaw();
        new_goal.orientation = Eigen::AngleAxis<double>(yaw, Eigen::Vector3d::UnitZ());
        _debug_goal_pose_samples.write(mGoalPose);
    }
  
    if(!mpMotionPlanningLibraries->plan(_planning_time_sec)) {
        LOG_WARN("Planning could not be finished");
    } else { 
        mpMotionPlanningLibraries->printPathInWorld();
        std::vector <base::Waypoint > path = mpMotionPlanningLibraries->getPathInWorld();
        _path.write(path);

        std::vector<base::Trajectory> vec_traj; 
        vec_traj.push_back(mpMotionPlanningLibraries->getTrajectoryInWorld(0.6));
        _trajectory.write(vec_traj);
    }
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

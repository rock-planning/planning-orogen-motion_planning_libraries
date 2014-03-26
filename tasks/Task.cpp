/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#include <base/logging/logging_printf_style.h>

#include <envire/Orocos.hpp>

#include <global_path_planner/GlobalPathPlanner.hpp>

using namespace global_path_planner;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    mpGlobalPathPlanner = new Ompl();
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
    mpGlobalPathPlanner = new Ompl();
}

Task::~Task()
{
    delete mpGlobalPathPlanner;
    mpGlobalPathPlanner = NULL;
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
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
        mpGlobalPathPlanner->setTravGrid(&mEnv, _traversability_map_id);
    }
     
    // Set start pose.
    if(_start_pose_samples.read(mStartPose) == RTT::NewData) {
        mpGlobalPathPlanner->setStartPoseInWorld(mStartPose);
        _debug_start_pose_samples.write(mStartPose);
    }
    
    // Set goal pose.
    if(_goal_pose_samples.read(mGoalPose) == RTT::NewData) {
        mpGlobalPathPlanner->setGoalPoseInWorld(mGoalPose);
        base::samples::RigidBodyState new_goal;
        new_goal.position = mGoalPose.position;
        double yaw = mGoalPose.getYaw();
        new_goal.orientation = Eigen::AngleAxis<double>(yaw, Eigen::Vector3d::UnitZ());
        _debug_goal_pose_samples.write(mGoalPose);
    }
  
    if(!mpGlobalPathPlanner->plan(30)) {
        LOG_WARN("Planning could not be finished");
    } else { 
#if 0
        // Create test waypoints
        /*
        std::vector <base::Waypoint > path;
        path.push_back(base::Waypoint(base::Vector3d(0,0,0), 0, 0, 0));
        path.push_back(base::Waypoint(base::Vector3d(0,1,0), M_PI/2.0, 0, 0));
        path.push_back(base::Waypoint(base::Vector3d(1,1,0), M_PI, 0, 0));
        path.push_back(base::Waypoint(base::Vector3d(1,0,0), -M_PI/2.0, 0, 0));
        _path.write(path);
        */ 
        std::vector <base::Waypoint > path = mpGlobalPathPlanner->getPath();
        _path.write(path);

        // Test: Start/goal pose converted to a waypoint.
        base::Waypoint wp_start(mStartPose.position, mStartPose.getYaw(), 0, 0);
        _waypoint_start.write(wp_start);
        base::Waypoint wp_goal(mGoalPose.position, mGoalPose.getYaw(), 0, 0);
        _waypoint_goal.write(wp_goal);
        
        std::vector<base::Trajectory> vec_traj; 
        vec_traj.push_back(mpGlobalPathPlanner->getTrajectory(0.6));
        _trajectory.write(vec_traj);
#endif
    }
    
        std::vector <base::Waypoint > path = mpGlobalPathPlanner->getPathInWorld();
        _path.write(path);
        
        
        std::vector<base::Trajectory> vec_traj; 
        vec_traj.push_back(mpGlobalPathPlanner->getTrajectoryInWorld(0.6));
        _trajectory.write(vec_traj);
        
    // Send all valid samples as waypoints.
    //_samples.write(mpGlobalPathPlanner->getSamples());
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

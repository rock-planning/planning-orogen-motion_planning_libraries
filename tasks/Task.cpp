/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#include <global_path_planner/GlobalPathPlanner.hpp>
#include <envire/Orocos.hpp>

using namespace global_path_planner;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    mpGlobalPathPlanner = new GlobalPathPlanner();
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
    mpGlobalPathPlanner = new GlobalPathPlanner();
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
    base::samples::RigidBodyState start_pose;
    if(_start_pose_samples.read(start_pose) == RTT::NewData) {
        mpGlobalPathPlanner->setStartWorld(start_pose);
        _debug_start_pose_samples.write(start_pose);
    }
    
    // Set goal pose.
    base::samples::RigidBodyState goal_pose;
    if(_goal_pose_samples.read(goal_pose) == RTT::NewData) {
        mpGlobalPathPlanner->setGoalWorld(goal_pose);
        _debug_goal_pose_samples.write(goal_pose);
    }
    
    if(!mpGlobalPathPlanner->plan(10)) {
        LOG_WARN("Planning could not be finished");
    } else { 
        std::vector<base::Trajectory> vec_traj; 
        vec_traj.push_back(mpGlobalPathPlanner->getTrajectory(0.6));
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

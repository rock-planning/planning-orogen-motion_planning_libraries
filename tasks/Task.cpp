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
    // Use a loop here because binary_events could contain partial updates
    bool new_map = false;
    while(_traversability_map.read(binary_event) == RTT::NewData)
    {
        mEnv.applyEvents(*binary_event);   
        new_map = true;
    }
    if(new_map) {
        mpMotionPlanningLibraries->setTravGrid(&mEnv, _traversability_map_id);
    }
     
    // Set start state / pose. 
    if(_start_state.connected()) {
        if(_start_state.readNewest(mStartState) == RTT::NewData) {
            if(mpMotionPlanningLibraries->setStartState(mStartState)) {
                mStartPose = mStartState.mPose;
                _start_pose_samples_debug.write(mStartPose);
            }
        }
    } else {   
        if(_start_pose_samples.readNewest(mStartPose) == RTT::NewData) {
            if(mpMotionPlanningLibraries->setStartState(State(mStartPose))) {
                _start_pose_samples_debug.write(mStartPose);
            }
        }
    }
    
    // Set goal state / pose.
    if(_goal_state.connected()) {
         if(_goal_state.readNewest(mGoalState) == RTT::NewData) {
            if(mpMotionPlanningLibraries->setGoalState(mGoalState)) {
                mGoalPose = mGoalState.mPose;
                _goal_pose_samples_debug.write(mGoalPose);
            }
        }
    } else {
        if(_goal_pose_samples.readNewest(mGoalPose) == RTT::NewData) {
            if(mpMotionPlanningLibraries->setGoalState(State(mGoalPose))) {
                _goal_pose_samples_debug.write(mGoalPose);
            }
        }
    }

    if(!mpMotionPlanningLibraries->plan(_planning_time_sec)) {
        LOG_WARN("Planning could not be finished");
        enum MplErrors err = mpMotionPlanningLibraries->getError();
        switch(err) {
            case MPL_ERR_MISSING_START: state(MISSING_START); break;
            case MPL_ERR_MISSING_GOAL: state(MISSING_GOAL); break;
            case MPL_ERR_MISSING_TRAV: state(MISSING_TRAV); break;
            case MPL_ERR_MISSING_START_GOAL: state(MISSING_START_GOAL); break;
            case MPL_ERR_MISSING_START_TRAV: state(MISSING_START_TRAV); break;
            case MPL_ERR_MISSING_GOAL_TRAV: state(MISSING_GOAL_TRAV); break;
            case MPL_ERR_MISSING_START_GOAL_TRAV: state(MISSING_START_GOAL_TRAV); break;
            case MPL_ERR_PLANNING_FAILED: state(PLANNING_FAILED); break;
            case MPL_ERR_WRONG_STATE_TYPE: state(WRONG_STATE_TYPE); break;
            case MPL_ERR_INITIALIZE_MAP: state(INITIALIZE_MAP_ERROR); break;
            case MPL_ERR_SET_START_GOAL: state(SET_START_GOAL_ERROR); break;
            default: state(UNDEFINED_ERROR); break;
        }
    } else {
        state(RUNNING);
        //mpMotionPlanningLibraries->printPathInWorld();
        std::vector <base::Waypoint > path = mpMotionPlanningLibraries->getPathInWorld();
        _waypoints.write(path);

        std::vector<base::Trajectory> vec_traj = 
                mpMotionPlanningLibraries->getTrajectoryInWorld(_trajectory_speed.get());
        _trajectory.write(vec_traj);
        
        std::vector<struct State> states = mpMotionPlanningLibraries->getStatesInWorld();
        _states.write(states); 
    }
    
    // Export automatically generated SBPL motion primitives.
    // (will be generated in SBPL+XYTHETA if no mprim file has been specified.
    if(_config.get().mPlanningLibType == LIB_SBPL &&
        _config.get().mEnvType == ENV_XYTHETA &&
        _config.get().mSBPLEnvFile.empty()) {
        struct SbplMotionPrimitives mprims;
        if(mpMotionPlanningLibraries->getSbplMotionPrimitives(mprims)) {
            _sbpl_mprims_debug.write(mprims);
        }
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

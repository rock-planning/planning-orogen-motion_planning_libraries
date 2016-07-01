/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#include <base/logging/logging_printf_style.h>

#include <envire/Orocos.hpp>
#include <motion_planning_libraries/MotionPlanningLibraries.hpp>


using namespace motion_planning_libraries;

Task::Task(std::string const& name)
    : TaskBase(name), mEscapeTrajAvailable(false)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine), mEscapeTrajAvailable(false)
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
    mEscapeTrajAvailable = false;
    
    if (! TaskBase::startHook())
        return false;
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();
    
    // Uses the task state to communicate which input is missing.
    enum MplErrors err_inputs;
    mpMotionPlanningLibraries->allInputsAvailable(err_inputs);
    setTaskState(err_inputs);
    
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
        // Just wait some time for a new goal pose in case both
        // map and new goal have been sent together.
        sleep(1);
    }
    
    // Start and goal can only be set if the traversability map is available.
    if(!mpMotionPlanningLibraries->travGridAvailable()) {
        return;
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
            double initial_footprint = _initial_footprint.get();
            printf("FOOTPRINT1 %4.2f\n",initial_footprint);
            State start_state(mStartPose);
            // Uses the footprint property or calculates the medium set
            // footprint.
            // TODO: Use a port supporting the current footprint of the system.
            printf("FOOTPRINT2 %4.2f\n",initial_footprint);
            if(initial_footprint <= 0) {
                initial_footprint = (_config.get().getMaxRadius() + 
                        _config.get().getMinRadius()) / 2.0;
            }
            printf("FOOTPRINT3 %4.2f\n",initial_footprint);
            start_state.mFootprintRadius = initial_footprint;
            if(mpMotionPlanningLibraries->setStartState(start_state)) {
                _start_pose_samples_debug.write(mStartPose);
            }
            printf("FOOTPRINT4 %4.2f\n",start_state.mFootprintRadius);
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
    
    mpMotionPlanningLibraries->allInputsAvailable(err_inputs);
    setTaskState(err_inputs);

    if(mpMotionPlanningLibraries->replanningRequired()) {
        double cost = 0.0;
        state(PLANNING);
        if(!mpMotionPlanningLibraries->plan(_planning_time_sec, cost)) {
            enum MplErrors err = mpMotionPlanningLibraries->getError();
            if(err != MPL_ERR_NONE && err != MPL_ERR_REPLANNING_NOT_REQUIRED) {
                std::string err_str;
                if(mpMotionPlanningLibraries->getErrorString(err, err_str)) {
                    LOG_WARN("Planning could not be finished, error %s", err_str.c_str());
                } else {
                    LOG_WARN("Planning could not be finished, error %d", (int)err);
                }
                // In case of a real error an empty trajectory will be sent.
                std::vector<base::Trajectory> empty_trajectories;
                _trajectory.write(empty_trajectories);
            }
            setTaskState(err);
            
            if(err == MPL_ERR_START_ON_OBSTACLE || err == MPL_ERR_START_GOAL_ON_OBSTACLE) {
                LOG_INFO("Start state lies on an obstacle, tries to generate an escape trajectory");
                if(generateEscapeTrajectory()) {
                    LOG_INFO("Escape trajectory has been created");
                } else {
                    LOG_WARN("Escape trajectory could not be created");
                }
            }
        } else {
            LOG_INFO("Planning successful");
            // If foundFinalSolution() is not supported or implemented,
            // we will only plan once and switch to planning successful.
            bool final_solution = mpMotionPlanningLibraries->foundFinalSolution();
            if(final_solution) {
                LOG_INFO("Planning found final solution");
                state(PLANNING_SUCCESSFUL);
            } 
        
            // Compare new and old path and just publish and print path if its new.
            std::vector <base::Waypoint > path = mpMotionPlanningLibraries->getPathInWorld();
            
            bool new_path = false;
            if(path.size() != mLastPath.size()) {
                new_path = true;
            } else {
                std::vector <base::Waypoint >::iterator it_new = path.begin();
                std::vector <base::Waypoint >::iterator it_last = mLastPath.begin();
                for(; it_new < path.end() && it_last < mLastPath.end(); it_new++, it_last++) {
                    if(it_new->position != it_last->position || 
                        it_new->heading != it_last->heading) {
                        new_path = true;
                        break;
                    }
                }
            }
            mLastPath = path;
            
            if(new_path) {
                LOG_INFO("New path found");
                mEscapeTrajAvailable = true;
            } else {
                LOG_INFO("No new path found");
            }
            
            // The final solution is also only provided once because
            // replanningRequired() will return false as soon as an optimal 
            // solution has been found.
            // So every new path is returned or only the final solution.
            if((new_path && !_only_provide_optimal_trajectories.get()) || 
                    (_only_provide_optimal_trajectories.get() && final_solution)) {
                
                LOG_INFO("Publish found path");
                mpMotionPlanningLibraries->printPathInWorld();
                
                _waypoints.write(path);

                std::vector<base::Trajectory> vec_traj = 
                        mpMotionPlanningLibraries->getTrajectoryInWorld();
                _trajectory.write(vec_traj);
                
                std::vector<struct State> states = mpMotionPlanningLibraries->getStatesInWorld();
                _states_mpl.write(states); 
                
                _path_cost.write(cost);
            }
        }
    }
    
    // Export automatically generated SBPL motion primitives
    // (will be generated in SBPL+XYTHETA if no mprim file has been specified).
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

bool Task::generateEscapeTrajectory() {
    bool ret = false;
    std::vector<base::Trajectory> escape_traj;

    if(mpMotionPlanningLibraries && mEscapeTrajAvailable) {
        escape_traj = mpMotionPlanningLibraries->getEscapeTrajectoryInWorld();

        if(escape_traj.size() > 0) {
            ret = true;
        }
    }

    if(_send_escape_traj_to_traj_port.get()) {
        _trajectory.write(escape_traj);
    } else {
        _escape_trajectory.write(escape_traj);
    }
    mEscapeTrajAvailable = false;
    return ret;
}

void Task::setTaskState(enum MplErrors err) {
    if(err != MPL_ERR_NONE && err != MPL_ERR_REPLANNING_NOT_REQUIRED) {
        LOG_INFO("Setting task state, receives error: %s", getTaskStateName(err).c_str());
    }
    switch(err) {
        case MPL_ERR_NONE: break; // Does not change the current state.
        case MPL_ERR_REPLANNING_NOT_REQUIRED: break; // Does not change the current state.
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
        case MPL_ERR_START_ON_OBSTACLE: state(START_ON_OBSTACLE); break;
        case MPL_ERR_GOAL_ON_OBSTACLE: state(GOAL_ON_OBSTACLE); break;
        case MPL_ERR_START_GOAL_ON_OBSTACLE: state(START_GOAL_ON_OBSTACLE); break;
        case MPL_ERR_SET_START_GOAL: state(SET_START_GOAL_ERROR); break;
        case MPL_ERR_GOAL_COULD_ONLY_BE_REACHED_IMPRECISELY: state(GOAL_COULD_ONLY_BE_REACHED_IMPRECISELY); break;
        default: state(UNDEFINED_ERROR); break;
    }
}

std::string Task::getTaskStateName(enum MplErrors err) {
    switch(err) {
        case MPL_ERR_NONE: return "No error";
        case MPL_ERR_REPLANNING_NOT_REQUIRED: return "Replanning not required";
        case MPL_ERR_MISSING_START: return "Missing: Start";
        case MPL_ERR_MISSING_GOAL: return "Missing: Goal";
        case MPL_ERR_MISSING_TRAV: return "Missing: Trav";
        case MPL_ERR_MISSING_START_GOAL: return "Missing: Start, Goal";
        case MPL_ERR_MISSING_START_TRAV: return "Missing: Start, Trav";
        case MPL_ERR_MISSING_GOAL_TRAV: return "Missing: Goal, Trav";
        case MPL_ERR_MISSING_START_GOAL_TRAV: return "Missing: Start, Goal, Trav";
        case MPL_ERR_PLANNING_FAILED: return "Planning failed";
        case MPL_ERR_WRONG_STATE_TYPE: return "Wrong state type (correct start/goal?)";
        case MPL_ERR_INITIALIZE_MAP: return "Map initialization failed";
        case MPL_ERR_START_ON_OBSTACLE: return "Start lies on an obstacle";
        case MPL_ERR_GOAL_ON_OBSTACLE: return "Goal lies on an obstacle";
        case MPL_ERR_START_GOAL_ON_OBSTACLE: return "Start and goal lie on an obstacle";
        case MPL_ERR_SET_START_GOAL: return "Start/goal could not be set";
        case MPL_ERR_GOAL_COULD_ONLY_BE_REACHED_IMPRECISELY: return "End of trajectory does not reach the goal position";
        default: return "Unknown state";
    }
}

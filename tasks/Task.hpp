/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef MOTION_PLANNING_LIBRARIES_TASK_HPP
#define MOTION_PLANNING_LIBRARIES_TASK_HPP

#include "motion_planning_libraries/TaskBase.hpp"

#include <envire/core/Environment.hpp>

namespace motion_planning_libraries {

class MotionPlanningLibraries;


/**
 * \mainpage MPL - ROCK Module
 *
 * \section Introduction
 * This describes the MotionPlanningLibraries ROCK Module and its integration 
 * into a small navigation stack.
 * 
 * \section MPL-Module
 * Within the MPL library documentation you can find an instruction how to 
 * setup the configuration struct. The property  \a traversability_map_id should be set
 * to the id of the traversability map (otherwise the first found traversability map will be used)
 * and \a planning_time_sec defines the maximum time which should be used for planning 
 * during each call of the updateHook.
 * 
 * The test module can be used to generate test traversability maps. See the test_planning script
 * for an example.
 * 
 * \section nav_stack Small Navigation Stack
 * The following image shows the ROCK modules which are required to set up a small navigation stack.
 * The GraphSlam module uses the odometry and the Velodyne laser scanner data to create a MLS map
 * and to calculcate the pose of the system within the GraphSlam frame. The created MLS
 * is converted into a traversability map which is used from the MPL-Module for planning. The 
 * current pose of the robot defines the starting pose and an additional goal pose
 * has to be provided. The planned trajectory is passed together with the current pose 
 * to the trajectory follower which generates a Motion2D command (containing forward and
 * rotational speed) which is sent to a specific robot-controller which converts
 * the Motion2D cmmand into motor commands. 
 * \image html simple_navigation_rock.jpg "Simple ROCK navigation stack"
 * 
 * \section start Execute Small Navigation Stack
 * To execute such an navigation stack deployments have to be created containing all relevant modules. 
 * In the following example all tasks are combined within one deployment.
 * \code{.unparsed}
 * using_task_library 'odometry'
 * using_task_library 'velodyne_lidar'
 * using_task_library 'graph_slam'
 * using_task_library 'traversability'
 * using_task_library 'motion_planning_libraries'
 * using_task_library 'trajectory_follower'
 * using_task_library 'robot_motioncontroller'
 * 
 * deployment 'example_deployment' do
 *     task("odometry", "odometry::Skid")
 *     task("velodyne", "velodyne_lidar::LaserScanner")
 *     task("graph_slam", "graph_slam::VelodyneSLAM")
 *     task('traversability', "traversability::Simple").triggered
 *     task("planner", "motion_planning_libraries::Task").periodic(1.0)
 *     task("follower", "trajectory_follower::Task").periodic(0.01)
 *     task("controller", "robot_motioncontroller::Task").periodic(0.01)
 * 
 *     add_default_logger
 * end
 * \endcode
 * 
 * After the deployments have been created a Ruby script can be used to start the navigation stack.
 * \code
#! /usr/bin/env ruby

require 'rock/bundle'
require 'readline'
require 'orocos'
require 'optparse'
require 'transformer/runtime'

Orocos::MQueue.auto = true

include Orocos
Orocos::CORBA.max_message_size = 840000000
Bundles.initialize

Bundles.transformer.load_conf(Bundles.find_file('config', 'transforms_scripts.rb'))

ENV['LANG'] = 'C'
ENV['LC_NUMERIC'] = 'C'

Bundles.run example_deployment,
        'wait' => 500,
        'output' => nil,
        'valgrind' => false  do

    Orocos.log_all_ports()

    # Get tasks.
    odometry = Orocos.name_service.get odometry
    velodyne = Orocos.name_service.get velodyne
    graph_slam = Orocos.name_service.get graph_slam
    traversability = Orocos.name_service.get traversability
    planner = Orocos.name_service.get planner
    follower = Orocos.name_service.get follower
    controller = Orocos.name_service.get controller
    
    # Set all configurations, e.g.: odometry.apply_conf(['default'])  
    
    # Setup transformer.
    Bundles.transformer.setup(odometry, velodyne, graph_slam, 
            traversability, planner, follower, controller)

    # Configure all tasks, e.g.: odometry.configure()

    # Connect ports.
    odometry.odometry_samples.connect_to graph_slam.odometry_samples, :type => :buffer, :size => 400
    velodyne.laser_scans.connect_to graph_slam.lidar_samples, :type => :buffer, :size => 100
    graph_slam.envire_map.connect_to traversability.mls_map
    graph_slam.pose_samples.connect_to planner.start_pose_samples
    graph_slam.pose_samples.connect_to follower.pose
    traversability.traversability_map.connect_to planner.traversability_map
    planner.trajectory.connect_to follower.trajectory
    follower.motion_command.connect_to controller.motion_command
    
    # Start all tasks, e.g.: odometry.start()

    Readline.readline "Hit ENTER to stop" 
end
 * \endcode
 *
 * \section todos TODOs
 * \todo "Check if a replanning has to be executed before changing to planning state."
 */    
class Task : public TaskBase
{
	friend class TaskBase;
    protected:
        boost::shared_ptr<MotionPlanningLibraries> mpMotionPlanningLibraries;
        envire::Environment mEnv;
        base::samples::RigidBodyState mStartPose;
        base::samples::RigidBodyState mGoalPose;
        State mStartState;
        State mGoalState;
        std::vector <base::Waypoint > mLastPath;
      
    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "motion_planning_libraries::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
        ~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         * 
         * Passes the traversability map and the start- and goal-pose to the planning library
         * and starts planning. 
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
        
        /**
         * Can be used if the system touches an obstacle to create a path
         * back to a area without any obstacles. This requires an already
         * planned path, otherwise an empty trajectory will be returned.
         * The esacpe trajectory is written to the port 'escape_trajectory'.
         */
        bool generateEscapeTrajectory();   
        
        /**
         * Returns the MPL library pointer.
         * \warning "Multithreading is not supported!"
         */
        inline boost::shared_ptr<MotionPlanningLibraries> getMPLPointer() {
            return mpMotionPlanningLibraries;
        }
        
        
    private:
        void setTaskState(enum MplErrors err);

    };
}

#endif


require 'orocos'
require 'vizkit'
require 'readline'

Orocos::CORBA.max_message_size = 12000000 # stucks if > than this

include Orocos
Orocos.initialize

# Content of these language variables must not be german, otherwise '.' and ','
# are mixed reading numbers and a bad_alloc error occurs loading the scenes.
ENV['LANG'] = 'C'
ENV['LC_NUMERIC'] = 'C'

Orocos.run  'motion_planning_libraries::Task' => 'planner',
            'motion_planning_libraries::Test' => 'test',
            "valgrind" => false,
            'output' => nil, 
            "wait" => 1000 do
          
    planner = TaskContext::get 'planner'
    planner.traversability_map_id = "trav" 
    planner.planning_time_sec = 10.0
    
    planner.config do |p|
        p.mPlanningLibType = :LIB_SBPL
        p.mEnvType = :ENV_XYTHETA
        p.mRobotWidth = 0.5
        p.mRobotLength = 0.5
        p.mRobotForwardVelocity = 1.0 # m/sec.
        p.mRobotBackwardVelocity = 0.5 # m/sec.
        p.mRobotRotationalVelocity = 0.4 # rad/sec.
        p.mSearchUntilFirstSolution = false
        
        # SBPL specific configuration
        p.mSBPLEnvFile = ""
        p.mSBPLMotionPrimitivesFile = File.join(ENV['AUTOPROJ_PROJECT_BASE'], '/external/sbpl/matlab/mprim/pr2_10cm.mprim')
        p.mSBPLForwardSearch = false # ADPlanner throws 'g-values are non-decreasing' if true
    end

    planner.configure
    planner.start

    test = TaskContext::get 'test'
    test.traversability_map_id = 'trav'
    test.traversability_map_type = 'RANDOM_CIRCLES'
    test.traversability_map_width_m = 120
    test.traversability_map_height_m = 10
    test.traversability_map_scalex =  0.1   
    test.traversability_map_scaley = 0.1
    test.number_of_random_circles = 100

    test.configure
    test.start
       
    test.traversability_map.connect_to(planner.traversability_map)
    test.start_pose_samples.connect_to(planner.start_pose_samples) 
    test.goal_pose_samples.connect_to(planner.goal_pose_samples)     
        
    t1 = Thread.new do
        while true do
            Readline::readline("Hit enter to generate a new test environment ... ")
            test.trigger
        end
    end

    Vizkit.display planner
    Vizkit.display test
    Vizkit.display test.port("traversability_map")
    Vizkit.display test.port("start_pose_samples"), :widget => Vizkit.default_loader.RigidBodyStateVisualization
    Vizkit.display test.port("goal_pose_samples"), :widget => Vizkit.default_loader.RigidBodyStateVisualization
    Vizkit.display planner.port("debug_goal_pose_samples"), :widget => Vizkit.default_loader.RigidBodyStateVisualization
    Vizkit.display planner.path
    Vizkit.display planner.waypoint_start
    Vizkit.display planner.waypoint_goal
    Vizkit.display planner.trajectory
    Vizkit.display planner.samples
 
    Vizkit.exec

    Readline::readline("Hit ENTER to stop")    
end
  

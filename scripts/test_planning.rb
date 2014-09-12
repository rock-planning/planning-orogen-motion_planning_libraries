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
    planner.planning_time_sec = 30.0
    
    planner.config do |p|
        p.mPlanningLibType = :LIB_OMPL
        p.mEnvType = :ENV_SHERPA
        p.mFootprintRadiusMinMax.first = 0.5
        p.mFootprintRadiusMinMax.second = 2.0
        p.mNumFootprintClasses = 10
        p.mTimeToAdaptFootprint = 10
        p.mAdaptFootprintPenalty = 0
        p.mRobotForwardVelocity = 0.8 # m/sec.
        p.mRobotBackwardVelocity = 0.4 # m/sec.
        p.mRobotRotationalVelocity = 0.05 # 0.2 # rad/sec.
        p.mSearchUntilFirstSolution = false
        p.mReplanDuringEachUpdate = true
        
        # SBPL specific configuration
        p.mSBPLEnvFile = ""
        p.mSBPLMotionPrimitivesFile = File.join(ENV['AUTOPROJ_CURRENT_ROOT'], '/external/sbpl/matlab/mprim/pr2_10cm.mprim')
        p.mSBPLForwardSearch = false # ADPlanner throws 'g-values are non-decreasing' if true
    end

    planner.configure
    planner.start

    test = TaskContext::get 'test'
    test.traversability_map_id = 'trav'
    test.traversability_map_type = 'SMALL_OPENING'
    test.traversability_map_width_m = 120
    test.traversability_map_height_m = 10
    test.traversability_map_scalex =  0.1   
    test.traversability_map_scaley = 0.1
    test.number_of_random_circles = 50
    test.opening_length = 2.0

    test.configure
    test.start
       
    test.traversability_map.connect_to(planner.traversability_map)
    test.start_state.connect_to(planner.start_state) 
    test.goal_state.connect_to(planner.goal_state)     
        
    t1 = Thread.new do
        while true do
            Readline::readline("Hit enter to generate a new test environment ... ")
            test.trigger
        end
    end

    Vizkit.display planner
    Vizkit.display test.port("traversability_map")
    #Vizkit.display test.port("start_pose_samples"), :widget => Vizkit.default_loader.RigidBodyStateVisualization
    #Vizkit.display test.port("goal_pose_samples"), :widget => Vizkit.default_loader.RigidBodyStateVisualization
    #Vizkit.display planner.port("debug_goal_pose_samples"), :widget => Vizkit.default_loader.RigidBodyStateVisualization
    #Vizkit.display planner.states
    #Vizkit.display planner.waypoint_start
    #Vizkit.display planner.waypoint_goal
    Vizkit.display planner.trajectory
 
    Vizkit.exec

    Readline::readline("Hit ENTER to stop")    
end
  

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
    planner.planning_time_sec = 4.0
    
    planner.config do |p|
        p.mPlanningLibType = :LIB_SBPL
        p.mEnvType = :ENV_XYTHETA
        p.mPlanner = :ANYTIME_DSTAR
        p.mFootprintLengthMinMax.first = 1.0
        p.mFootprintLengthMinMax.second = 1.0
        p.mFootprintWidthMinMax.first = 1.0
        p.mFootprintWidthMinMax.second = 1.0
        p.mMaxAllowedSampleDist = -1
        p.mNumFootprintClasses = 10
        p.mTimeToAdaptFootprint = 10
        p.mAdaptFootprintPenalty = 2
        p.mSearchUntilFirstSolution = false
        p.mReplanDuringEachUpdate = true
        p.mUseIntermediatePoints = true
        p.mNumPrimPartition = 2
        
        # EO2
        p.mSpeeds.mSpeedForward = 0.12
        p.mSpeeds.mSpeedBackward = 0.00
        p.mSpeeds.mSpeedLateral = 0.0
        p.mSpeeds.mSpeedTurn = 0.1
        p.mSpeeds.mSpeedPointTurn = 0.0 # 0.15
        p.mSpeeds.mMultiplierForward = 1
        p.mSpeeds.mMultiplierBackward = 4
        p.mSpeeds.mMultiplierLateral = 3
        p.mSpeeds.mMultiplierTurn = 2
        p.mSpeeds.mMultiplierPointTurn = 5
        
        # SBPL specific configuration
        p.mSBPLEnvFile = ""
        p.mSBPLMotionPrimitivesFile = ""
        #p.mSBPLMotionPrimitivesFile = File.join(ENV['AUTOPROJ_CURRENT_ROOT'], '/external/sbpl/matlab/mprim/pr2_10cm.mprim')
        #p.mSBPLMotionPrimitivesFile = File.join(ENV['AUTOPROJ_CURRENT_ROOT'], '/external/sbpl/matlab/mprim/output_unicycle.mprim')
        #p.mSBPLMotionPrimitivesFile = File.join(ENV['AUTOPROJ_CURRENT_ROOT'], 'planning/motion_planning_libraries/scripts/sbpl/unicycle_output.mprim')
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
    Vizkit.display planner.start_pose_samples_debug, :widget => Vizkit.default_loader.RigidBodyStateVisualization
    Vizkit.display planner.goal_pose_samples_debug, :widget => Vizkit.default_loader.RigidBodyStateVisualization
    Vizkit.display planner.sbpl_mprims_debug
 
    Vizkit.exec

    Readline::readline("Hit ENTER to stop")    
end
  

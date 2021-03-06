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
            'motion_planning_libraries::Test' => 'test' do
            #"valgrind" => false,
            #'output' => nil, 
            #"wait" => 1000 do
          
    planner = TaskContext::get 'planner'
    planner.traversability_map_id = "trav" 
    planner.planning_time_sec = 20.0
    planner.only_provide_optimal_trajectories = true
    
    planner.config do |p|
        p.mPlanningLibType = :LIB_SBPL
        p.mEnvType = :ENV_XYTHETA
        p.mPlanner = :UNDEFINED_PLANNER
        p.mFootprintLengthMinMax.first = 1.0
        p.mFootprintLengthMinMax.second = 1.0
        p.mFootprintWidthMinMax.first = 1.0
        p.mFootprintWidthMinMax.second = 1.0
        p.mFootprintRadiusMinMax.first =  0.6
        p.mFootprintRadiusMinMax.second = 1.58 
        p.mMaxAllowedSampleDist = 2.0
        p.mNumFootprintClasses = 10
        p.mTimeToAdaptFootprint = 20
        p.mAdaptFootprintPenalty = 2
        p.mSearchUntilFirstSolution = false
        p.mNumIntermediatePoints = 8
        p.mNumPrimPartition = 8
        p.mPrimAccuracy = 0.15
        
        p.mReplanning.mReplanDuringEachUpdate = false
        p.mReplanning.mReplanOnNewStartPose = false
        p.mReplanning.mReplanOnNewGoalPose = true
        p.mReplanning.mReplanOnNewMap = false
        p.mReplanning.mReplanMinDistStartGoal = 2.0
        
        # EO2
        p.mMobility.mSpeed = 0.5
        p.mMobility.mTurningSpeed = 0.15
        p.mMobility.mMultiplierForward = 1
        p.mMobility.mMultiplierBackward = 2
        p.mMobility.mMultiplierLateral = 3
        p.mMobility.mMultiplierForwardTurn = 2
        p.mMobility.mMultiplierBackwardTurn = 3
        p.mMobility.mMultiplierPointTurn = 4
        p.mMobility.mMultiplierLateralCurve = 4
        p.mMobility.mMinTurningRadius = 1.0
        
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
    test.number_of_random_circles = 10
    test.opening_length = 1.8
    test.footprint_min = 0.6
    test.footprint_max = 1.58
    

    test.configure
    test.start
       
    test.traversability_map.connect_to(planner.traversability_map)
    test.start_state.connect_to(planner.start_state) 
    test.goal_state.connect_to(planner.goal_state)     
        
    #t1 = Thread.new do
        while true do
            Readline::readline("Hit enter to generate a new test environment ... ")
            test.trigger
        end
    #end
#=begin
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
#=end
    Readline::readline("Hit ENTER to stop")    
end
  

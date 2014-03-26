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

Orocos.run  'global_path_planner::Task' => 'planner',
            'global_path_planner::Test' => 'test',
            "valgrind" => false, 
            "wait" => 1000 do
          
    planner = TaskContext::get 'planner'
    planner.traversability_map_id = "trav"
    planner.configure
    planner.start

    test = TaskContext::get 'test'
    test.traversability_map_id = 'trav'
    test.traversability_map_type = 'RANDOM_CIRCLES'
    test.traversability_map_width_m = 30
    test.traversability_map_height_m = 30
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
  

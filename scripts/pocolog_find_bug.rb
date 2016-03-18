require 'orocos/log'
require 'rock/bundle'
require 'orocos'
require 'vizkit'
require 'readline'

Orocos::CORBA.max_message_size = 12000000 # stucks if > than this

include Orocos
Bundles.initialize

#Bundles.transformer.load_conf(Bundles.find_file('config', 'transforms_scripts.rb'))

# Content of these language variables must not be german, otherwise '.' and ','
# are mixed reading numbers and a bad_alloc error occurs loading the scenes.
ENV['LANG'] = 'C'
ENV['LC_NUMERIC'] = 'C'


require 'pocolog'
include Pocolog

Bundles.run 'motion_planning_libraries::Task' => 'global_planner',
        'motion_planning_libraries::Test' => 'test',
        'output' => nil,
        'wait' => 1000 do

    planner = Orocos.name_service.get 'global_planner'
    planner.apply_conf(['default'])    
    planner.configure
    planner.start
    
    #Vizkit.display planner.trajectory  
    #Vizkit.display planner.port("start_pose_samples_debug"), :widget => Vizkit.default_loader.RigidBodyStateVisualization
    #Vizkit.display planner.port("goal_pose_samples_debug"), :widget => Vizkit.default_loader.RigidBodyStateVisualization  
    
    # Write only trav map.
    #file = Logfiles.new File.open('/media/research/projects/all/TransTerrA_15004/documentation/experiments/WP4400-path_and_motion_planning/20160314-1826_strange_path_planner/traversability.0.log')
    #data_stream = file.stream("/traversability.traversability_map")
    #trav_writer = planner.traversability_map.writer
    #data_stream.samples.each do |realtime, logical, sample|
    #    trav_writer.write(sample)
    #end
    
    test = TaskContext::get 'test'
    test.traversability_map_id = 'trav'
    test.traversability_map_type = 'CLEAR'
    test.traversability_map_width_m = 200
    test.traversability_map_height_m = 200
    test.traversability_map_scalex =  0.1   
    test.traversability_map_scaley = 0.1
    test.number_of_random_circles = 10
    test.opening_length = 3.0

    test.configure
    test.start
       
    test.traversability_map.connect_to(planner.traversability_map)
    
    test.trigger
    
    # Write 0,0,0 goal pose.
    goal_writer = planner.goal_pose_samples.writer
    goal_pose = goal_writer.new_sample
    goal_pose.position[0] = 1
    goal_pose.position[1] = 1
    goal_pose.position[2] = 0
    goal_pose.orientation = Eigen::Quaternion.from_angle_axis( 
                 0, Eigen::Vector3.new( 0, 0, 1 ) )
    goal_writer.write(goal_pose)
    
    
    # Write start poses around 1,0,0.
    file_planner = Logfiles.new File.open('/media/research/projects/all/TransTerrA_15004/documentation/experiments/WP4400-path_and_motion_planning/20160314-1826_strange_path_planner/planner.0.log')
    data_stream = file_planner.stream("/planner.start_pose_samples_debug")
    data_stream.samples.each do |realtime, logical, sample|
        puts "Start position: #{sample.position}"
    end

    start_writer = planner.start_pose_samples.writer
    start_pose = start_writer.new_sample
    start_pose.position[0] = 1.72
    start_pose.position[1] = 0.62 #-0.38
    start_pose.position[2] = 0
    start_pose.orientation = Eigen::Quaternion.from_angle_axis( 
                 0, Eigen::Vector3.new( 0, 0, 1 ) )
    start_writer.write(start_pose)

=begin    
    data_stream = file_planner.stream("/planner.start_pose_samples_debug")
    start_writer = planner.start_pose_samples.writer
    data_stream.samples.each do |realtime, logical, sample|
      
      if(sample.position[0] < 0.73 && sample.position[0] > 0.71 && sample.position[1] > -0.39 && sample.position[1] < -0.37)
          puts "Using start position: #{sample.position}"
          start_writer.write(sample)
          Readline.readline "Hit ENTER to test next start pose"
      end
    end
=end

    #Vizkit.control replay
    #Vizkit.exec
#    replay.run

    Readline.readline "Hit ENTER to stop"
end


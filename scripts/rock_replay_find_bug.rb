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
    file = Logfiles.new File.open('/media/research/projects/all/TransTerrA_15004/documentation/experiments/WP4400-path_and_motion_planning/20160317-1834_strange_planner2/traversability.0.log')
    data_stream = file.stream("/traversability.traversability_map")
    trav_writer = planner.traversability_map.writer
    data_stream.samples.each do |realtime, logical, sample|
        trav_writer.write(sample)
        break
    end
    
    # Write 0,0,0 goal pose.
    goal_writer = planner.goal_pose_samples.writer
    goal_pose = goal_writer.new_sample
    goal_pose.position[0] = 2
    goal_pose.position[1] = 0
    goal_pose.position[2] = 0
    goal_pose.orientation = Eigen::Quaternion.from_angle_axis( 
                 0, Eigen::Vector3.new( 0, 0, 1 ) )
    goal_writer.write(goal_pose)
    
    
    # Write start poses around 1,0,0.
    file_planner = Logfiles.new File.open('/media/research/projects/all/TransTerrA_15004/documentation/experiments/WP4400-path_and_motion_planning/20160317-1834_strange_planner2/planner.0.log')
    data_stream = file_planner.stream("/planner.start_pose_samples_debug")
    data_stream.samples.each do |realtime, logical, sample|
        puts "Start position: #{sample.position}"
    end
    
    data_stream = file_planner.stream("/planner.start_pose_samples_debug")
    start_writer = planner.start_pose_samples.writer
    data_stream.samples.each do |realtime, logical, sample|
      
      #if(sample.position[0] < 1.9 && sample.position[0] > 1.79 && sample.position[1] > 0.0 && sample.position[1] < 0.11)
          puts "Using start position: #{sample.position}"
          start_writer.write(sample)
          Readline.readline "Hit ENTER to test next start pose"
      #end
    end


    #Vizkit.control replay
    #Vizkit.exec
#    replay.run

    Readline.readline "Hit ENTER to stop"
end


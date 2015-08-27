require 'orocos/log'
require 'rock/bundle'
require 'orocos'
require 'vizkit'
require 'readline'

Orocos::CORBA.max_message_size = 12000000 # stucks if > than this

include Orocos
Bundles.initialize

Bundles.transformer.load_conf(Bundles.find_file('config', 'transforms_scripts.rb'))

# Content of these language variables must not be german, otherwise '.' and ','
# are mixed reading numbers and a bad_alloc error occurs loading the scenes.
ENV['LANG'] = 'C'
ENV['LC_NUMERIC'] = 'C'

replay = Log::Replay.open('20150825-1618')

Bundles.run 'motion_planning_libraries::Task' => 'global_planner',
        'output' => nil,
        'wait' => 1000 do

    planner = Orocos.name_service.get 'global_planner'
    planner.apply_conf(['default'])    
    planner.configure
    planner.start

    slam = replay.velodyne_slam
    trav = replay.traversability
    planner_replay = replay.global_planner
    exploration = replay.exploration
    
    Vizkit.display planner_replay.trajectory
    
    trav.traversability_map.connect_to(planner.traversability_map)
    slam.pose_samples.connect_to(planner.start_pose_samples)
    planner_replay.goal_pose_samples_debug.connect_to(planner.goal_pose_samples)
    
    t1 = Thread.new do
        goal_poses_reader = exploration.goals_out.reader
        goal_pose_writer = planner.goal_pose_samples.writer
        while true
            poses = nil
            while(!poses) do
                poses = goal_poses_reader.read_new()
                sleep 0.1
            end
            goal_pose_writer.write(poses[0])
            sleep 0.1
        end
    end

    Vizkit.control replay
    Vizkit.exec
#    replay.run

    Readline.readline "Hit ENTER to stop"
end


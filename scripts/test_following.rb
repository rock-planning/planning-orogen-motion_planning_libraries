require 'orocos'
require 'vizkit'
require 'readline'
require 'rock/bundle'

Orocos::CORBA.max_message_size = 12000000 # stucks if > than this

include Orocos
Orocos::CORBA.max_message_size = 120000000
Bundles.initialize

# Content of these language variables must not be german, otherwise '.' and ','
# are mixed reading numbers and a bad_alloc error occurs loading the scenes.
ENV['LANG'] = 'C'
ENV['LC_NUMERIC'] = 'C'

Bundles.run  'trajectory_follower::Task' => 'follower',
            'motion_planning_libraries::FollowingTest' => 'move',
            'sherpa_tt_footprint::Task' => 'footprint',
            "valgrind" => false,
            'output' => nil, 
            "wait" => 1000 do
    
    # Running tasks.      
    planner = TaskContext::get 'planner'
    test = TaskContext::get 'test'
    
    # New tasks.
    follower = TaskContext::get 'follower'
    move = TaskContext::get 'move'
    footprint = TaskContext::get 'footprint'
    
    Orocos.conf.apply( follower, ['default'] )
    footprint.footprint_change_threshold = 0.1
    
    follower.configure
    move.configure
    footprint.configure
    
    test.start_pose_sample.connect_to follower.robot_pose, :type => :buffer, :size => 10
    test.start_pose_sample.connect_to move.start_pose, :type => :buffer, :size => 10
    planner.trajectory.connect_to follower.trajectory, :type => :buffer, :size => 10
    follower.motion_command.connect_to move.motion_command, :type => :buffer, :size => 10
    move.robot_pose.connect_to follower.robot_pose, :type => :buffer, :size => 10
    move.robot_pose.connect_to footprint.robot_pose_debug, :type => :buffer, :size => 10
    planner.states_mpl.connect_to footprint.states_mpl, :type => :buffer, :size => 10
    follower.follower_data.connect_to footprint.follower_data, :type => :buffer, :size => 10
    
    
    follower.start
    move.start
    footprint.start

    Readline::readline("Hit ENTER to stop")    
end
  

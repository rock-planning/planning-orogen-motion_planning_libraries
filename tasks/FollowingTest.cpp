/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "FollowingTest.hpp"
#include <base-logging/Logging.hpp>

using namespace motion_planning_libraries;

FollowingTest::FollowingTest(std::string const& name)
    : FollowingTestBase(name), mStartPose(), mStartPoseReceived(false), 
        mTimeStart()
{
}

FollowingTest::FollowingTest(std::string const& name, RTT::ExecutionEngine* engine)
    : FollowingTestBase(name, engine), mStartPose(), mStartPoseReceived(false), 
        mTimeStart()
{
}

FollowingTest::~FollowingTest()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See FollowingTest.hpp for more detailed
// documentation about them.

bool FollowingTest::configureHook()
{
    if (! FollowingTestBase::configureHook())
        return false;
    return true;
}

bool FollowingTest::startHook()
{
    if (! FollowingTestBase::startHook())
        return false;  
    return true;
}

void FollowingTest::updateHook()
{
    FollowingTestBase::updateHook();
    
    if(_start_pose.readNewest(mStartPose) == RTT::NewData) {
        mCurrentPose = mStartPose;
        mStartPoseReceived = true;
        LOG_INFO("Start pose received\n");
    }  
    
    if(!mStartPoseReceived) {
        return;
    }
    
    base::commands::Motion2D motion_cmd;
    if(_motion_command.readNewest(motion_cmd) == RTT::NewData) { 
        base::Time time_now = base::Time::now();
        double time_sec = (time_now - mTimeStart).toSeconds();
        
        // Avoids the big initial cap.
        if(time_sec > 2) {
            mTimeStart = base::Time::now();
            time_sec = 0;
        }
         
        double translation = motion_cmd.translation * time_sec;
        double rotation = motion_cmd.rotation * time_sec;
        mTimeStart = time_now;
        LOG_DEBUG("Passed time %4.2f, translation %4.2f, rotation %4.2f\n", 
                time_sec, translation, rotation);
        
        LOG_DEBUG("x %4.2f y %4.2f z %4.2f yaw %4.2f\n",
                mCurrentPose.position[0], mCurrentPose.position[1], mCurrentPose.position[2], 
                mCurrentPose.getYaw());
                
        base::Vector3d mov_vec(translation, 0, 0);
        mov_vec = mCurrentPose.orientation * mov_vec;
        mov_vec = Eigen::AngleAxis<double>(rotation, Eigen::Vector3d::UnitZ()) * mov_vec;
        mCurrentPose.position = mCurrentPose.position + mov_vec;
        mCurrentPose.orientation = Eigen::AngleAxis<double>(mCurrentPose.getYaw() + 
                rotation, Eigen::Vector3d::UnitZ()); 
        
        LOG_DEBUG("x %4.2f y %4.2f z %4.2f yaw %4.2f\n",
                mCurrentPose.position[0], mCurrentPose.position[1], mCurrentPose.position[2], 
                mCurrentPose.getYaw());
                
        _robot_pose.write(mCurrentPose);
    }  
}

void FollowingTest::errorHook()
{
    FollowingTestBase::errorHook();
}

void FollowingTest::stopHook()
{
    FollowingTestBase::stopHook();
}

void FollowingTest::cleanupHook()
{
    FollowingTestBase::cleanupHook();
}

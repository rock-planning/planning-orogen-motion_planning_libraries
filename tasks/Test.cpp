/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Test.hpp"

#include <stdlib.h>
#include <time.h> 

#include <envire/maps/TraversabilityGrid.hpp>
#include <envire/operators/SimpleTraversability.hpp>
#include <orocos/envire/Orocos.hpp>

using namespace global_path_planner;

Test::Test(std::string const& name)
    : TestBase(name), mpFrameNode(NULL), mpTravGrid(NULL)
{
}

Test::Test(std::string const& name, RTT::ExecutionEngine* engine)
    : TestBase(name, engine), mpFrameNode(NULL), mpTravGrid(NULL)
{
}

Test::~Test()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Test.hpp for more detailed
// documentation about them.

bool Test::configureHook()
{
    if (! TestBase::configureHook())
        return false;
    return true;
}

bool Test::startHook()
{
    if (! TestBase::startHook())
        return false;
    return true;
}

void Test::updateHook()
{
    TestBase::updateHook();
    
    int width = _traversability_map_width_m.get();
    int height = _traversability_map_height_m.get();
    base::samples::RigidBodyState start = createRandomRBS(width, height);
    _start_pose_samples.write(start);
    
    base::samples::RigidBodyState goal = createRandomRBS(width, height);
    _goal_pose_samples.write(goal);
    
    createTraversabilityMap();
    envire::OrocosEmitter emitter_tmp(&mEnv, _traversability_map);
    emitter_tmp.setTime(base::Time::now());
    emitter_tmp.flush();
}

void Test::errorHook()
{
    TestBase::errorHook();
}

void Test::stopHook()
{
    TestBase::stopHook();
}

void Test::cleanupHook()
{
    TestBase::cleanupHook();
}

base::samples::RigidBodyState Test::createRandomRBS(int max_width_m, int max_height_m) {
    base::samples::RigidBodyState rbs;
    srand (time(NULL));
    
    rbs.position = base::Vector3d(rand() % max_width_m, rand() % max_height_m, 0);
    
    // Create an angle between 0 and 360 in radians.
    double rot_radians = (((rand() % 360) / 360.0) / 180) * M_PI; 
    rbs.orientation = Eigen::AngleAxis<double>(rot_radians, base::Vector3d(0,0,1));
    
    return rbs;
}

void Test::createTraversabilityMap() {

    LOG_INFO("Create traversability map");

    if(mpTravGrid != NULL) {
        mEnv.detachItem(mpTravGrid);
    }
    if(mpFrameNode != NULL) {
        mEnv.detachItem(mpFrameNode);
    }

    envire::TraversabilityGrid* trav = new envire::TraversabilityGrid(
            (size_t)_traversability_map_width_m.get() / _traversability_map_scalex.get(), 
            (size_t)_traversability_map_height_m.get() / _traversability_map_scaley.get(), 
            _traversability_map_scalex.get(), 
            _traversability_map_scaley.get());
    mEnv.attachItem(trav);
    mpTravGrid = trav;
    
    //Set a random pose of the traversability map.
    base::samples::RigidBodyState rbs = createRandomRBS(10, 10);
    envire::FrameNode* frame_node = new envire::FrameNode(/*rbs.getTransform()*/);
    mEnv.getRootNode()->addChild(frame_node);
    trav->setFrameNode(frame_node);
    mpFrameNode = frame_node;
   
    switch (_traversability_map_type.get()) {
        case CLEAR: {
            break;
        }
        case RANDOM_CIRCLES: {
            int num = rand() % 10 + 5;
            int center_x = 0, center_y = 0, radius = 0;
            int num_cells_x = _traversability_map_width_m.get() / 
                    _traversability_map_scalex.get();
            int num_cells_y = _traversability_map_height_m.get() / 
                    _traversability_map_scaley.get();
            for(int i=0; i<num; ++i) {
                center_x = rand() % num_cells_x;
                center_y = rand() % num_cells_y;
                radius = rand() % (int)(3 / _traversability_map_scalex.get()) + 1;
                drawCircle(trav, center_x, center_y, radius);
            }
            break;
        }
        default: {
            LOG_WARN("Trav map type unknown");
            break;
        }
    }
}

void Test::drawCircle(envire::TraversabilityGrid* trav, unsigned int center_x, 
        unsigned int center_y, int radius) {
        
    LOG_INFO("Draw circle at (%d,%d) with radius %d", center_x, center_y, radius);
    envire::TraversabilityGrid::ArrayType& trav_array = trav->getGridData();
    int start_x = center_x - radius;
    int end_x = center_x + radius;
    int start_y = center_y - radius;
    int end_y = center_y + radius;
    
    if(start_x < 0)
        start_x = 0;
    if(end_x > (int)trav->getCellSizeX())
        end_x = trav->getCellSizeX();
    if(start_y < 0)
        start_y = 0;
    if(end_y > (int)trav->getCellSizeY())
        end_y = trav->getCellSizeY();
        
    for(int x=start_x; x < end_x; ++x) {
        for(int y=start_y; y < end_y; ++y) {
            if(dist(x,center_x,y,center_y) < radius) {
                trav_array[y][x] = envire::SimpleTraversability::CLASS_OBSTACLE;
            }
        }
    }
}

double Test::dist(int x1, int y1, int x2, int y2) {
    return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}

/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Test.hpp"

#include <stdlib.h>
#include <time.h> 

#include <envire/maps/TraversabilityGrid.hpp>
#include <envire/operators/SimpleTraversability.hpp>
#include <orocos/envire/Orocos.hpp>

#include <global_path_planner/GlobalPathPlanner.hpp>

using namespace global_path_planner;

Test::Test(std::string const& name)
    : TestBase(name), mpEnv(NULL), mpFrameNode(NULL), mpTravGrid(NULL)
{
    srand (time(NULL));
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
    
    createTraversabilityMap();
    envire::OrocosEmitter emitter_tmp(mpEnv, _traversability_map);
    emitter_tmp.setTime(base::Time::now());
    emitter_tmp.flush();
    
    int width = _traversability_map_width_m.get() / _traversability_map_scalex.get();
    int height = _traversability_map_height_m.get() / _traversability_map_scaley.get();

    // Create random grid poses (start and goal) and transform them to the world.
    base::samples::RigidBodyState start;
    GlobalPathPlanner::grid2world(mpTravGrid, createRandomGridPose(width, height), start);
    _start_pose_samples.write(start);
    
    base::samples::RigidBodyState goal;
    GlobalPathPlanner::grid2world(mpTravGrid, createRandomGridPose(width, height), goal);
    _goal_pose_samples.write(goal);
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



base::samples::RigidBodyState Test::createRandomGridPose(int max_width_m, int max_height_m) {
    base::samples::RigidBodyState rbs;
    
    rbs.position = base::Vector3d(rand() % max_width_m, rand() % max_height_m, 0);
    
    // Create an angle between 0 and 360 in radians.
    double rot_radians = ((rand() % 360) / 180.0) * M_PI; 
    std::cout << "Rotation " << rot_radians << std::endl;
    rbs.orientation = Eigen::AngleAxis<double>(rot_radians, base::Vector3d(0,0,1));
    
    rbs.time = base::Time::now();
    
    return rbs;
}

void Test::createTraversabilityMap() {

    LOG_INFO("Create traversability map");
    
    // Using always the same environment leads to visualization-problems of 
    // the map in Vizkit.
    if(mpEnv != NULL) {
        delete mpEnv;
    }
    mpEnv = new envire::Environment();

    envire::TraversabilityGrid* trav = new envire::TraversabilityGrid(
            (size_t)_traversability_map_width_m.get() / _traversability_map_scalex.get(), 
            (size_t)_traversability_map_height_m.get() / _traversability_map_scaley.get(), 
            _traversability_map_scalex.get(), 
            _traversability_map_scaley.get());
    mpEnv->attachItem(trav);
    mpTravGrid = trav;
    
    // Set a random pose of the traversability map (grid coordinates used as meters).
    mRBSTravGrid = createRandomGridPose(10, 10);
    LOG_INFO("Test: Map position (%4.2f, %4.2f)", mRBSTravGrid.position[0], mRBSTravGrid.position[1]);
    envire::FrameNode* frame_node = new envire::FrameNode(mRBSTravGrid.getTransform());
    mpEnv->getRootNode()->addChild(frame_node);
    trav->setFrameNode(frame_node);
    mpFrameNode = frame_node;
   
    switch (_traversability_map_type.get()) {
        case CLEAR: {
            break;
        }
        case RANDOM_CIRCLES: {
            static int num = rand() % 10 + 5;
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
            if(dist(x,y,center_x,center_y) < radius) {
                trav_array[y][x] = envire::SimpleTraversability::CLASS_OBSTACLE;
            }
        }
    }
}

double Test::dist(int x1, int y1, int x2, int y2) {
    double dist = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
    return dist;
}

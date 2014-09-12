/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Test.hpp"

#include <stdlib.h>
#include <time.h> 

#include <envire/core/Environment.hpp>
#include <envire/maps/TraversabilityGrid.hpp>
#include <envire/operators/SimpleTraversability.hpp>
#include <orocos/envire/Orocos.hpp>

#include <motion_planning_libraries/MotionPlanningLibraries.hpp>

using namespace motion_planning_libraries;

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

    // Create and write start and goal pose.
    State start, goal;
    createStartGoalState(_traversability_map_width_m.get(), 
            _traversability_map_height_m.get(), start, goal);

    _start_state.write(start);
    _goal_state.write(goal);
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

// PRIVATE
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
    
    // Have to create a shared_ptr to the real trav data.
    //mpTravData = boost::shared_ptr<TravData>(new TravData(
    //        mpTravGrid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY)));
    // Creates a shared-pointer from the passed reference.
    TravData& travData = mpTravGrid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY);
    mpTravData = boost::shared_ptr<TravData>(&travData, NullDeleter());
    
    
    // Set a random pose of the traversability map.
    mRBSTravGrid = createPose(10, 10, rand(), rand(), rand());
    LOG_INFO("Test: Map position (%4.2f, %4.2f)", mRBSTravGrid.position[0], mRBSTravGrid.position[1]);
    envire::FrameNode* frame_node = new envire::FrameNode(/*mRBSTravGrid.getTransform()*/);
    mpEnv->getRootNode()->addChild(frame_node);
    trav->setFrameNode(frame_node);
    mpFrameNode = frame_node;
    
    // Defines driveability values.
    trav->setTraversabilityClass(0, envire::TraversabilityClass(0.5)); // unknown
    // class 1 (obstacle) to 10 -> driveability 0.0 to 1.0
    for(int i=0; i < 10; ++i) {  
        trav->setTraversabilityClass(i+1, envire::TraversabilityClass(i/10.0));
    }
    
    // Pass trav grid to helper class.
    mGridCalculations.setTravGrid(trav, mpTravData);
    
    int num_cells_x = _traversability_map_width_m.get() / 
            _traversability_map_scalex.get();
    int num_cells_y = _traversability_map_height_m.get() / 
            _traversability_map_scaley.get();
   
    switch (_traversability_map_type.get()) {
        case CLEAR: {
            break;
        }
        case RANDOM_CIRCLES: {
            static int num = _number_of_random_circles.get();
            int center_x = 0, center_y = 0, radius = 0;
            int cost_class = 0;
            // 1/2 obstacles        
            for(int i=0; i<num/2; ++i) {
                center_x = rand() % num_cells_x;
                center_y = rand() % num_cells_y;
                radius = rand() % (int)(3 / _traversability_map_scalex.get()) + 1;
                drawCircle(trav, center_x, center_y, radius, 1); // obstacle
            }
            // 1/2 other classes 2 - 10
            for(int i=0; i<num/2.0+0.5; ++i) {
                center_x = rand() % num_cells_x;
                center_y = rand() % num_cells_y;
                radius = rand() % (int)(3 / _traversability_map_scalex.get()) + 1;
                cost_class = rand() % 9 + 2;
                drawCircle(trav, center_x, center_y, radius, cost_class);
            }
            break;
        }
        case RANDOM_RECTANGLES: {
            static int num = _number_of_random_circles.get();
            int center_x = 0, center_y = 0;
            double orientation = 0;
            int cost_class = 0;
            int width = 0;
            int length = 0;
            
            // 1/2 obstacles        
            for(int i=0; i<num/2; ++i) {
                center_x = rand() % num_cells_x;
                center_y = rand() % num_cells_y;
                orientation = (((rand() % 360) - 180) / 180.0) * M_PI;
                width = rand() % (int)(3 / _traversability_map_scaley.get()) + 1;
                length = rand() % (int)(3 / _traversability_map_scalex.get()) + 1;
                mGridCalculations.setFootprintRectangleInGrid(length, width);
                mGridCalculations.setFootprintPoseInGrid(center_x, center_y, orientation);
                mGridCalculations.setValue(1); // obstacle
            }
            // 1/2 other classes 2 - 10
            for(int i=0; i<num/2.0+0.5; ++i) {
                center_x = rand() % num_cells_x;
                center_y = rand() % num_cells_y;
                orientation = (((rand() % 360) - 180) / 180.0) * M_PI;
                width = rand() % (int)(3 / _traversability_map_scaley.get()) + 1;
                length = rand() % (int)(3 / _traversability_map_scalex.get()) + 1;
                mGridCalculations.setFootprintRectangleInGrid(length, width);
                mGridCalculations.setFootprintPoseInGrid(center_x, center_y, orientation);
                cost_class = rand() % 9 + 2;
                mGridCalculations.setValue(cost_class);
            }
            
            break;
        }
        case SMALL_OPENING: {
            int length_wall = (num_cells_y - _opening_length.get() / _traversability_map_scaley.get()) / 2.0;
            drawRectangle(trav, num_cells_x/2, 0,                         10, length_wall, 1); // obstacle    
            drawRectangle(trav, num_cells_x/2, num_cells_y - length_wall, 10, length_wall, 1); // obstacle 
            
            // Test grid calculations
            mGridCalculations.setFootprintCircleInGrid(10);
            mGridCalculations.setFootprintPoseInGrid(20, 20, 0);
            mGridCalculations.setValue(1);
            
            mGridCalculations.setFootprintRectangleInGrid(20, 20);
            mGridCalculations.setFootprintPoseInGrid(20, 50, 0);
            mGridCalculations.setValue(1);
            break;
        }
        default: {
            LOG_WARN("Trav map type unknown");
            break;
        }
    }
}

void Test::createStartGoalState(int trav_width, int trav_height, State& start, State& goal) {
    switch (_traversability_map_type.get()) {
        case SMALL_OPENING: {    
            start.setPose(createPose(trav_width, trav_height, trav_width * 0.25, trav_height * 0.25, 180));
            start.mFootprintRadius = _footprint_max.get();
            goal.setPose(createPose (trav_width, trav_height, trav_width * 0.75, trav_height * 0.75, 180));
            goal.mFootprintRadius = _footprint_max.get();
            break;
        }
        default: {
           start.mPose = createPose(trav_width, trav_height, rand(), rand(), rand());
           createFootprint(_footprint_min.get(), _footprint_max.get(), start.mFootprintRadius);
           goal.mPose = createPose(trav_width, trav_height, rand(), rand(), rand());
           createFootprint(_footprint_min.get(), _footprint_max.get(), start.mFootprintRadius);
        }
    }       
}

base::samples::RigidBodyState Test::createPose(int width, int height, 
        int x, int y, unsigned int theta_degree) {
    base::samples::RigidBodyState rbs;
    
    rbs.position = base::Vector3d(x % width, y % height, 0);
    
    // Create an angle in radians from -179 to 180 (required by OMPL)
    int rot_degree = (theta_degree % 360) - 180; 
    if(rot_degree == -180) {
        rot_degree = 180;
    }
    double rot_radians = (rot_degree / 180.0) * M_PI;
    std::cout << "Rot radians " << rot_radians << std::endl;
    rbs.orientation = Eigen::AngleAxis<double>(rot_radians, base::Vector3d(0,0,1));
    
    rbs.time = base::Time::now();
    
    return rbs;
}

void Test::drawCircle(envire::TraversabilityGrid* trav, unsigned int center_x, 
        unsigned int center_y, int radius, int cost_class) {
        
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
                trav_array[y][x] = cost_class; //envire::SimpleTraversability::CLASS_OBSTACLE;
                trav->setProbability(1.0, x, y);
            }
        }
    }
}

void Test::drawRectangle(envire::TraversabilityGrid* trav, int lowerleft_x, int lowerleft_y, 
        unsigned int width, unsigned int length, int cost_class) {
       
    envire::TraversabilityGrid::ArrayType& trav_array = trav->getGridData();    
    int start_x = lowerleft_x;
    int end_x = lowerleft_x + width;
    int start_y = lowerleft_y;
    int end_y = lowerleft_y + length;
     
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
            trav_array[y][x] = cost_class;
            trav->setProbability(1.0, x, y);
        }
    }
}

double Test::dist(int x1, int y1, int x2, int y2) {
    double dist = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
    return dist;
}

double Test::createFootprint(double fp_min_m, double fp_max_m, double& fp_radius) {
    fp_radius = (rand() % ((int)((fp_max_m - fp_min_m) * 1000))) / 1000.0  + fp_min_m;
}

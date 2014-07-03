/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef MOTION_PLANNING_LIBRARIES_TEST_TASK_HPP
#define MOTION_PLANNING_LIBRARIES_TEST_TASK_HPP

#include "motion_planning_libraries/TestBase.hpp"

#include <motion_planning_libraries/Helpers.hpp>

namespace envire {
    class Environment;
    class FrameNode;
    class TraversabilityGrid;
}

namespace motion_planning_libraries {
    
    /**
     * Used to create a shared-ptr from a reference.
     */
    struct NullDeleter
    {
        void operator()(void const *) const {}
    };

    /*! \class Test 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','motion_planning_libraries::Test')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Test : public TestBase
    {
	friend class TestBase;
    protected:

        envire::Environment* mpEnv;
        envire::FrameNode* mpFrameNode;
        envire::TraversabilityGrid* mpTravGrid;
        boost::shared_ptr<TravData> mpTravData;
        base::samples::RigidBodyState mRBSTravGrid;
        GridCalculations mGridCalculations;

    public:
        /** TaskContext constructor for Test
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Test(std::string const& name = "motion_planning_libraries::Test");

        /** TaskContext constructor for Test 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Test(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Test
         */
        ~Test();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
        
    private:        
        void createTraversabilityMap();
        
        void createStartGoalPose(int width, int height,
                base::samples::RigidBodyState& start, base::samples::RigidBodyState& goal);
        
        void drawCircle(envire::TraversabilityGrid* trav, unsigned int center_x, 
                unsigned int center_y, int radius, int cost_class);
        
        // theta: [0,360), required to pass random angles
        base::samples::RigidBodyState createPose(int width, int height, 
                int x, int y, unsigned int theta_degree);
              
        /** 
         *         |    ##
         * height/ |    l#
         * length  |  
         *    y    ------->
         *            width x
         * l = lower_left
         */      
        void drawRectangle(envire::TraversabilityGrid* trav, int lowerleft_x, int lowerleft_y, 
                unsigned int width, unsigned int length, int cost_class);
        
        double dist(int x1, int y1, int x2, int y2);
    };
}

#endif


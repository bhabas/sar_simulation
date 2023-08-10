#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>

#include <ros/ros.h>
#include <thread>

namespace gazebo {

    class World_Update_Plugin: public WorldPlugin
    {
        public:
            
        protected:
            void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
            void Update_Gravity();
            void Update_SimSpeed();

        private:

            // GAZEBO POINTERS
            physics::WorldPtr World_Ptr;

            bool Gravity_Flag = true;

            // UPDATE SIMULATION SPEED
            double Sim_Speed;
            double Step_Size = 0.001;


            // ROS SERVICES
            ros::NodeHandle nh;

    };

}

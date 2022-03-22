#include <ros/ros.h>

// IMPORT NEAT-MATRIX LIBRARY
#include "nml.h" 

// MESSAGE IMPORTS
#include "example_msgs/CustomMessage.h"
#include "rosgraph_msgs/Clock.h"


class MyClass // DEFINE CLASS
{
    public: // CLASS VARIABLES AND FUNCTIONS THAT ARE CALLABLE OUTSIDE INTERNAL CLASS FUNCTIONS



        MyClass(ros::NodeHandle *nh) // CLASS CONSTRUCTOR (SIMILAR TO PYTHON'S __INIT__())
        {

            pub = nh->advertise<example_msgs::CustomMessage>("/MyPub_cpp",1);
            sub = nh->subscribe("/clock",1,&MyClass::clock_Callback,this,ros::TransportHints().tcpNoDelay());

        }




        // DECLARE FUNCTION PROTOTYPES
        void clock_Callback(const rosgraph_msgs::Clock::ConstPtr &msg);




    private: // CLASS VARIABLES ONLY THAT ARE CALLABLE INSIDE INTERNAL CLASS FUNCTIONS



        // DECLARE PUBLISHERS AND SUBSCRIBERS
        ros::Publisher pub;
        ros::Subscriber sub;

};

// SUBSCRIBER CALLBACK FUNCTIONS (SIMILAR TO SELF.CLOCK_CALLBACK)
void MyClass::clock_Callback(const rosgraph_msgs::Clock::ConstPtr &msg)
{

    // NEAT MATRIX LIBRARY BASICS
    printf("\nCreating a 2*I matrix\n");
    nml_mat* m1 = nml_mat_smult(nml_mat_eye(2),2);
    nml_mat_print(m1);

    printf("\nCreating a 3x2 matrix from pre-defined array\n");
    double array[6] = { 
        1.0, 0.2, 3.0, 4.0, 5.0, 3.1 
    };

    nml_mat* m2 = nml_mat_from(3, 2, 6, array);
    nml_mat_print(m2);

    printf("\nMultiplying previous matrices\n");
    nml_mat* m3 = nml_mat_dot(m2,m1);
    nml_mat_print(m3);

    printf("\nm3(0,0) = %.3f\n",m3->data[0][0]);

    // FREE ALLOCATED MEMORY SPACE ON HEAP (MAKE SURE YOU FREE ANY NEW MATRICES YOU MAKE!)
    nml_mat_free(m1);
    nml_mat_free(m2);
    nml_mat_free(m3);



    // BASIC CUSTOM PUBLISHER
    example_msgs::CustomMessage new_msg;
    new_msg.custom_msg = "Hello c++";
    new_msg.custom_vector.x = 5;

    pub.publish(new_msg);


}
#include <Example_Cam_Plugin.h>


namespace gazebo
{
    void Example_Cam_Plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Example_Cam_Plugin_Plugin\n";
        printf("\n\n");
    }

    void Example_Cam_Plugin::CamData_Callback(const sensor_msgs::Image::ConstPtr &Camera_msg)
    {
        // std::cout << "CamData_Callback" << std::endl;

        //CREATE VECTOR TO STORE DATA FROM ROS MSG
        std::vector<uint8_t> Cam_Vec = Camera_msg->data;
        unsigned int Cam_Vec_Size = Cam_Vec.size();


        // INVERT IMAGE DATA
        for(unsigned int i = 0; i < Cam_Vec_Size; i++)
        {
            Cam_Vec[i] = 255-Cam_Vec[i];
        }

        // REFERENCE CONTENTS FROM INITIAL CAMERA MSG
        inverted_image_msg = *Camera_msg;

        // UPDATE DATA WITH INVERTED IMAGE
        inverted_image_msg.data = Cam_Vec;

        // PUBLISH INVERTED IMAGE ON ROS TOPIC
        Cam_Data_Pub.publish(inverted_image_msg);
    }

    GZ_REGISTER_MODEL_PLUGIN(Example_Cam_Plugin);
}
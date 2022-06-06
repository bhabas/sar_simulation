#include <ros/ros.h>

// IMPORT NEAT-MATRIX LIBRARY
#include "nml.h" 

// MESSAGE IMPORTS
#include "example_msgs/CustomMessage.h"
#include "rosgraph_msgs/Clock.h"
#include "sensor_msgs/Image.h"

// STD IMPORTS
#include <cmath>
#include <stdio.h>

#define WIDTH_PIXELS 160 //Picture dimensions
#define HEIGHT_PIXELS 160
float w = 3.6e-6; //Pixel width in meters
float f = 0.66e-3/2 ;//Focal length in meters
float O_up = WIDTH_PIXELS/2; //Pixel x,y offsets
float V_up = HEIGHT_PIXELS/2;
float U; //defining image coords
float V;

//Init outputs
int16_t It[HEIGHT_PIXELS*WIDTH_PIXELS] = {0};
double Gtemp = 0;
double Iuu = 0;
double Ivv = 0;
double Iuv = 0;
double IGu = 0;
double IGv = 0;
double IGG = 0;
double Iut = 0;
double Ivt = 0;
double IGt = 0;
double dt = 1;
double Prev_time = 1; //init as 1 to prevent divide by zero for first image

float Tau = 0;
float OFx = 0;
float OFy = 0;

// Init Sub-kernels
int kx0[3] = {-1, 0, 1};
int kx1[3] = {-2, 0, 2};
int kx2[3] = {-1, 0, 1};
int ky0[3] = {-1,-2,-1};
int ky2[3] = {1,2,1};

//Creting ROS Time object
ros::Time Cur_Time; 


class MyClass // DEFINE CLASS
{
    public: // CLASS VARIABLES AND FUNCTIONS THAT ARE CALLABLE OUTSIDE INTERNAL CLASS FUNCTIONS


        MyClass(ros::NodeHandle *nh) // CLASS CONSTRUCTOR (SIMILAR TO PYTHON'S __INIT__())
        {

            pub = nh->advertise<example_msgs::CustomMessage>("/MyPub_cpp",1);
            Camera_sub = nh->subscribe("/CF_Internal/camera/image_raw",1,&MyClass::Camera_Callback,this);

            // DYNAMICALLY ALLOCATE MEMORY TO STORE PREVIOUS IMAGE
            Prev_img = (uint8_t*)calloc(WIDTH_PIXELS*HEIGHT_PIXELS,sizeof(uint8_t));

        }


        // DECLARE FUNCTION PROTOTYPES
        void Camera_Callback(const sensor_msgs::Image::ConstPtr &Camera_msg);
        void Convolution_X_Y(const unsigned char* input, int ImgX,int ImgY);
        void Convolution_T(const unsigned char* CurImg,unsigned char* PrevImg);


        const uint8_t* Cur_img = NULL;  // Ptr to current image memory
        uint8_t* Prev_img = NULL;       // Ptr to prev image memory


    private: // CLASS VARIABLES ONLY THAT ARE CALLABLE INSIDE INTERNAL CLASS FUNCTIONS


        // DECLARE PUBLISHERS AND SUBSCRIBERS
        ros::Publisher pub;
        ros::Subscriber Camera_sub;

};

void MyClass::Camera_Callback(const sensor_msgs::Image::ConstPtr &Camera_msg){

    //CREATE VECTOR TO STORE DATA FROM ROS MSG
    std::vector<uint8_t> Cam_Vec = Camera_msg->data;
    std::vector<uint8_t> Cam_Vec_prev;
    Cam_Vec_prev.assign(Prev_img,Prev_img+WIDTH_PIXELS*HEIGHT_PIXELS);

    // IMAGE PROCESSING WORK GOES HERE
    Cur_img = &(Camera_msg->data)[0]; // Point to current image data address

    Cur_Time = Camera_msg->header.stamp;
    


    // Calling Convolutions
    MyClass::Convolution_T(Cur_img,Prev_img);
    MyClass::Convolution_X_Y(Cur_img,WIDTH_PIXELS,HEIGHT_PIXELS);

    example_msgs::CustomMessage new_msg;
    new_msg.Camera_data = Cam_Vec;
    new_msg.Prev_img = Cam_Vec_prev;
    new_msg.Tau = Tau;
    new_msg.OFx = OFx;
    new_msg.OFy = OFy;
    new_msg.header.stamp = Cur_Time;

    pub.publish(new_msg);

    memcpy(Prev_img,Cur_img,WIDTH_PIXELS*HEIGHT_PIXELS*sizeof(uint8_t)); // Copy memory to Prev_img address


} // ============ END OF MAIN LOOP ============ //

void MyClass::Convolution_T(const unsigned char* CurImg,unsigned char* PrevImg) //Still not sure where to do this in the most efficient way
{

    for(int i = 0; i < WIDTH_PIXELS*HEIGHT_PIXELS; i++){
        It[i] = CurImg[i] - PrevImg[i]; 
    }

}

void MyClass::Convolution_X_Y(const unsigned char* input, int ImgX, int ImgY) 
{

    Gtemp = 0;
    Iuu = 0;
    Ivv = 0;
    Iuv = 0;
    IGu = 0;
    IGv = 0;
    IGG = 0;
    Iut = 0;
    Ivt = 0;
    IGt = 0;

    //Where the convolution starts
    int X = 1;
    int Y = 1;

    // float Cur_time = ros::Time::now().toSec();
    
    for(int j = 0; j < (ImgX - 2)*(ImgY - 2); j++) // How many times the kernel center moves around the image
    {

        //GENERALIZE FOR CHANGE IN KERNEL SIZE
        if(X >= ImgX - 1) //if the edge of the kernel hits the edge of the image
        { 
        
            X = 1; //move the kernel back to the left edge of the image
            Y++; //and slide the kernel down the image

        }

        //Sub Kernel Indexing 
        uint32_t i0 = (X - 1) + (Y - 1) * ImgX; //First grab top left location of whole kernel
        uint32_t i1 = i0 + ImgX; //then each following row is separated by the image width
        uint32_t i2 = i1 + ImgX;

        U = (X - O_up)*w + (w/2); // Using current location of the Kernel center
        V = (Y - V_up)*w + (w/2); //calculate the current pixel grid locations (u,v)


        int32_t Xsum = 0; //reset rolling sum to 0
        int32_t Ysum = 0;

        //GENERALIZE FOR CHANGE IN KERNEL SIZE
        for(int k = 0; k < 3; k++){

            //Sub kernel 0
            Xsum += kx0[k] * input[i0 + k];
            Ysum += ky0[k] * input[i0 + k];

            //Sub kernel 1 (skipping ky1)
            Xsum += kx1[k] * input[i1 + k];

            //Sub kernel 2
            Xsum += kx2[k] * input[i2 + k];
            Ysum += ky2[k] * input[i2 + k];

        }

        //Sum assigned to middle value
        int16_t Ittemp = It[i1 + 1];
        Gtemp = (Xsum*U + Ysum*V);

        //LHS Matrix values (rolling sums)
        Iuu += Xsum*Xsum;
        Ivv += Ysum*Ysum;
        Iuv += Xsum*Ysum;
        IGu += Gtemp*Xsum;
        IGv += Gtemp*Ysum;
        IGG += Gtemp*Gtemp;

        //RHS Matrix Values (rolling sums)
        Iut += Xsum*Ittemp;
        Ivt += Ysum*Ittemp; 
        IGt += Gtemp*Ittemp;

        X++; // move center of kernel over
        
    } // END OF CONVOLUTION

    dt = Cur_Time.toSec() - Prev_time;
    printf("\ndt: %f\n",dt);


    // Packing final result into the matrices and applying the floating point math
    double LHS[9] = {f/powf(8*w,2)*Iuu, f/powf(8*w,2)*Iuv, 1/powf(8*w,2)*IGu,
                     f/powf(8*w,2)*Iuv, f/powf(8*w,2)*Ivv, 1/powf(8*w,2)*IGv,
                     f/powf(8*w,2)*IGu, f/powf(8*w,2)*IGv, 1/powf(8*w,2)*IGG};

    double RHS[3] = {-Iut/(8*w*dt), -Ivt/(8*w*dt), -IGt/(8*w*dt)};

    nml_mat* m_A = nml_mat_from(3,3,9,LHS);
    nml_mat* m_b = nml_mat_from(3,1,3,RHS);
    nml_mat_print(m_A);
    nml_mat_print(m_b);

    nml_mat_qr *QR = nml_mat_qr_solve(m_A); // A = Q*R
    nml_mat* y = nml_mat_dot(nml_mat_transp(QR->Q),m_b); // y = Q^T*b
    nml_mat* x_QR = nml_ls_solvebck(QR->R,y); // Solve R*x = y via back substitution
    nml_mat_print(x_QR);

    OFy = x_QR->data[0][0];
    OFx = x_QR->data[1][0];
    Tau = 1/(x_QR->data[2][0]);

    std::cout <<"\nTTC: " << Tau << "\n";
    std::cout <<"\nOFy: " << OFy << "\n";
    std::cout <<"\nOFx: " << OFx << "\n";

    nml_mat_free(m_A);
    nml_mat_free(m_b);
    nml_mat_qr_free(QR);
    nml_mat_free(x_QR);

    Prev_time = Cur_Time.toSec(); //reset Prev_time to Current iteration time


} // ========= END OF CONVOLUTION X Y =========

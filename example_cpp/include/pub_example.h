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
float f = 0.33e-3;//Focal length in meters
float O_up = WIDTH_PIXELS/2; //Pixel x,y offsets
float V_up = HEIGHT_PIXELS/2;
float U; //defining image coords
float V;

//Init outputs
int Ix[HEIGHT_PIXELS*WIDTH_PIXELS] = {0}; //zero init Ix Iy
int Iy[HEIGHT_PIXELS*WIDTH_PIXELS] = {0};
int It[HEIGHT_PIXELS*WIDTH_PIXELS] = {0};
float Gtemp = 0;
float Iuu = 0;
float Ivv = 0;
float Iuv = 0;
float IGu = 0;
float IGv = 0;
float IGG = 0;
float Iut = 0;
float Ivt = 0;
float IGt = 0;

// Init Sub-kernels
int kx0[3] = {-1, 0, 1};
int kx1[3] = {-2, 0, 2};
int kx2[3] = {-1, 0, 1};
int ky0[3] = {-1,-2,-1};
int ky2[3] = {1,2,1};


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
        void Convolution_T(const unsigned char* CurImg,const unsigned char* PrevImg);

        // IMAGE PROCESSING VARIABLES
        // uint8_t WIDTH_PIXELS = 160;
        // uint8_t HEIGHT_PIXELS = 160;

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
    // std::vector<uint8_t> Cam_Vec_prev(Prev_img, Prev_img + sizeof(Camera_msg->data)/sizeof(Prev_img[0]));
    std::vector<uint8_t> Cam_Vec_prev;
    Cam_Vec_prev.assign(Prev_img,Prev_img+WIDTH_PIXELS*HEIGHT_PIXELS);

    // IMAGE PROCESSING WORK GOES HERE
    Cur_img = &(Camera_msg->data)[0]; // Point to current image data address

    // Calling Convolutions
    MyClass::Convolution_T(Cur_img,Prev_img);
    MyClass::Convolution_X_Y(Cur_img,WIDTH_PIXELS,HEIGHT_PIXELS);

    // std::vector<uint8_t> Prev_img_Vec;
    // std::copy(std::begin(Prev_img),std::end(Prev_img),std::back_inserter(Prev_img_Vec));

    // sensorData.Tau = 0.0f;
    // sensorData.OFx = 0.0f;
    // sensorData.OFy = 0.0f;

    printf("Prev_Val: %u\n",Prev_img[0]);
    printf("Cur_Val: %u\n",Cur_img[0]);
    printf("\n");

    memcpy(Prev_img,Cur_img,sizeof(Camera_msg->data)); // Copy memory to Prev_img address

    // Copying and Writing to Output Message
    // std::vector<int64_t> ConvX_Vec;
    // std::copy(st// std::vector<uint8_t> Prev_img_Vec;
    // std::copy(std::begin(Prev_img),std::end(Prev_img),std::back_inserter(Prev_img_Vec));

    example_msgs::CustomMessage new_msg;
    new_msg.Camera_data = Cam_Vec;
    // new_msg.Prev_img = Prev_img_Vec;

    pub.publish(new_msg);


} // ============ END OF MAIN LOOP ============ //

void MyClass::Convolution_T(const unsigned char* CurImg,const unsigned char* PrevImg) //Still not sure where to do this in the most efficient way
{

    for(int i = 0; i < WIDTH_PIXELS*HEIGHT_PIXELS; i++){
        It[i] = CurImg[i] - PrevImg[i]; //assumes dt = 1
        //std::cout << (It[i]) << "\n";
    }

}

void MyClass::Convolution_X_Y(const unsigned char* input, int ImgX, int ImgY) 
{

    //Where the convolution starts
    int X = 1;
    int Y = 1;
    
    for(int j = 0; j < (ImgX - 2)*(ImgY - 2); j++) // How many times the kernel center moves around the image
    {

        //GENERALIZE FOR CHANGE IN KERNEL SIZE
        if(X >= ImgX - 1) //if the edge of the kernel hits the edge of the image
        { 
        
            X = 1; //move the kernel back to the left edge of the image
            Y++; //and slide the kernel down the image

        }

        //Sub Kernel Indexing 
        int i0 = (X - 1) + (Y - 1) * ImgX; //First grab top left location of whole kernel
        int i1 = i0 + ImgX; //then each following row is separated by the image width
        int i2 = i1 + ImgX;

        U = (X - O_up)*w + (w/2); // Using current location of the Kernel center
        V = (Y - V_up)*w + (w/2); //calculate the current pixel grid locations (u,v)

        // ######  DEBUGGING  ######
        /*//
        std::cout << "i0: " << i0 << "\n";
        std::cout << "i1: " << i1 << "\n";
        std::cout << "i2: " << i2 << "\n";
        *///

        int Xsum = 0; //reset rolling sum to 0
        int Ysum = 0;

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
        int Ittemp = It[i1 + 1];
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

    // Packing final result into the matrices and applying the floating point math
    double LHS[9] = {f/powf(8*w,2)*Iuu, f/powf(8*w,2)*Iuv, 1/powf(8*w,2)*IGu,
                     f/powf(8*w,2)*Iuv, f/powf(8*w,2)*Ivv, 1/powf(8*w,2)*IGv,
                     f/powf(8*w,2)*IGu, f/powf(8*w,2)*IGv, 1/powf(8*w,2)*IGG};

    double RHS[3] = {-Iut/(8*w), -Ivt/(8*w), -IGt/(8*w)};

    nml_mat* m_A = nml_mat_from(3,3,9,LHS);
    nml_mat* m_b = nml_mat_from(3,1,3,RHS);

    nml_mat_qr *QR = nml_mat_qr_solve(m_A); // A = Q*R
    nml_mat* y = nml_mat_dot(nml_mat_transp(QR->Q),m_b); // y = Q^T*b
    nml_mat* x_QR = nml_ls_solvebck(QR->R,y); // Solve R*x = y via back substitution
    nml_mat_print(x_QR);

    nml_mat_free(m_A);
    nml_mat_free(m_b);
    nml_mat_qr_free(QR);
    nml_mat_free(x_QR);


} // ========= END OF CONVOLUTION X Y =========

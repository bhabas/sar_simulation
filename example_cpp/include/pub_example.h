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

#define Pixel_width 160 //Picture dimensions
#define Pixel_height 120
float w = 3.6e-6; //Pixel width in meters
float f = 0.33e-3;//Focal length in meters
float O_up = Pixel_width/2; //Pixel x,y offsets
float O_vp = Pixel_height/2;
float U_grid[Pixel_width]; //defining image sensor arrays
float V_grid[Pixel_width];
int Prev_img[Pixel_width * Pixel_height];
float Ky[3] = {1,2,1}; 
float Kx[3] = {1,0,-1};
float kernelX[9] = {1,0,-1,2,0,-2,1,0,-1};
float kernelY[9] = {-1,-2,-1,0,0,0,1,2,1};

unsigned char *input;
unsigned char *Output;
unsigned char *Output2;

class MyClass // DEFINE CLASS
{
    public: // CLASS VARIABLES AND FUNCTIONS THAT ARE CALLABLE OUTSIDE INTERNAL CLASS FUNCTIONS


        MyClass(ros::NodeHandle *nh) // CLASS CONSTRUCTOR (SIMILAR TO PYTHON'S __INIT__())
        {

            pub = nh->advertise<example_msgs::CustomMessage>("/MyPub_cpp",1);
            //sub = nh->subscribe("/clock",1,&MyClass::clock_Callback,this,ros::TransportHints().tcpNoDelay());
            Camera_sub = nh->subscribe("/CF_Internal/camera/image_raw",1,&MyClass::Camera_Callback,this);

        }


        // DECLARE FUNCTION PROTOTYPES
        //void clock_Callback(const rosgraph_msgs::Clock::ConstPtr &msg);
        void Camera_Callback(const sensor_msgs::Image::ConstPtr &Camera_msg);
        bool convolve2DSeparable(unsigned char* in, unsigned char* out, int dataSizeX, int dataSizeY, float* kernelX, int kSizeX, float* kernelY, int kSizeY);
        bool convolve2DSlow(unsigned char* in, unsigned char* out, int dataSizeX, int dataSizeY, float* kernel, int kernelSizeX, int kernelSizeY);


    private: // CLASS VARIABLES ONLY THAT ARE CALLABLE INSIDE INTERNAL CLASS FUNCTIONS


        // DECLARE PUBLISHERS AND SUBSCRIBERS
        ros::Publisher pub;
        //ros::Subscriber sub;
        ros::Subscriber Camera_sub;

};

void MyClass::Camera_Callback(const sensor_msgs::Image::ConstPtr &Camera_msg){

    //CREATE VECTOR TO STORE DATA FROM ROS MSG
    std::vector<uint8_t> Cam_Vec = Camera_msg->data;
    Cam_Vec.erase(Cam_Vec.begin() + Pixel_width*Pixel_height, Cam_Vec.end());

    //CREATE POINTER(ARRAY) TO DATA LOCATION OF FIRST INDEX
    uint8_t* Cur_img = &Cam_Vec[0];

    std::cout << "\n\nCamera Array\n";

    //iterate through the whole array and print the current image
    //pre-init vars
    uint i = 0; //i has to be initialized as 1 to prevent divide by zero
    uint x = 0;
    uint y = 0;
    unsigned char X_Output[Pixel_width*Pixel_height];
    unsigned char Y_Output[Pixel_width*Pixel_height];
    uint j = 0;

    //MyClass::convolve2DSeparable(Cur_img,X_Output,Pixel_width,Pixel_height,Kx,3,Ky,3);
    //MyClass::convolve2DSeparable(Cur_img,Y_Output,Pixel_width,Pixel_height,Ky,3,Kx,3);
    MyClass::convolve2DSlow(Cur_img,X_Output,Pixel_width,Pixel_height,kernelX,3,3);
    MyClass::convolve2DSlow(Cur_img,Y_Output,Pixel_width,Pixel_height,kernelY,3,3);

    std::vector<uint8_t> ConvX;
    std::copy(std::begin(X_Output),std::end(X_Output),std::back_inserter(ConvX));

    std::vector<uint8_t> ConvY;
    std::copy(std::begin(Y_Output),std::end(Y_Output),std::back_inserter(ConvY));

    // int Img_data[Pixel_width*Pixel_height];

    // for(int i = 0; i < Pixel_width*Pixel_height;i++){
    //     Img_data[i] = (int)Cur_img[i];
    // }

    example_msgs::CustomMessage new_msg;
    //new_msg.custom_vector.x = 5;
    new_msg.Camera_data = Cam_Vec;
    new_msg.Xconv = ConvX;
    new_msg.Yconv = ConvY;

    pub.publish(new_msg);


    printf("\nTest Output\n");


/*  //=========================================================

    for(y = 0; y < 5;){
        
        for(x = 0; x < 5;){

            i = x + Pixel_width * y;
            printf("%u,",Cur_img[i]);
            x++;

        }
        y++;
        printf("\n");
    }
    

   for(uint n = 0; n < Pixel_width*4; n++){

        if(i == Pixel_width - 1) {
            printf("%u\n\n",Cur_img[n]);
            i = 0;
        }

        else {
            printf("%u,",Cur_img[n]);
        }

       i++;

    }

    printf("\n");

    for(y = 0; y < 6;){
        
        for(x = 0; x < 6;){

            i = x + Pixel_width * y;
            printf("%u,",Output[i]);
            x++;

        }
        y++;
        printf("\n");
    }

    printf("\n");

    for(y = 0; y < 6;){
        
        for(x = 0; x < 6;){

            i = x + Pixel_width * y;
            printf("%u,",Output2[i]);
            x++;

        }
        y++;
        printf("\n");
    }



// ===================================================*/



    // for(uint n = 0; n < Pixel_height * Pixel_width; n++){

    //     if(i % 120 == 0) printf("%u\n",Cur_img[n]);

    //     else printf("%u,",Cur_img[n]);

    //     i++;
        
    // }

    // printf("\nOutput of Separable Convolution\n");
    // i = 1;

    // for(uint n = 0; n < Pixel_height * Pixel_width; n++){

    //     if(i %120 == 0) printf("%u\n",Output[n]);

    //     else printf("%u,", Output[n]);
    //     i++;

    // }

}

// SUBSCRIBER CALLBACK FUNCTIONS (SIMILAR TO SELF.CLOCK_CALLBACK)
// void MyClass::clock_Callback(const rosgraph_msgs::Clock::ConstPtr &msg)
// {

//     // NEAT MATRIX LIBRARY BASICS
//     printf("\nCreating a 2*I matrix\n");
//     nml_mat* m1 = nml_mat_smult(nml_mat_eye(2),2);
//     nml_mat_print(m1);

//     printf("\nCreating a 3x2 matrix from pre-defined array\n");
//     double array[6] = { 
//         1.0, 0.2, 3.0, 4.0, 5.0, 3.1 
//     };

//     nml_mat* m2 = nml_mat_from(3, 2, 6, array);
//     nml_mat_print(m2);

//     printf("\nMultiplying previous matrices\n");
//     nml_mat* m3 = nml_mat_dot(m2,m1);
//     nml_mat_print(m3);

//     printf("\nm3(0,0) = %.3f\n",m3->data[0][0]);

//     // FREE ALLOCATED MEMORY SPACE ON HEAP (MAKE SURE YOU FREE ANY NEW MATRICES YOU MAKE!)
//     nml_mat_free(m1);
//     nml_mat_free(m2);
//     nml_mat_free(m3);



//     // BASIC CUSTOM PUBLISHER
//     example_msgs::CustomMessage new_msg;
//     new_msg.custom_msg = "Hello c++";
//     new_msg.custom_vector.x = 5;

//     pub.publish(new_msg);


// }

bool MyClass::convolve2DSeparable(unsigned char* in, unsigned char* out, int dataSizeX, int dataSizeY, float* kernelX,
    int kSizeX, float* kernelY, int kSizeY)
{
    int i, j, k, m, n;
    float *tmp, *sum;                // intermediate data buffer
    unsigned char *inPtr, *outPtr;   // working pointers
    float *tmpPtr, *tmpPtr2;         // working pointers
    int kCenter, kOffset, endIndex;  // kernel indice

    // check validity of params
    if (!in || !out || !kernelX || !kernelY)
        return false;
    if (dataSizeX <= 0 || kSizeX <= 0)
        return false;

    // allocate temp storage to keep intermediate result
    tmp = new float[dataSizeX * dataSizeY];
    if (!tmp)
        return false;  // memory allocation error

    // store accumulated sum
    sum = new float[dataSizeX];
    if (!sum)
        return false;  // memory allocation error

    // covolve horizontal direction ///////////////////////

    // find center position of kernel (half of kernel size)
    kCenter = kSizeX >> 1;           // center index of kernel array
    endIndex = dataSizeX - kCenter;  // index for full kernel convolution

    // init working pointers
    inPtr = in;
    tmpPtr = tmp;  // store intermediate results from 1D horizontal convolution

    // start horizontal convolution (x-direction)
    for (i = 0; i < dataSizeY; ++i)  // number of rows
    {
        kOffset = 0;  // starting index of partial kernel varies for each sample

        // COLUMN FROM index=0 TO index=kCenter-1
        for (j = 0; j < kCenter; ++j)
        {
            *tmpPtr = 0;  // init to 0 before accumulation

            for (k = kCenter + kOffset, m = 0; k >= 0; --k, ++m)  // convolve with partial of kernel
            {
                *tmpPtr += *(inPtr + m) * kernelX[k];
            }
            ++tmpPtr;   // next output
            ++kOffset;  // increase starting index of kernel
        }

        // COLUMN FROM index=kCenter TO index=(dataSizeX-kCenter-1)
        for (j = kCenter; j < endIndex; ++j)
        {
            *tmpPtr = 0;  // init to 0 before accumulate

            for (k = kSizeX - 1, m = 0; k >= 0; --k, ++m)  // full kernel
            {
                *tmpPtr += *(inPtr + m) * kernelX[k];
            }
            ++inPtr;   // next input
            ++tmpPtr;  // next output
        }

        kOffset = 1;  // ending index of partial kernel varies for each sample

        // COLUMN FROM index=(dataSizeX-kCenter) TO index=(dataSizeX-1)
        for (j = endIndex; j < dataSizeX; ++j)
        {
            *tmpPtr = 0;  // init to 0 before accumulation

            for (k = kSizeX - 1, m = 0; k >= kOffset; --k, ++m)  // convolve with partial of kernel
            {
                *tmpPtr += *(inPtr + m) * kernelX[k];
            }
            ++inPtr;    // next input
            ++tmpPtr;   // next output
            ++kOffset;  // increase ending index of partial kernel
        }

        inPtr += kCenter;  // next row
    }
    // END OF HORIZONTAL CONVOLUTION //////////////////////

    // start vertical direction ///////////////////////////

    // find center position of kernel (half of kernel size)
    kCenter = kSizeY >> 1;           // center index of vertical kernel
    endIndex = dataSizeY - kCenter;  // index where full kernel convolution should stop

    // set working pointers
    tmpPtr = tmpPtr2 = tmp;
    outPtr = out;

    // clear out array before accumulation
    for (i = 0; i < dataSizeX; ++i)
        sum[i] = 0;

    // start to convolve vertical direction (y-direction)

    // ROW FROM index=0 TO index=(kCenter-1)
    kOffset = 0;  // starting index of partial kernel varies for each sample
    for (i = 0; i < kCenter; ++i)
    {
        for (k = kCenter + kOffset; k >= 0; --k)  // convolve with partial kernel
        {
            for (j = 0; j < dataSizeX; ++j)
            {
                sum[j] += *tmpPtr * kernelY[k];
                ++tmpPtr;
            }
        }

        for (n = 0; n < dataSizeX; ++n)  // convert and copy from sum to out
        {
            // covert negative to positive
            *outPtr = (unsigned char)((float)fabs(sum[n]) + 0.5f);
            sum[n] = 0;  // reset to zero for next summing
            ++outPtr;    // next element of output
        }

        tmpPtr = tmpPtr2;  // reset input pointer
        ++kOffset;         // increase starting index of kernel
    }

    // ROW FROM index=kCenter TO index=(dataSizeY-kCenter-1)
    for (i = kCenter; i < endIndex; ++i)
    {
        for (k = kSizeY - 1; k >= 0; --k)  // convolve with full kernel
        {
            for (j = 0; j < dataSizeX; ++j)
            {
                sum[j] += *tmpPtr * kernelY[k];
                ++tmpPtr;
            }
        }

        for (n = 0; n < dataSizeX; ++n)  // convert and copy from sum to out
        {
            // covert negative to positive
            *outPtr = (unsigned char)((float)fabs(sum[n]) + 0.5f);
            sum[n] = 0;  // reset for next summing
            ++outPtr;    // next output
        }

        // move to next row
        tmpPtr2 += dataSizeX;
        tmpPtr = tmpPtr2;
    }

    // ROW FROM index=(dataSizeY-kCenter) TO index=(dataSizeY-1)
    kOffset = 1;  // ending index of partial kernel varies for each sample
    for (i = endIndex; i < dataSizeY; ++i)
    {
        for (k = kSizeY - 1; k >= kOffset; --k)  // convolve with partial kernel
        {
            for (j = 0; j < dataSizeX; ++j)
            {
                sum[j] += *tmpPtr * kernelY[k];
                ++tmpPtr;
            }
        }

        for (n = 0; n < dataSizeX; ++n)  // convert and copy from sum to out
        {
            // covert negative to positive
            *outPtr = (unsigned char)((float)fabs(sum[n]) + 0.5f);
            sum[n] = 0;  // reset for next summing
            ++outPtr;    // next output
        }

        // move to next row
        tmpPtr2 += dataSizeX;
        tmpPtr = tmpPtr2;  // next input
        ++kOffset;         // increase ending index of kernel
    }
    // END OF VERTICAL CONVOLUTION ////////////////////////

    // deallocate temp buffers
    delete[] tmp;
    delete[] sum;
    return true;
}

bool MyClass::convolve2DSlow(unsigned char* in, unsigned char* out, int dataSizeX, int dataSizeY, float* kernel, int kernelSizeX,int kernelSizeY)
{
    int i, j, m, n, mm, nn;
    int kCenterX, kCenterY;  // center index of kernel
    float sum;               // temp accumulation buffer
    int rowIndex, colIndex;

    // check validity of params
    if (!in || !out || !kernel)
        return false;
    if (dataSizeX <= 0 || kernelSizeX <= 0)
        return false;

    // find center position of kernel (half of kernel size)
    kCenterX = kernelSizeX / 2;
    kCenterY = kernelSizeY / 2;

    for (i = 0; i < dataSizeY; ++i)  // rows
    {
        for (j = 0; j < dataSizeX; ++j)  // columns
        {
            sum = 0;                           // init to 0 before sum
            for (m = 0; m < kernelSizeY; ++m)  // kernel rows
            {
                mm = kernelSizeY - 1 - m;  // row index of flipped kernel

                for (n = 0; n < kernelSizeX; ++n)  // kernel columns
                {
                    nn = kernelSizeX - 1 - n;  // column index of flipped kernel

                    // index of input signal, used for checking boundary
                    rowIndex = i + (kCenterY - mm);
                    colIndex = j + (kCenterX - nn);

                    // ignore input samples which are out of bound
                    if (rowIndex >= 0 && rowIndex < dataSizeY && colIndex >= 0 && colIndex < dataSizeX)
                        sum += in[dataSizeX * rowIndex + colIndex] * kernel[kernelSizeX * mm + nn];
                }
            }
            out[dataSizeX * i + j] = (unsigned char)((float)fabs(sum) + 0.5f);
        }
    }

    return true;
}

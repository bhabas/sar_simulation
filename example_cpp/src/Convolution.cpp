#include <cmath>
#include <stdio.h>
#include <string.h>
#include <iostream>

#include "nml.h"

//Initial test image
unsigned char Cur_img[25] = {0, 0 , 0 , 0 , 0,
                             0, 0 , 10, 0 , 0,
                             0,10 , 20, 10, 0,
                             0, 0 , 10, 0 , 0,
                             0, 0 , 0 , 0 , 0};                       


unsigned char prev_img[25] = {0,0,0,0,0,
                              0,0,0,0,0,
                              0,0,10,0,0,
                              0,0,0,0,0,
                              0,0,0,0,0};                       

// Init Kernels
int kx0[3] = {-1, 0, 1};
int kx1[3] = {-2, 0, 2};
int kx2[3] = {-1, 0, 1};

int ky0[3] = {-1,-2,-1};
int ky2[3] = {1 , 2, 1};
// Init floating point constants
float Ugrid;
float Vgrid;
float w = 3.6e-6;
float f = 3.3e-4;
//float Output[9] = {0}; //init all entries to be zero

//Init outputs
int Ix[25] = {0};
int Iy[25] = {0};
int It[25] = {0};
float Iuu = 0;
float Ivv = 0;
float Iuv = 0;
float Gtemp = 0;
float IGu;
float IGv;
float IGG;
float Iut;
float Ivt;
float IGt;

void Convolution_X_Y(unsigned char* input, int ImgX, int ImgY); //init functions
void Convolution_T();


int main(){
    
   // Convolution(); //init function
    Convolution_T();
    Convolution_X_Y(Cur_img,5,5);
    return 1;

}

void Convolution_T() //Still not sure where to do this in the most efficient way
{

    for(int i = 0; i < 25; i++){
        It[i] = Cur_img[i] - prev_img[i]; //assumes dt = 1
    }

}

void Convolution_X_Y(unsigned char* input, int ImgX, int ImgY) 
{

    //Where the convolution starts
    int X = 1;
    int Y = 1;
    Ugrid = -2.826e-4; //both start in the same location
    Vgrid = -2.826e-4;
    
    
    for(int j = 0; j < (ImgX - 2)*(ImgY - 2); j++) // How many times the kernel center moves around the image
    {

        //GENERALIZE FOR CHANGE IN KERNEL SIZE
        if(X <= ImgX - 1) //if the edge of the kernel hits the edge of the image
        { 
        
            X = 1; //move the kernel back to the left edge of the image
            Y++; //and slide the kernel down the image

        }

        //Sub Kernel Idexing
        int i0 = (X - 1) + (Y - 1) * ImgX;
        int i1 = i0 + ImgX;
        int i2 = i1 + ImgX;

        Ugrid = (X - w/2)*w + w/2; // Using current location of the Kernel center
        Vgrid = (Y - w/2)*w + w/2; //calculate the current pixel grid locations (u,v)

        // ######  DEBUGGING  ######
        /*//
        std::cout << "i0: " << i0 << "\n";
        std::cout << "i1: " << i1 << "\n";
        std::cout << "i2: " << i2 << "\n";
        *///

        int Xsum = 0; //reset rolling sum to 0
        int Ysum = 0;
        float Usum = 0;
        float Vsum = 0;

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
        Usum = Xsum/(8*w); //kernel normalization and divide by pixel width
        Vsum = Ysum/(8*w);
        Ix[i1 + 1] = Usum;
        Iy[i1 + 1] = Vsum;
        int Ittemp = It[i1 + 1]; //this will crop It the same way the imageGrads are shouldn't be a problem

        Gtemp = (Usum*Ugrid + Vsum*Vgrid);

        //LHS Matrix values (rolling sums)
        Iuu += Usum*Usum;
        Ivv += Vsum*Vsum;
        Iuv += Usum*Vsum;
        IGu += Gtemp*Usum;
        IGv += Gtemp*Vsum;
        IGG += Gtemp*Gtemp;

        //RHS Matrix Values (rolling sums
        Iut += Usum*Ittemp; //MAKE SURE THIS IS RIGHT ORDER OF OPERATIONS
        Ivt += Vsum*Ittemp; 
        IGt += Gtemp*Ittemp;
        

        X++; // move top left of kernel over

        //std::cout << "loop (X , Y): " << X << "\n";

    }

    double LHS[9] = {f*Iuu, f*Iuv, IGu,
                     f*Iuv, f*Ivv, IGv,
                     f*IGu, f*IGv, IGG};

    double RHS[3] = {-Iut, -Ivt, -IGt};

    //DO MATRIX MAGIC
    

    //Create matrix pointers
    nml_mat* lhs = nml_mat_from(3,3,9, LHS);
    nml_mat* rhs = nml_mat_from(1,3,3,RHS); 



    nml_mat_free(lhs);
    nml_mat_free(rhs);

    // ======= DEBUGGING ========

    /*
    std::cout << "\n Ix \n";

    int i = 0;

    for(int j = 0; j < ImgX * ImgY; j++){

        if(i == ImgX - 1){
            printf("%d\n",Ix[j]);
            i = -1;
        }

        else{
            printf("%d,",Ix[j]);
        }

        i++;
    }

    std::cout << "\n Iy \n";
    i = 0;

    for(int j = 0; j < ImgX * ImgY; j++){

        if(i == ImgX - 1){
            printf("%d\n",Iy[j]);
            i = -1;
        }

        else{
            printf("%d,",Iy[j]);
        }

        i++;
    } */// END OF DEBUGGING 

} // ========= END OF CONVOLUTION X Y =========
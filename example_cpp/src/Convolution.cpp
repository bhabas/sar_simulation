#include <math.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <stdint.h>

#include "nml.h"

//Define Image size
#define Pixel_width 6
#define Pixel_height 6

//Initial test image
unsigned char Cur_img[Pixel_width*Pixel_height] = {0, 0, 0, 0, 0, 0,
                             0, 8, 8, 8, 8, 0,
                             0, 8, 0, 0, 8, 0,
                             0, 8, 0, 0, 8, 0,
                             0, 8, 8, 8, 8, 0,
                             0, 0, 0, 0, 0, 0};                      


unsigned char prev_img[Pixel_width*Pixel_height] = {0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0,
                              0, 0, 8, 8, 0, 0,
                              0, 0, 8, 8, 0, 0,
                              0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0}; 
                                              

// Init floating point constants
float w = 3.6e-6;
float f = 3.3e-4;
float O_up = Pixel_width/2;
float V_up = Pixel_height/2;
float U;
float V;

//Init outputs
float Ix[Pixel_width*Pixel_height] = {0};
float Iy[Pixel_width*Pixel_height] = {0};
float It[Pixel_width*Pixel_height] = {0};
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
int ky2[3] = {1 , 2, 1};

void Convolution_X_Y(unsigned char* input, int ImgX, int ImgY); //init functions
void Convolution_T();


int main(){
    
   // Convolution(); //init function
    Convolution_T();
    Convolution_X_Y(Cur_img,Pixel_width,Pixel_height);
    return 1;

}

void Convolution_T() //Still not sure where to do this in the most efficient way
{

    for(int i = 0; i < Pixel_width*Pixel_height; i++){
        It[i] = Cur_img[i] - prev_img[i]; //assumes dt = 1
        //std::cout << (It[i]) << "\n";
    }

}

void Convolution_X_Y(unsigned char* input, int ImgX, int ImgY) 
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

        //Sub Kernel Idexing
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
        std::cout << "X: " << X << "\tO_up: " << O_up << "\n";
        std::cout << "U: " << U << "\tVgrid: " << V << "\n";    
        std::cout << "\nIG: " << Gtemp << "\tIu: " << Usum << "\tIv: " << Vsum << "\n";
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
        //Usum = Xsum;//(8*w); //kernel normalization and divide by pixel width
        //Vsum = Ysum;//(8*w);
        //Ix[i1 + 1] = Usum;
        //Iy[i1 + 1] = Vsum;
        //std::cout << "Iu: " << Usum << "\tIv: " << Vsum << "\n";
        int Ittemp = It[i1 + 1]; //this will crop It the same way the imageGrads are shouldn't be a problem

        Gtemp = (Xsum*U + Ysum*V);
        std::cout << "\nIG: " << Gtemp << "\tIu: " << Usum << "\tIv: " << Vsum << "\n";

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

    double LHS[9] = {f/powf(8*w,2)*Iuu, f/powf(8*w,2)*Iuv, 1/powf(8*w,2)*IGu,
                     f/powf(8*w,2)*Iuv, f/powf(8*w,2)*Ivv, 1/powf(8*w,2)*IGv,
                     f/powf(8*w,2)*IGu, f/powf(8*w,2)*IGv, 1/powf(8*w,2)*IGG};

    double RHS[3] = {-Iut/(8*w), -Ivt/(8*w), -IGt/(8*w)};

    //DO MATRIX MAGIC
    
    // INITIALIZE MATRICES FROM DATA
    nml_mat* m_A = nml_mat_from(3,3,9, LHS);
    nml_mat* m_b = nml_mat_from(3,1,3,RHS); 

    printf("\nLHS:\n");
    nml_mat_print(m_A);
    printf("\nRHS:\n");
    nml_mat_print(m_b);

    nml_mat_qr *QR = nml_mat_qr_solve(m_A); // A = Q*R
    nml_mat* y = nml_mat_dot(nml_mat_transp(QR->Q),m_b); // y = Q^T*b
    nml_mat* x_QR = nml_ls_solvebck(QR->R,y); // Solve R*x = y via back substitution
    nml_mat_print(x_QR);

    nml_mat_free(m_A);
    nml_mat_free(m_b);
    nml_mat_qr_free(QR);
    nml_mat_free(x_QR);

} // ========= END OF CONVOLUTION X Y =========
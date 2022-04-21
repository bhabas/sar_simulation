#include <math.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <stdint.h>

#include "nml.h"

//Initial test image
unsigned char Cur_img[36] = {0, 0, 0, 0, 0, 0,
                             0, 8, 8, 8, 8, 0,
                             0, 8, 0, 0, 8, 0,
                             0, 8, 0, 0, 8, 0,
                             0, 8, 8, 8, 8, 0,
                             0, 0, 0, 0, 0, 0};                      


unsigned char prev_img[36] = {0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0,
                              0, 0, 8, 8, 0, 0,
                              0, 0, 8, 8, 0, 0,
                              0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0}; 
                                              

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
float Ix[25] = {0};
float Iy[25] = {0};
float It[25] = {0};
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
        //std::cout << (It[i]) << "\n";
    }

}

void Convolution_X_Y(unsigned char* input, int ImgX, int ImgY) 
{

    //Where the convolution starts
    int X = 1;
    int Y = 1;
    float O_up = ImgX/2;
    float V_up = ImgY/2;
    
    for(int j = 0; j < (ImgX - 2)*(ImgY - 2); j++) // How many times the kernel center moves around the image
    {

        //GENERALIZE FOR CHANGE IN KERNEL SIZE
        if(X >= ImgX - 1) //if the edge of the kernel hits the edge of the image
        { 
        
            X = 1; //move the kernel back to the left edge of the image
            Y++; //and slide the kernel down the image

        }

        //Sub Kernel Idexing
        int i0 = (X - 1) + (Y - 1) * ImgX;
        int i1 = i0 + ImgX;
        int i2 = i1 + ImgX;

        Ugrid = (X - O_up*w) + (w/2); // Using current location of the Kernel center
        Vgrid = (Y - V_up*w) + (w/2); //calculate the current pixel grid locations (u,v)

        
        // ######  DEBUGGING  ######
        /*//
        std::cout << "i0: " << i0 << "\n";
        std::cout << "i1: " << i1 << "\n";
        std::cout << "i2: " << i2 << "\n";
        std::cout << "X: " << X << "\tO_up: " << O_up << "\n";
        std::cout << "Ugrid: " << Ugrid << "\tVgrid: " << Vgrid << "\n";        
        *///

        float Xsum = 0; //reset rolling sum to 0
        float Ysum = 0;
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

        //if(X == 2 && Y == 1){
        //    std::cout << "Xsum: " << Xsum << "\tYsum: " << Ysum << "\n";
        //}

        //Sum assigned to middle value
        Usum = Xsum/(8*w); //kernel normalization and divide by pixel width
        Vsum = Ysum/(8*w);
        Ix[i1 + 1] = Usum;
        Iy[i1 + 1] = Vsum;
        //std::cout << "Iu: " << Usum << "\tIv: " << Vsum << "\n";
        int Ittemp = It[i1 + 1]; //this will crop It the same way the imageGrads are shouldn't be a problem

        Gtemp = (Usum*Ugrid + Vsum*Vgrid);
        std::cout << "\nIG: " << Gtemp << "\n";

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

    std::cout << "\nIvv: " << Ivv; 

    double LHS[9] = {f*Iuu, f*Iuv, IGu,
                    f*Iuv, f*Ivv, IGv,
                    f*IGu, f*IGv, IGG};

    double RHS[3] = {-Iut, -Ivt, -IGt};

    //DO MATRIX MAGIC

    // double LHS[9] = {1,2,3,
    //                  4,5,6,
    //                  7,8,1};
    
    // double RHS[3] = {5,3,1};
    
    // INITIALIZE MATRICES FROM DATA
    nml_mat* m_A = nml_mat_from(3,3,9, LHS);
    nml_mat* m_B = nml_mat_from(3,1,3,RHS); 

    printf("\nLHS:\n");
    nml_mat_print(m_A);
    printf("\nRHS:\n");
    nml_mat_print(m_B);


    // SOLVE FOR OPTICAL FLOW VALUES
    nml_mat_lup *LUP = nml_mat_lup_solve(m_A);
    nml_mat* x = nml_ls_solve(LUP,m_B);
    printf("\nX:\n");
    nml_mat_print(x);

    // DEALLOCATE MATRIX MEMORY
    nml_mat_free(m_A);
    nml_mat_free(m_B);
    nml_mat_lup_free(LUP);
    nml_mat_free(x);


    // // EXAMPLE SOLVE Ax = b

    // double A[9] = {
    //     1.0, 2.0, 3.0,
    //     4.0, 5.0, 6.0,
    //     7.0, 8.0, 1.0,
    // };

    // double b[3] = {5.0,3.0,1.0};
    
    // nml_mat* m_A = nml_mat_from(3,3,9,A);
    // nml_mat* m_b = nml_mat_from(3,1,3,b);
    // nml_mat_lup *LUP = nml_mat_lup_solve(m_A);

    // nml_mat *x = nml_ls_solve(LUP, m_b);
    // nml_mat_print(x);

    // nml_mat_free(m_A);
    // nml_mat_free(m_b);
    // nml_mat_lup_free(LUP);
    // nml_mat_free(x);

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
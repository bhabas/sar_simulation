#include <cmath>
#include <stdio.h>
#include <string.h>
#include <iostream>

//Initial test image
int img[9] = {10,12,19,13,20,7,6,12,18};

// Init Kernels
int kx0[3] = {-1, 0, 1};
int kx1[3] = {-2, 0, 2};
int kx2[3] = {-1, 0, 1};
int ky0[3] = {-1,-2,-1};
int ky2[3] = {1,2,1};
float Output[9] = {0}; //init all entries to be zero

int Ix[9] = {0};
int Iy[9] = {0};

bool Convolution(int in[9], int ImgX, int ImgY); //init function


int main(){
    
   // Convolution(); //init function
    Convolution(img,3,3);
    return 1;

}

bool Convolution(int in[9], int ImgX, int ImgY) 
{

    //Where the convolution starts
    int X = 1;
    int Y = 1;
    //sub Kernel locations
    int y0;
    int y1;
    int y2;

    //Rolling sum
    int Xsum = 0;
    int Ysum = 0;
    
    for(int j = 0; j < (ImgX - 2)*(ImgY - 2); j++) // How many times the kernel center moves around the image
    {

        //GENERALIZE FOR CHANGE IN KERNEL SIZE
        if(X == ImgX - 1) //if the edge of the kernel hits the edge of the image
        { 
        
            X = 1; //move the kernel back to the left edge of the image
            Y++; //and slide the kernel down the image

        }

        //sub Kernel indexing locations
        int x0 = X , x1 = X , x2 = X; 
        int y0 = Y * ImgX;
        int y2 = y0 + (ImgX * 2); //skipping the middle y kernel so move the last Ysub kernel two rows down

        //Can do this in the aboce step just doing this for readability
        int i0 = x0 - 1;
        int i1 = i0 + ImgX;
        int i2 = i1 + ImgX;

        // ######  DEBUGGING  ######
        /*
        std::cout << "i0: " << i0 << "\n";
        std::cout << "i1: " << i1 << "\n";
        std::cout << "i2: " << i2 << "\n";
        */

        //GENERALIZE FOR CHANGE IN KERNEL SIZE
        for(int k = 0; k < 3; k++){

            //Sub kernel 0
            Xsum += kx0[k] * img[i0 + k];
            Ysum += ky0[k] * img[i0 + k];

            //Sub kernel 1 (skipping ky1)
            Xsum += kx1[k] * img[i1 + k];

            //Sub kernel 2
            Xsum += kx2[k] * img[i2 + k];
            Ysum += ky2[k] * img[i2 + k];

        }

        //Sum assigned to middle value
        Ix[i1 + 1] = Xsum;
        Iy[i1 + 1] = Ysum;
        X++;

        for(int j = 0; j < 10; j++){

            printf("%d\n",Ix[j]);

        }

        std::cout << "\n Iy \n";

        for(int j = 0; j < 10; j++){

            printf("%d\n",Iy[j]);

        }

        std::cout << "loop \n";

    }

}
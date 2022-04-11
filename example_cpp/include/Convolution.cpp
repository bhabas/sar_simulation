#include <cmath>
#include <stdio.h>
#include <string.h>

//Initial test image
int img[9] = {10,12,19,13,20,7,6,12,18};

// Init Kernels
int kx0[3] = {-1, 0, 1};
int kx1[3] = {-2, 0, 2};
int kx2[3] = {-1, 0, 1};
int ky0[3] = {-1,-2,-1};
int ky2[3] = {1,2,1};
float Output[9] = {0}; //init all entries to be zero

bool Convolution(unsigned int in, float out, int ImgX, int ImgY); //init function


int main(){
    
   // Convolution(); //init function
    
    return 1;

}

bool Convolution(unsigned int in, float out, int ImgX, int ImgY,int Ksize) {

    //Where the convolution starts
    int X = 1;
    int Y = 1;
    //sub Kernel locations
    int y0;
    int y1;
    int y2;

    for(int j = 0; j < (ImgX - 1)*(ImgY - 1); j++) // How many times the kernel center moves around the image
    {

        if(X == ImgX - 1){ //if the edge of the kernel hits the edge of the image
        
            X = 1; //move the kernel back to the left edge of the image
            Y++; //and slide the kernel down the image

        }
        //sub Kernel indexing locations
        int x0 = X - 1, x1 = X - 1, x2 = X - 1; 
        int y0 = Y * ImgX;
        int y2 = y0 + (ImgX * 2); //skipping the middle y kernel so move the last Ysub kernel two rows down

        int i0 = x0 + 

    }

}
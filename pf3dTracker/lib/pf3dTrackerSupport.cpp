/**
* 
* Support Library of the 3d position tracker implementing the particle filter.
* See \ref icub_pf3dtracker \endref
*
* Copyright (C) 2009 RobotCub Consortium
*
* Author: Matteo Taiana
*
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#include <iCub/pf3dTrackerSupport.hpp>
#include <iostream>
using namespace std;

std::string itos(int i)  // convert int to string
{
    std::stringstream s;
    s << i;
    return s.str();
}

int max3(int v1, int v2, int v3)
{
    int max;
    max = (v1 > v2) ? v1 : v2;
    max = (max > v3) ? max : v3;
    return(max);
}

int min3(int v1, int v2, int v3)
{
    int min;
    min = (v1 < v2) ? v1 : v2;
    min = (min < v3) ? min : v3;
    return(min);
}

int printMatrix(float *matrix, int matrixColumns, int matrixRows)
{
    int i,j;
    
    for(j=0;j<matrixRows;j++)
    {
        for(i=0;i<matrixColumns;i++)
            std::cout<<matrix[i+j*matrixColumns]<<" ";
        std::cout<<std::endl;    
    }    
    
    return 0;
}

void rgbToYuvBin(int &R, int &G, int &B, int &YBin, int &UBin, int &VBin)
{
    //I copied the transformation from the wikipedia. WARNING ??? !!!
    float Y, U, V;
    Y=  0.299F  *(float)R +0.587F * (float)G + 0.114F *(float)B;
    U= -0.147F*float(R) -0.289F*float(G) +0.436F*float(B); //-255*436<U<255*436
    U+=0.436F*255.0F; //0<U<255*2*0.436
    U=U/(2.0F*0.436F);//0<U<255
    V= ((0.615F*float(R) -0.515F*float(G)+0.100F*float(B)+0.615F*255.0F)/(2.0F*0.615F+0.1F));    
    
    YBin=(int)Y/ 64; //I want Y to vary between 0 and 3.
    UBin=(int)U/ 32; //I want U to vary between 0 and 7.
    VBin=(int)V/ 32; //I want V to vary between 0 and 7.
    
    if(YBin<0||YBin>3)
        std::cout<<"something's wrong with Y: "<<YBin<<" "<<Y;
    if(UBin<0||UBin>7)
        std::cout<<"something's wrong with U: "<<UBin<<" "<<U;
    if(VBin<0||VBin>7)
        std::cout<<"something's wrong with V: "<<VBin<<" "<<V<<" R= "<<R<<" G= "<<G<<" B= "<<B<<"\n";
}


void rgbToYuvBinImage(cv::Mat & image,cv::Mat & transformedImage)
{
    int a1,a2,r,g,b, s,t,u;
    for(a1=0;a1<transformedImage.cols;a1++)
        for(a2=0;a2<transformedImage.rows;a2++)
    {
        r=(((uchar*)(image.data + image.step*a2))[a1*3+0]);
        g=(((uchar*)(image.data + image.step*a2))[a1*3+1]);
        b=(((uchar*)(image.data + image.step*a2))[a1*3+2]);
        rgbToYuvBin(r,g,b, s,t,u);
        (((uchar*)(transformedImage.data + transformedImage.step*a2))[a1*3+0])=s;
        (((uchar*)(transformedImage.data + transformedImage.step*a2))[a1*3+1])=t;
        (((uchar*)(transformedImage.data + transformedImage.step*a2))[a1*3+2])=u;
        //yuvBinsImage[a1][a2][0], yuvBinsImage[a1][a2][1], yuvBinsImage[a1][a2][2]);
    }
}

void setPixel(int u, int v, int r, int g, int b, cv::Mat & image)
{
    //std::cout<<"u= "<<u<<"  v= "<<v<<std::endl;
    if(u>-1&&u<image.cols && v>-1&&v<image.rows)
    {
        (((uchar*)(image.data + image.step*v))[u*3+0])=r;
        (((uchar*)(image.data + image.step*v))[u*3+1])=g;
        (((uchar*)(image.data + image.step*v))[u*3+2])=b;
    }
}      

void fillLut(Lut *lut)
{
    int r,g,b, y,u,v;
    int index;
    for(r=0;r<256;r++)
        for(g=0;g<256;g++)
            for(b=0;b<256;b++)
            {
                rgbToYuvBin(r,g,b, y,u,v);
                index=r*65536+g*256+b;
                lut[index].y=y;
                lut[index].u=u;
                lut[index].v=v;
            }
}

void rgbToYuvBinImageLut(cv::Mat & image,cv::Mat & transformedImage, Lut *lut)
{
    int a1,a2,r,g,b;
    int index;

    for(a1=0;a1<image.cols;a1++)
        for(a2=0;a2<image.rows;a2++)
    {
        r=(((uchar*)(image.data + image.step*a2))[a1*3+0]);
        g=(((uchar*)(image.data + image.step*a2))[a1*3+1]);
        b=(((uchar*)(image.data + image.step*a2))[a1*3+2]);
        index=r*65536+g*256+b;
        (((uchar*)(transformedImage.data + transformedImage.step*a2))[a1*3+0])=lut[index].y;
        (((uchar*)(transformedImage.data + transformedImage.step*a2))[a1*3+1])=lut[index].u;
        (((uchar*)(transformedImage.data + transformedImage.step*a2))[a1*3+2])=lut[index].v;
        //rgbToYuvBin(r,g,b, yuvBinsImage[a1][a2][0], yuvBinsImage[a1][a2][1], yuvBinsImage[a1][a2][2]);
    }

}

void rgbToYuvBinLut(int &R, int &G, int &B, int &YBin, int &UBin, int &VBin, Lut *lut)
{
    //I copied the transformation from the wikipedia. WARNING ??? !!!
    float Y, U, V;
    Y=  0.299F  *(float)R +0.587F * (float)G + 0.114F *(float)B;
    U= -0.147F*float(R) -0.289F*float(G) +0.436F*float(B); //-255*436<U<255*436
    U+=0.436F*255.0F; //0<U<255*2*0.436
    U=U/(2.0F*0.436F);//0<U<255
    V= ((0.615F*float(R) -0.515F*float(G)+0.100F*float(B)+0.615F*255.0F)/(2.0F*0.615F+0.1F));

    YBin=(int)Y/ 64; //I want Y to vary between 0 and 3.
    UBin=(int)U/ 32; //I want U to vary between 0 and 7.
    VBin=(int)V/ 32; //I want V to vary between 0 and 7.

    if(YBin<0||YBin>3)
        std::cout<<"something's wrong with Y: "<<YBin<<" "<<Y;
    if(UBin<0||UBin>7)
        std::cout<<"something's wrong with U: "<<UBin<<" "<<U;
    if(VBin<0||VBin>7)
        std::cout<<"something's wrong with V: "<<VBin<<" "<<V<<" R= "<<R<<" G= "<<G<<" B= "<<B<<"\n";
}


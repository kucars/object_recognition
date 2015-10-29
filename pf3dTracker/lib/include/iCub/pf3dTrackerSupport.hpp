/**
* Copyright: (C) 2009 RobotCub Consortium
* Authors: Matteo Taiana, Ugo Pattacini
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/


#ifndef _PF3DTRACKERSUPPORT_
#define _PF3DTRACKERSUPPORT_
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <time.h>

#ifdef _CH_
#pragma package <opencv>
#endif
#ifndef _EiC
//#include "cv.h"
//#include "highgui.h"
#include <opencv2/core/core.hpp>
#endif

struct Lut
{
  int y;
  int u;
  int v;
};

int printMatrix(float *matrix, int matrixColumns, int matrixRows);

void rgbToYuvBin(int &R, int &G, int &B, int &YBin, int &UBin, int &VBin);

void rgbToYuvBinImage(cv::Mat &image,cv::Mat &yuvBinsImage);

void rgbToYuvBinImageLut(cv::Mat &image,cv::Mat &yuvBinsImage, Lut *lut);

void setPixel(int u, int v, int r, int g, int b, IplImage *image);

std::string itos(int i);

int max3(int v1, int v2, int v3);

int min3(int v1, int v2, int v3);

void fillLut(Lut *lut);

#endif /* _PF3DTRACKERSUPPORT_ */

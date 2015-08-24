/*
 * ivcam_sample.cc

 *
 *  Created on: Oct 26, 2014
 *      Author: ubu
 */
#include <stdio.h>
#include <stdlib.h>
#include "ivcam.h"


using namespace ivcam_env;

static void printparams(CalibrationParameters *params);

int main(int argc, char *argv[])
{

    Ivcam ivcam;
    IVCAM_STATUS status;
    Frame frame;
    int i = 0;

    frame.depth = new uint8_t[640*480*2];
    frame.color = new uint8_t[640*480*2];

    status = ivcam.Init(640, 480, 640, 480);

    if (status != SUCCESS)
    {
        fprintf(stderr, "Failed to init ivcam\n");
        exit(101);
    }

    CalibrationParameters *params = ivcam.ReadCalibrationData();
    printparams(params);

    status = ivcam.StartCapture(IMG_TYPE_COLOR | IMG_TYPE_DEPTH);

    if (status != SUCCESS)
    {
        fprintf(stderr, "Failed to start capturing\n");
        exit(102);
    }

    int framesInLine = 0;
    for (i = 0; i< 200; i++)
    {
        status = ivcam.ReadFrame(&frame);
        if (status == SUCCESS)
        	fprintf(stdout, "%s", (status == SUCCESS) ? ".":"0");
        framesInLine++;
        if (framesInLine == 80)
        {
            framesInLine = 0;
            fprintf(stdout, "\n");
        }
    }
    if (frame.depth)
    	delete frame.depth;
    if (frame.color)
    	delete frame.color;
    return 0;
}

void printparams(CalibrationParameters *params) {
	printf("$$$printParameters************************"); printf("\n");
	printf("$$$%f", params->Rmax); printf("\n");
	printf("$$$%f", params->Kc[0][0]); printf("\n");
	printf("$$$%f", params->Distc[0]); printf("\n");
	printf("$$$%f", params->Invdistc[0]); printf("\n");
	printf("$$$%f", params->Pp[0][0]); printf("\n");
	printf("$$$%f", params->Kp[0][0]); printf("\n");
	printf("$$$%f", params->Rp[0][0]); printf("\n");
	printf("$$$%f", params->Tp[0]); printf("\n");
	printf("$$$%f", params->Distp[0]); printf("\n");
	printf("$$$%f", params->Invdistp[0]); printf("\n");
	printf("$$$%f", params->Pt[0][0]); printf("\n");
	printf("$$$%f", params->Kt[0][0]); printf("\n");
	printf("$$$%f", params->Rt[0][0]); printf("\n");
	printf("$$$%f", params->Tt[0]); printf("\n");
	printf("$$$%f", params->Distt[0]); printf("\n");
	printf("$$$%f", params->Invdistt[0]); printf("\n");
	printf("$$$%f", params->QV[0]); printf("\n");
	printf("$$$printParameters ########################"); printf("\n");
}

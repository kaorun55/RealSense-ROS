/*
 * ivcam.h
 *
 *  Created on: Sep 28, 2014
 *      Author: ubu
 */

#ifndef IVCAM_H_
#define IVCAM_H_


#include <stdint.h>
#include "ivcamParameterReader.h"

#include <string>
#include <vector>

#define IN
#define OUT

namespace ivcam_env
{
	using namespace std;
#define SUCCESS  0
#define FAILURE -1
#define STREAM_COLOR (IMG_TYPE_COLOR)
#define STREAM_DEPTH (IMG_TYPE_DEPTH)
#define CLEAR(x) memset(&(x), 0, sizeof(x))

typedef int IVCAM_STATUS;
typedef int STREAM_TYPE;

#define STREAM_TYPE_DEPTH 1
#define STREAM_TYPE_COLOR 2
#define IMG_TYPE_DEPTH  STREAM_TYPE_DEPTH
#define IMG_TYPE_COLOR  STREAM_TYPE_COLOR

	class XUProperty;

    struct Frame
    {
        uint8_t* depth;
        uint8_t* color;
    };

    struct buffer
    {
        void *start;
        size_t length;
    };

    enum DEVICE
    {
        DEV_DEPTH = 1,
        DEV_COLOR = 2,
    };

    struct Point2DF32 {
          float x, y;
    };

    struct Point3DF32 {
        float x, y, z;
    };
//delete this at the end
//	typedef enum
//	{
//		RightHandedCoordinateSystem  =0,
//		LeftHandedCoordinateSystem
//	} TCoordinatSystemDirection;
// ************************
    class Ivcam
    {
    public:
        Ivcam();
        ~Ivcam();

        IVCAM_STATUS Init(int cw, int ch, int dw, int dh);
        IVCAM_STATUS StartCapture(STREAM_TYPE imageType);
        IVCAM_STATUS StopCapture(void);
        IVCAM_STATUS ReadFrame(Frame* pFrame);
        void Close(void);


        CalibrationParameters* ReadCalibrationData();

    	/**The method return the UV map of the depth image*/
    	//Input: npoints           - number of points to be maped (can be used to map part of the image).
    	//		 pos2d	           - array of 3D points in the the size of npoints
    	//						     pos2d.z is the depth in MM units and pos2d.x and pos2d.y are the index of the pixels.
    	//		 isUVunitsRelative - if true  - output UV units are in the range of 0-1 and relative to the image size
    	//							 if false - output UV units are absolute.
    	//Output: posc   - array of 3D points in the size of npoint pos2d.x and pos2d.y are the UV map of pixel (x,y)
    	//return: TIVCAM_STATUS
        IVCAM_STATUS  MapDepthToColorCoordinates(IN unsigned int npoints,IN Point3DF32 *pos2d, OUT Point2DF32 *posc, IN bool isUVunitsRelative = true);


    	/**The method return the UV map of the depth image*/
    	//Input: width             - width of the image.
    	//		 height			   - height of the image.
    	//		 pSrcDepth	       - array of input depth in MM in the size of width*height.
    	//		 isUVunitsRelative - if true  - output UV units are in the range of 0-1 and relative to the image size
    	//							 if false - output UV units are absolute.
    	//Output: pDestUV          - array of output UV map should be allocated in the size of width*height*2.
    	//return: TIVCAM_STATUS
        IVCAM_STATUS  MapDepthToColorCoordinates(IN unsigned int width, IN unsigned int height, IN uint16_t* pSrcDepth, OUT float* pDestUV, IN bool isUVunitsRelative = true);


    	/**The method convert the depth to coordinates in real world*/
    	//Input: npoints           - number of points to be converted (can be used to convert part of the image).
    	//		 pos2d	           - array of 3D points in the the size of npoints
    	//						     pos2d.z is the depth in MM units and pos2d.x and pos2d.y are the index of the pixels.
    	//Output: pos3d            - array of 3D points in the size of npoint pos2d.x pos2d.y pos2d.z are the coordinates in real world of pixel (x,y)
    	//Return: TIVCAM_STATUS
        IVCAM_STATUS  ProjectImageToRealWorld(IN unsigned int npoints, IN Point3DF32 *pos2d, OUT Point3DF32 *pos3d);


    	/**The method convert the depth to coordinates in real world*/
    	//Input: width             - width of the image.
    	//		 height			   - height of the image.
    	//		 pSrcDepth	       - array of input depth in MM in the size of width*height.
    	//Output: pDestXYZ         - array of output XYZ coordinates in real world of the corresponding to the input depth image should be allocated in the size of width*height*3.
    	//return: TIVCAM_STATUS
        IVCAM_STATUS  ProjectImageToRealWorld(IN unsigned int width, IN unsigned int height, IN uint16_t* pSrcDepth, OUT float* pDestXYZ);


    	/**The method convert the depth in 1/32 MM units to MM*/
    	//Input: d                 - depth in 1/32 MM units.
    	//return: float            - depth in MM units.
    	float  ConvertDepth_Uint16ToMM( IN uint16_t d);

public:
    	IVCAM_STATUS 		GetXUPropertyNames(vector<string> props);
//    	XUProperty *		GetXUProperty(string name);
    	int 				GetXUPropertyMin(string property);
    	int 				GetXUPropertyMax(string property);
    	int 				GetXUPropertyDef(string property);
    	int 				GetXUPropertyCur(string property);
    	int 				GetXUPropertyLen(string property);
    	IVCAM_STATUS 		SetXUProperty	(string property, void *val);
    	IVCAM_STATUS		SetXUProperty	(string property, int value);

    	int                 GetFdByDevice(DEVICE device);

    private:
        IVCAM_STATUS        OpenDevice(const char* devName);
        IVCAM_STATUS 		FindDevices(const char *devType);
        IVCAM_STATUS  		InitDevice(int fd, uint32_t format, uint32_t width, uint32_t height, uint32_t fps);
        IVCAM_STATUS        InitColorDevice(int w, int h);
        IVCAM_STATUS        InitDepthDevice(int w, int h);
        IVCAM_STATUS 		InitXU();
        //IVCAM_STATUS
        IVCAM_STATUS		InitCalibration(void);
        IVCAM_STATUS        SetFps(int fd,int fps);
        IVCAM_STATUS        InitMmap(int fd, ivcam_env::buffer*& bufsList);
        const char*         GetDeviceStrByDesc(int fd);

        IVCAM_STATUS        AllocateBuffers(int fd, int numOfBuffersToAllocate, struct buffer*& buffers);
        static int          rioctl(int fh, int request, void *arg);
        IVCAM_STATUS        readImage(int fd, uint8_t* pBuffer);
        void 				closeDevice(int fd, buffer *&bufList);

    private:
        const int m_numOfBuffers;
        char m_colorfName[240];
        char m_depthfName[240];
        int m_colorFd;
        int m_depthFd;
        int m_activeStreams;
        struct buffer *m_colorBuffers;
        struct buffer *m_depthBuffers;
        CalibrationParameters m_calibrationParams;

        IVCAMParameterReader *mHWMonitor;

        int mColorWidth, mColorHeight;
        int mDepthWidth,  mDepthHeight;
        friend class XUProperty;

    };
}

#endif /* IVCAM_H_ */

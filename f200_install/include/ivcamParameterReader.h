/*
 * ivcamParameterReader.h
 *
 *  Created on: Nov 11, 2014
 *      Author: albert
 */

#ifndef IVCAMPARAMETERREADER_H_
#define IVCAMPARAMETERREADER_H_

namespace ivcam_env
{
	typedef enum {IVCAM_SUCCESS, IVCAM_FAILURE = -1 } IVCAM_RESULT;
	typedef struct
	{
		float	Rmax;
		float	Kc[3][3];		//[3x3]: intrinsic calibration matrix of the IR camera
		float	Distc[5];		// [1x5]: forward distortion parameters of the IR camera
		float	Invdistc[5];	// [1x5]: the inverse distortion parameters of the IR camera
		float	Pp[3][4];		// [3x4] : projection matrix
		float	Kp[3][3];		// [3x3]: intrinsic calibration matrix of the projector
		float	Rp[3][3];		// [3x3]: extrinsic calibration matrix of the projector
		float	Tp[3];			// [1x3]: translation vector of the projector
		float	Distp[5];		// [1x5]: forward distortion parameters of the projector
		float	Invdistp[5];	// [1x5]: inverse distortion parameters of the projector
		float	Pt[3][4];		// [3x4]: IR to RGB (texture mapping) image transformation matrix
		float	Kt[3][3];
		float	Rt[3][3];
		float	Tt[3];
		float	Distt[5];		// [1x5]: The inverse distortion parameters of the RGB camera
		float	Invdistt[5];
		float   QV[6];
	} CalibrationParameters;

	class IVCAMParameterReader {
		static IVCAMParameterReader *theReader;
	public:
		static IVCAMParameterReader *get(void);
		virtual void SetDepthResolution(int w, int h) = 0;
		virtual bool StartTempCompensationLoop(void) = 0;
		virtual void StopTempCompensationLoop(void)  = 0;
		virtual CalibrationParameters *getParameters() = 0;
		virtual ~IVCAMParameterReader();
	};

}

#endif /* IVCAMPARAMETERREADER_H_ */

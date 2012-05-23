/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://pixhawk.ethz.ch>

(c) 2009 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

This file is part of the PIXHAWK project

    PIXHAWK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PIXHAWK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 * @file
 *   @brief Definition of the class PxCameraCalibration.
 *
 *   @author Petri Tanskanen <mavteam@student.ethz.ch>
 *
 */

/** @addtogroup camera_calibration */
/*@{*/


#ifndef _PX_CAMERA_CALIBRATION_H_
#define _PX_CAMERA_CALIBRATION_H_

#include <opencv2/core/core.hpp>

#include "PxMatrix.h"

/**
 * @brief The superclass for calibration data wrapper classes.
 *
 * PxCameraCalibration defines the interface to load calibration data for cameras and undistort images and points.
 *
 */
class PxCameraCalibration
{
public:
	/** @brief Creates an undistortion mapping for cvRemap. */
	virtual void initUndistortMap(cv::Mat &rMapX, cv::Mat &rMapY) const = 0;

	/** @brief Undistorts one or more points. */
	virtual void undistortPoints(const CvPoint2D32f *pSrc, CvPoint2D32f *pDest, const int count) const = 0;

	/** @brief Returns the intrinsic matrix. */
	const cv::Mat &getIntrinsicMatrix(void) const;

	/** @brief Returns the inverse of the intrinsic matrix. */
	const cv::Mat &getInverseIntrinsicMatrix(void) const;

	/** @brief Returns the image size for which this calibration is valid. */
	const cv::Size &getSize(void) const;

protected:
	cv::Size m_size;						///< Image Size for this calibration
	cv::Mat m_intrisicMatrix;
	cv::Mat m_intrisicMatrixInverse;
};

#endif //_PX_CAMERA_CALIBRATION_H_

/*@}*/

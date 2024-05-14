/////////////////////////////////////////////////////////////////////////////////////////
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA CORPORATION & AFFILIATES assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA CORPORATION & AFFILIATES. No third party distribution is allowed unless
// expressly authorized by NVIDIA. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA CORPORATION & AFFILIATES products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA CORPORATION & AFFILIATES.
//
// SPDX-FileCopyrightText: Copyright (c) 2015-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//
// NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
// property and proprietary rights in and to this material, related
// documentation and any modifications thereto. Any use, reproduction,
// disclosure or distribution of this material and related documentation
// without an express license agreement from NVIDIA CORPORATION or
// its affiliates is strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef SAMPLES_COMMON_MATHUTILS_HPP_
#define SAMPLES_COMMON_MATHUTILS_HPP_

#include <math.h>
#include <algorithm>
#include <dw/core/base/Types.h>
#include <fstream>

#define DEG2RAD(x) (static_cast<float>(x) * 0.01745329251994329575f)
#define RAD2DEG(x) (static_cast<float>(x) * 57.29577951308232087721f)

//Note all 4x4 matrices here are in column-major ordering

////////////////////////////////////////////////////////////
void cross(float dst[3], const float x[3], const float y[3]);

////////////////////////////////////////////////////////////
void normalize(float dst[3]);
void lookAt(float M[16], const float eye[3], const float center[3], const float up[3]);
void frustum(float M[16], const float l, const float r,
             const float b, const float t,
             const float n, const float f);

void perspective(float M[16], float fovy, float aspect, float n, float f);
void ortho(float M[16], const float l, const float r, const float b, const float t, const float n, const float f);
void ortho(float M[16], float fovy, float aspect, float n, float f);

dwTransformation3f rigidTransformation(dwQuaternionf const& rotation, dwVector3f const& translation);
void quaternionToRotationMatrix(dwMatrix3d& mat, dwQuaterniond const& quaternion);
void positionToTranslateMatrix(float32_t translate[16], const float32_t position[3]);
void rotationToTransformMatrix(float32_t transform[16], const float32_t rotation[9]);
void quaternionToEulerAngles(dwQuaternionf const& quaternion, float32_t& roll, float32_t& pitch, float32_t& yaw);

dwVector3f pos2DTo3D(const dwVector2f& in);
dwVector2f pos3DTo2D(const dwVector3f& in);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** Calculates focal length of pinhole camera based on horizontal and vertical Field Of View angle (in radians)
*   and size of the image
**/
dwVector2f focalFromFOV(const dwVector2f fov, dwVector2ui imageSize);

//------------------------------------------------------------------------------
// This is a wrapped around the Mat4.hpp ::  Mat4_AxB for easy usage.
// Returns a * b
dwTransformation3f operator*(const dwTransformation3f& a, const dwTransformation3f& b);

//------------------------------------------------------------------------------
// This is a wrapped around the Mat4.hpp ::  Mat4_AxB for easy usage.
// Sets a -> a * b
dwTransformation3f& operator*=(dwTransformation3f& a, const dwTransformation3f& b);

//------------------------------------------------------------------------------
// Transform point by transformation matrix
dwVector4f operator*(const dwTransformation3f& T, const dwVector4f& p);

//------------------------------------------------------------------------------
// This is a wrapped around the Mat4.hpp ::  Mat4_AxBinv for easy usage.
// Returns a * inv(b)
dwTransformation3f operator/(const dwTransformation3f& a, const dwTransformation3f& b);

//------------------------------------------------------------------------------
// This is a wrapped around the Mat4.hpp ::  Mat4_AxBinv for easy usage.
// Sets a -> a * inv(b)
dwTransformation3f& operator/=(dwTransformation3f& a, const dwTransformation3f& b);

//------------------------------------------------------------------------------
// Pretty printer for a dwTransformation3f object.
std::ostream& operator<<(std::ostream& o, const dwTransformation3f& tx);

//------------------------------------------------------------------------------
// Returns the translation component of the transformation T
dwVector3f getTranslation(const dwTransformation3f& T);

//------------------------------------------------------------------------------
// Returns the magnitude of the translation of T
float32_t getTranslationMagnitude(const dwTransformation3f& T);

//------------------------------------------------------------------------------
// Returns the magnitude of the rotation of T, as used in angle/axis representation
// It is computed as acos( (trace(R) - 1) / 2)
float32_t getRotationMagnitude(const dwTransformation3f& T);

//------------------------------------------------------------------------------
// Returns the rotation matrix based on roll, pitch and yaw in degrees
void getRotationMatrix(dwMatrix3f* R, float32_t rollInDegrees, float32_t pitchInDegrees, float32_t yawInDegrees);

//------------------------------------------------------------------------------
// Returns the homography matrix based on the input camera transformation matrix, output camera rotation and translation matrix
void computeHomography(dwMatrix3f* homographyOut, dwTransformation3f transformationIn, dwMatrix3f camOutRotationMatrix, float32_t camOutTranslation[], float32_t normal[], float32_t distanceToPlane);

//------------------------------------------------------------------------------
// Converts seconds to microseconds
inline constexpr dwTime_t secs2MicroSecs(float32_t sec)
{
    float32_t const microsec = 1000000.F * sec;
    return static_cast<dwTime_t>(microsec);
}

//------------------------------------------------------------------------------
// Converts seconds to microseconds
inline constexpr dwTime_t secs2MicroSecs(float64_t sec)
{
    float64_t const microsec = 1000000.F * sec;
    return static_cast<dwTime_t>(microsec);
}

//------------------------------------------------------------------------------
// Converts usec numbers that fit inside a float64 to seconds
template <typename Scalar = float64_t>
inline constexpr Scalar microSecs2Secs(dwTime_t usec)
{
    return static_cast<Scalar>(usec) / Scalar(1000000);
}

#endif // SAMPLES_COMMON_MATHUTILS_HPP_

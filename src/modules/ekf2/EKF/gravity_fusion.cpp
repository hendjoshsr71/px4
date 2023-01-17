/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gravity_fusion.cpp
 * Fuse observations from the gravity vector to constrain roll
 * and pitch (a la complementary filter).
 * 
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Daniel M. Sahu <danielmohansahu@gmail.com>
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

void Ekf::fuseGravity()
{
	// gravity
	constexpr float g = CONSTANTS_ONE_G;

	// get latest estimated orientation
	const float &q0 = _state.quat_nominal(0);
	const float &q1 = _state.quat_nominal(1);
	const float &q2 = _state.quat_nominal(2);
	const float &q3 = _state.quat_nominal(3);

	// zero-mean gaussian observation noise for acceleration
	// const float R_ACC_Z = sqrt(_params.accel_noise);
	constexpr float R_ACC_Z = 1.0f; // this is a model

	// use raw accelerometer measurement (body frame) at EKF time horizon
	const Vector3f measurement = _imu_sample_delayed.delta_vel / _imu_sample_delayed.delta_vel_dt - getAccelBias();

	// calculate innovation -> the difference between measured and estimated angles
	const Vector3f estimate = _R_to_earth.transpose() * Vector3f(0,0,-g);
	_gravity_innov = estimate - measurement;

	// Eulerf euler(_R_to_earth.transpose());
	// PX4_INFO("Euler (phi,theta,psi): (%f, %f, %f)", (double)euler(0), (double)euler(1), (double)euler(2));
	// PX4_INFO("measurement   (x,y,z): (%f, %f, %f)", (double)measurement(0), (double)measurement(1), (double)measurement(2));
	// PX4_INFO("estimate      (x,y,z): (%f, %f, %f)", (double)estimate(0), (double)estimate(1), (double)estimate(2));
	// PX4_INFO("innovation    (x,y,z): (%f, %f, %f)", (double)_gravity_innov(0), (double)_gravity_innov(1), (double)_gravity_innov(2));

	// auto-generated intermediate variables
	const float HK0 = 2*g;
	const float HK1 = HK0*q2;
	const float HK2 = -HK0*q3;
	const float HK3 = HK0*q0;
	const float HK4 = -HK0*q1;
	const float HK5 = P(0,3)*q1;
	const float HK6 = -HK5 + P(0,0)*q2 - P(0,1)*q3 + P(0,2)*q0;
	const float HK7 = P(0,2)*q2;
	const float HK8 = P(1,2)*q3;
	const float HK9 = HK7 - HK8 + P(2,2)*q0 - P(2,3)*q1;
	const float HK10 = powf(g, 2);
	const float HK11 = 4*HK10;
	const float HK12 = HK11*q0;
	const float HK13 = P(0,3)*q2;
	const float HK14 = HK13 - P(1,3)*q3 + P(2,3)*q0 - P(3,3)*q1;
	const float HK15 = HK11*q1;
	const float HK16 = HK11*q2;
	const float HK17 = P(1,2)*q0;
	const float HK18 = P(1,3)*q1;
	const float HK19 = HK17 - HK18 + P(0,1)*q2 - P(1,1)*q3;
	const float HK20 = HK11*q3;
	const float HK21 = HK0/(HK12*HK9 - HK14*HK15 + HK16*HK6 - HK19*HK20 + R_ACC_Z);
	const float HK22 = HK13 + P(0,0)*q1 + P(0,1)*q0 + P(0,2)*q3;
	const float HK23 = P(0,1)*q1;
	const float HK24 = HK23 + HK8 + P(1,1)*q0 + P(1,3)*q2;
	const float HK25 = HK5 + P(1,3)*q0 + P(2,3)*q3 + P(3,3)*q2;
	const float HK26 = P(2,3)*q2;
	const float HK27 = HK17 + HK26 + P(0,2)*q1 + P(2,2)*q3;
	const float HK28 = HK0/(HK12*HK24 + HK15*HK22 + HK16*HK25 + HK20*HK27 + R_ACC_Z);
	const float HK29 = 4*g;
	const float HK30 = P(1,1)*q1 + P(1,2)*q2;
	const float HK31 = 16*HK10;
	const float HK32 = P(1,2)*q1 + P(2,2)*q2;
	const float HK33 = HK29/(HK30*HK31*q1 + HK31*HK32*q2 + R_ACC_Z);
	
	// Observation Jacobians for X Axis
	SparseVector24f<0,1,2,3> Hfusion;
	Hfusion.at<0>() = HK1;
	Hfusion.at<1>() = HK2;
	Hfusion.at<2>() = HK3;
	Hfusion.at<3>() = HK4;

	// Calculate Kalman gains for X Axis
	Vector24f Kfusion;
	Kfusion(0) = HK21*HK6;
	Kfusion(1) = HK19*HK21;
	Kfusion(2) = HK21*HK9;
	Kfusion(3) = HK14*HK21;
	Kfusion(4) = HK21*(P(0,4)*q2 - P(1,4)*q3 + P(2,4)*q0 - P(3,4)*q1);
	Kfusion(5) = HK21*(P(0,5)*q2 - P(1,5)*q3 + P(2,5)*q0 - P(3,5)*q1);
	Kfusion(6) = HK21*(P(0,6)*q2 - P(1,6)*q3 + P(2,6)*q0 - P(3,6)*q1);
	Kfusion(7) = HK21*(P(0,7)*q2 - P(1,7)*q3 + P(2,7)*q0 - P(3,7)*q1);
	Kfusion(8) = HK21*(P(0,8)*q2 - P(1,8)*q3 + P(2,8)*q0 - P(3,8)*q1);
	Kfusion(9) = HK21*(P(0,9)*q2 - P(1,9)*q3 + P(2,9)*q0 - P(3,9)*q1);
	Kfusion(10) = HK21*(P(0,10)*q2 - P(1,10)*q3 + P(2,10)*q0 - P(3,10)*q1);
	Kfusion(11) = HK21*(P(0,11)*q2 - P(1,11)*q3 + P(2,11)*q0 - P(3,11)*q1);
	Kfusion(12) = HK21*(P(0,12)*q2 - P(1,12)*q3 + P(2,12)*q0 - P(3,12)*q1);
	Kfusion(13) = HK21*(P(0,13)*q2 - P(1,13)*q3 + P(2,13)*q0 - P(3,13)*q1);
	Kfusion(14) = HK21*(P(0,14)*q2 - P(1,14)*q3 + P(2,14)*q0 - P(3,14)*q1);
	Kfusion(15) = HK21*(P(0,15)*q2 - P(1,15)*q3 + P(2,15)*q0 - P(3,15)*q1);
	Kfusion(16) = HK21*(P(0,16)*q2 - P(1,16)*q3 + P(2,16)*q0 - P(3,16)*q1);
	Kfusion(17) = HK21*(P(0,17)*q2 - P(1,17)*q3 + P(2,17)*q0 - P(3,17)*q1);
	Kfusion(18) = HK21*(P(0,18)*q2 - P(1,18)*q3 + P(2,18)*q0 - P(3,18)*q1);
	Kfusion(19) = HK21*(P(0,19)*q2 - P(1,19)*q3 + P(2,19)*q0 - P(3,19)*q1);
	Kfusion(20) = HK21*(P(0,20)*q2 - P(1,20)*q3 + P(2,20)*q0 - P(3,20)*q1);
	Kfusion(21) = HK21*(P(0,21)*q2 - P(1,21)*q3 + P(2,21)*q0 - P(3,21)*q1);
	Kfusion(22) = HK21*(P(0,22)*q2 - P(1,22)*q3 + P(2,22)*q0 - P(3,22)*q1);
	Kfusion(23) = HK21*(P(0,23)*q2 - P(1,23)*q3 + P(2,23)*q0 - P(3,23)*q1);

	// perform fusion for X axis
	if (measurementUpdate(Kfusion, Hfusion, _gravity_innov(0)))
		PX4_DEBUG("Successfully fused X measurement.");

	// Observation Jacobians update for Y
	Hfusion.at<0>() = HK4;
	Hfusion.at<1>() = -HK3;
	Hfusion.at<2>() = HK2;
	Hfusion.at<3>() = -HK1;

	// Kalman gains update for Y Axis
	Kfusion(0) = -HK22*HK28;
	Kfusion(1) = -HK24*HK28;
	Kfusion(2) = -HK27*HK28;
	Kfusion(3) = -HK25*HK28;
	Kfusion(4) = -HK28*(P(0,4)*q1 + P(1,4)*q0 + P(2,4)*q3 + P(3,4)*q2);
	Kfusion(5) = -HK28*(P(0,5)*q1 + P(1,5)*q0 + P(2,5)*q3 + P(3,5)*q2);
	Kfusion(6) = -HK28*(P(0,6)*q1 + P(1,6)*q0 + P(2,6)*q3 + P(3,6)*q2);
	Kfusion(7) = -HK28*(P(0,7)*q1 + P(1,7)*q0 + P(2,7)*q3 + P(3,7)*q2);
	Kfusion(8) = -HK28*(P(0,8)*q1 + P(1,8)*q0 + P(2,8)*q3 + P(3,8)*q2);
	Kfusion(9) = -HK28*(P(0,9)*q1 + P(1,9)*q0 + P(2,9)*q3 + P(3,9)*q2);
	Kfusion(10) = -HK28*(P(0,10)*q1 + P(1,10)*q0 + P(2,10)*q3 + P(3,10)*q2);
	Kfusion(11) = -HK28*(P(0,11)*q1 + P(1,11)*q0 + P(2,11)*q3 + P(3,11)*q2);
	Kfusion(12) = -HK28*(P(0,12)*q1 + P(1,12)*q0 + P(2,12)*q3 + P(3,12)*q2);
	Kfusion(13) = -HK28*(P(0,13)*q1 + P(1,13)*q0 + P(2,13)*q3 + P(3,13)*q2);
	Kfusion(14) = -HK28*(P(0,14)*q1 + P(1,14)*q0 + P(2,14)*q3 + P(3,14)*q2);
	Kfusion(15) = -HK28*(P(0,15)*q1 + P(1,15)*q0 + P(2,15)*q3 + P(3,15)*q2);
	Kfusion(16) = -HK28*(P(0,16)*q1 + P(1,16)*q0 + P(2,16)*q3 + P(3,16)*q2);
	Kfusion(17) = -HK28*(P(0,17)*q1 + P(1,17)*q0 + P(2,17)*q3 + P(3,17)*q2);
	Kfusion(18) = -HK28*(P(0,18)*q1 + P(1,18)*q0 + P(2,18)*q3 + P(3,18)*q2);
	Kfusion(19) = -HK28*(P(0,19)*q1 + P(1,19)*q0 + P(2,19)*q3 + P(3,19)*q2);
	Kfusion(20) = -HK28*(P(0,20)*q1 + P(1,20)*q0 + P(2,20)*q3 + P(3,20)*q2);
	Kfusion(21) = -HK28*(P(0,21)*q1 + P(1,21)*q0 + P(2,21)*q3 + P(3,21)*q2);
	Kfusion(22) = -HK28*(P(0,22)*q1 + P(1,22)*q0 + P(2,22)*q3 + P(3,22)*q2);
	Kfusion(23) = -HK28*(P(0,23)*q1 + P(1,23)*q0 + P(2,23)*q3 + P(3,23)*q2);

	// perform fusion for Y axis
	if (measurementUpdate(Kfusion, Hfusion, _gravity_innov(1)))
		PX4_DEBUG("Successfully fused Y measurement.");

	// Observation Jacobians update for Z
	Hfusion.at<0>() = 0;
	Hfusion.at<1>() = HK29*q1;
	Hfusion.at<2>() = HK29*q2;
	Hfusion.at<3>() = 0;

	// Kalman gains update for Z Axis
	Kfusion(0) = HK33*(HK23 + HK7);
	Kfusion(1) = HK30*HK33;
	Kfusion(2) = HK32*HK33;
	Kfusion(3) = HK33*(HK18 + HK26);
	Kfusion(4) = HK33*(P(1,4)*q1 + P(2,4)*q2);
	Kfusion(5) = HK33*(P(1,5)*q1 + P(2,5)*q2);
	Kfusion(6) = HK33*(P(1,6)*q1 + P(2,6)*q2);
	Kfusion(7) = HK33*(P(1,7)*q1 + P(2,7)*q2);
	Kfusion(8) = HK33*(P(1,8)*q1 + P(2,8)*q2);
	Kfusion(9) = HK33*(P(1,9)*q1 + P(2,9)*q2);
	Kfusion(10) = HK33*(P(1,10)*q1 + P(2,10)*q2);
	Kfusion(11) = HK33*(P(1,11)*q1 + P(2,11)*q2);
	Kfusion(12) = HK33*(P(1,12)*q1 + P(2,12)*q2);
	Kfusion(13) = HK33*(P(1,13)*q1 + P(2,13)*q2);
	Kfusion(14) = HK33*(P(1,14)*q1 + P(2,14)*q2);
	Kfusion(15) = HK33*(P(1,15)*q1 + P(2,15)*q2);
	Kfusion(16) = HK33*(P(1,16)*q1 + P(2,16)*q2);
	Kfusion(17) = HK33*(P(1,17)*q1 + P(2,17)*q2);
	Kfusion(18) = HK33*(P(1,18)*q1 + P(2,18)*q2);
	Kfusion(19) = HK33*(P(1,19)*q1 + P(2,19)*q2);
	Kfusion(20) = HK33*(P(1,20)*q1 + P(2,20)*q2);
	Kfusion(21) = HK33*(P(1,21)*q1 + P(2,21)*q2);
	Kfusion(22) = HK33*(P(1,22)*q1 + P(2,22)*q2);
	Kfusion(23) = HK33*(P(1,23)*q1 + P(2,23)*q2);

	// perform fusion for Z axis
	if (measurementUpdate(Kfusion, Hfusion, _gravity_innov(2)))
		PX4_DEBUG("Successfully fused Z measurement.");
}

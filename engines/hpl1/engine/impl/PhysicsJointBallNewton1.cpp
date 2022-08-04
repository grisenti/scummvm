/*
 * Copyright (C) 2006-2010 - Frictional Games
 *
 * This file is part of HPL1 Engine.
 *
 * HPL1 Engine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * HPL1 Engine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with HPL1 Engine.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "hpl1/engine/impl/PhysicsJointBallNewton1.h"

#include "hpl1/engine/impl/PhysicsBodyNewton1.h"
#include "hpl1/engine/impl/PhysicsWorldNewton1.h"

namespace hpl {

	//////////////////////////////////////////////////////////////////////////
	// CONSTRUCTORS
	//////////////////////////////////////////////////////////////////////////

	//-----------------------------------------------------------------------

	cPhysicsJointBallNewton::cPhysicsJointBallNewton(const tString &asName,
						iPhysicsBody *apParentBody, iPhysicsBody *apChildBody,
						iPhysicsWorld *apWorld, const cVector3f &avPivotPoint)
	: iPhysicsJointNewton<iPhysicsJointBall>(asName,apParentBody,apChildBody,apWorld,avPivotPoint)
	{
		VEC3_CONST_ARRAY(pivotPint, avPivotPoint);
		mpNewtonJoint = NewtonConstraintCreateBall(mpNewtonWorld, pivotPint,
												mpNewtonChildBody, mpNewtonParentBody);

		mvPinDir = cVector3f(0,0,0);
		mvPivotPoint = avPivotPoint;

		mfMaxConeAngle =0;
		mfMaxTwistAngle = 0;
		mvConePin = mvPinDir;
	}

	//-----------------------------------------------------------------------

	cPhysicsJointBallNewton::~cPhysicsJointBallNewton()
	{
	}

	//-----------------------------------------------------------------------

	//////////////////////////////////////////////////////////////////////////
	// PUBLIC METHODS
	//////////////////////////////////////////////////////////////////////////

	//-----------------------------------------------------------------------

	void cPhysicsJointBallNewton::SetConeLimits(const cVector3f& avPin, float afMaxConeAngle, float afMaxTwistAngle)
	{
		VEC3_CONST_ARRAY(pin, avPin);
		NewtonBallSetConeLimits(mpNewtonJoint, pin, afMaxConeAngle, afMaxTwistAngle);
		mvConePin = avPin;
		mvPinDir = mvConePin;
		mfMaxConeAngle = afMaxConeAngle;
		mfMaxTwistAngle = afMaxTwistAngle;
	}

	cVector3f cPhysicsJointBallNewton::GetAngles()
	{
		float angles[3];
		NewtonBallGetJointAngle(mpNewtonJoint, angles);
		return cVector3f::fromArray(angles);
	}

	//-----------------------------------------------------------------------

	cVector3f cPhysicsJointBallNewton::GetVelocity()
	{
		return cVector3f(0,0,0);
	}
	cVector3f cPhysicsJointBallNewton::GetAngularVelocity()
	{
		float vel[3];
		NewtonBallGetJointOmega(mpNewtonJoint, vel);
		return cVector3f::fromArray(vel);
	}
	cVector3f cPhysicsJointBallNewton::GetForce()
	{
		float force[3];
		NewtonBallGetJointForce(mpNewtonJoint, force);
		return cVector3f::fromArray(force);
	}

	//-----------------------------------------------------------------------

	float cPhysicsJointBallNewton::GetDistance()
	{
		return 0;
	}
	float cPhysicsJointBallNewton::GetAngle()
	{
		return 0;
	}

	//-----------------------------------------------------------------------

}

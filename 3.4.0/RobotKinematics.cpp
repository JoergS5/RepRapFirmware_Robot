/*
 * RobotKinematics.cpp
 *
 *  Created on: 13 Apr 2022
 *      Author: JoergS5
 *
 *	documentation:
 *	https://docs.duet3d.com/User_manual/Machine_configuration/Configuring_RepRapFirmware_for_a_Robot_printer
 *	documentation DH parameters:
 *	https://docs.duet3d.com/User_manual/Machine_configuration/Configuring_Robot_DH_parameters
 *	discussion:
 *	https://forum.duet3d.com/topic/17421/robotic-kinematics/210?_=1607408460374
 */

#include "RobotKinematics.h"

#if SUPPORT_ROBOT

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <Storage/MassStorage.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Movement/DDA.h>
#include <Tools/Tool.h>
#include <Movement/Move.h>

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(RobotKinematics, __VA_ARGS__)

constexpr ObjectModelTableEntry RobotKinematics::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. kinematics members
	{ "name",	OBJECT_MODEL_FUNC(self->GetName(true)), 	ObjectModelEntryFlags::none },

	// 1. DH parameters
};

constexpr uint8_t RobotKinematics::objectModelTableDescriptor[] = { 1, 1 };

DEFINE_GET_OBJECT_MODEL_TABLE(RobotKinematics)

#endif

// Constructor
RobotKinematics::RobotKinematics() noexcept
	: Kinematics(KinematicsType::robot, SegmentationType(true, true, true))
{
}

// Return the name of the current kinematics.
// If 'forStatusReport' is true then the string must be the one for that kinematics expected by DuetWebControl and PanelDue.
// Otherwise it should be in a format suitable for printing.
// For any new kinematics, the same string can be returned regardless of the parameter.
const char *RobotKinematics::GetName(bool forStatusReport) const noexcept
{
	return "Robot";
}

/*
 *	Config: Process M669 and set parameters.
 *
 *	\param reply
 *		Error messages reported back.
 *
 *	\param error
 *		true if there was an error.
 *
 *	\return bool
 *		true if one of the parameters is changed.
 */

bool RobotKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException)
{
	if (mCode == 669)
	{
		bool seen = false;
		if (gb.Seen('A'))
		{
			String<StringLength20> str;
			if(gb.TryGetQuotedString('A', str.GetRef(), seen)) {
				const char * cArr = str.c_str();
				if(cArr[0] == 'M') {		// set axis mode, e.g. ABC or IJKUVW or none
					axisMode = cArr[1] - '0';
				}
				else {	// set axis config, i.e. which axes are rotational and prismatic
					for(int i=0; i < 7; i++) {
						if(cArr[i] == 'R') {
							axisConfig[i] = 'R';
						}
						else if(cArr[i] == 'P') {
							axisConfig[i] = 'P';
						}
						else {
							axisConfig[i] = ' ';
						}
					}
				}
				seen = true;
			}
			else {	// set axis' DH parameters and homing/min/max angles (7 parameters for each axis)
				float vals[8];	// axis number and 7 parameters
				if(gb.TryGetFloatArray('A', 8, vals, reply, seen, false)) {
					int axisnr = vals[0];
					for(int i=1; i < 8; i++) {
						dh[axisnr][i-1] = vals[i];
					}
					seen = true;
				}
			}
		}

		if (gb.Seen('P'))
		{
			pMode = gb.GetIValue();
			seen = true;
		}

		if (gb.Seen('Q'))
		{
			qMode = gb.GetIValue();
			seen = true;
		}

		if (gb.Seen('R'))
		{
			reportingMode = gb.GetIValue();
			seen = true;
		}

		if (gb.Seen('S'))
		{
			Kinematics::TryConfigureSegmentation(gb);
			seen = true;
		}

		if (gb.Seen('T'))
		{
			Kinematics::TryConfigureSegmentation(gb);
			seen = true;
		}

		return seen;
	}
	else // Config was called, but is was not a M669 command
	{
		return Kinematics::Configure(mCode, gb, reply, error);
	}
}

/*
 * Inverse kinematics, converting cartesian coordinates and orientation to actuator positions.
 *
 * If a motor speed is much higher than the maximum speed, it means the robot is near or at a singularity.
 * To avoid standstill (the overall speed for all axes would be lowered), the speed for this specific axis is set to 0,
 * resulting in an inexact movement. This setting is only effective for a short segment.
 *
 * \param machinePos
 * 		X, Y, Z (= cartesian in mm), rotations by X, Y, Z in degrees  = given value
 *
  *	\param stepsPerMm
 *		steps per mm for prismatic, steps per degrees for rotational axis
 *
 *	\param motorPos
 *		stepper positions = searched values
 *
 *	\return
 *		Return true if successful, false if we were unable to convert
 */

bool RobotKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes,
		size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept
{
	clock_t start, stop;
	std::srand(static_cast<unsigned int>(std::time(nullptr)));
	start = clock();

	// restore last position and save new one. Calculate planned distance => maximum allowed accels/speeds
	// calculate where in segmentation we are

	// get maximum speed/accel for each axis (and extruder?)
//	float maxFeedrate[numOfAxes];
//	float maxAccel[numOfAxes];
//	for(int i=0; i < numOfAxes; i++) {
//		maxFeedrate[i] = reprap.GetPlatform().MaxFeedrate(i);
//		maxAccel[i] = reprap.GetPlatform().Acceleration(i);
//	}

	// calculate target angles
	double angles[numOfAxes];
	double lastKnownAngles0[numOfAxes];
	for(int i=0; i < numOfAxes; i++) lastKnownAngles0[i] = lastKnownAngles[i];
	int iterations = findTarget(lastKnownAngles0, machinePos, angles);

	// calculate distance (in most cases, a segment)

	// then calculate maximum allowed speeds

	// limit angles to maximum values, if near singularity
	for(int i=0; i < numOfAxes; i++) {
		if(angles[i] > (double) 20.0) {
			angles[i] = (double) 20.0;
		}
		// TODO calculate new position and store as current position
	}


	segmentCounter++;


	// convert angels to motorPos
	anglesToMotorPos(angles, stepsPerMm, motorPos);

	if(reportingMode == 1) {
		stop = clock();
		double timediff = (double) (stop-start);
		outToConsole("CartesianToMotorSteps time: %f ms", timediff);
		outToConsole("iterations: %d", iterations);
		outToConsole("segment counter: %d", segmentCounter);
	}

	return true;
}

void RobotKinematics::outToConsole(const char *_ecv_array format, ...) const noexcept {
	va_list vargs;
	va_start(vargs, format);
	reprap.GetPlatform().DebugMessage(format, vargs);
	va_end(vargs);
}

// Convert motor positions (measured in steps from reference position) to Cartesian coordinates
// 'motorPos' is the input vector of motor positions
// 'stepsPerMm' is as configured in M92. On a Scara or polar machine this would actually be steps per degree.
// 'numDrives' is the number of machine drives to convert, which will always be at least 3
// 'machinePos' is the output set of converted axis and extruder positions

/*
 *	Forward kinematics: calculate endpoint position and orientation from motor positions.
 *
 *	\param motorPos
 *		motor position as angle*microsteps*ratio for rotational, or mm*microsteps*ratio for prismatic actuator.
 *
 *	\param stepsPerMm
 *		steps per mm for prismatic, steps per degrees for rotational axis
 *
 *	\param machinePos
 *		result six coordinates: x, y, z in mm, rotations by x, y, z in degrees
 *		TODO extruder position?
 */

void RobotKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes,
		size_t numTotalAxes, float machinePos[]) const noexcept
{
	double angles[numOfAxes];
	motorPosToAngles(motorPos, stepsPerMm, angles);

	double matrixLast[16];
	getForward(angles, matrixLast);
	double euler[3];
	getEulerAnglesZYX(matrixLast, euler);

	machinePos[0] = (float) matrixLast[3];	// X
	machinePos[1] = (float) matrixLast[7];	// Y
	machinePos[2] = (float) matrixLast[11];	// Z
	machinePos[3] = (float) euler[0];		// rotation by X
	machinePos[4] = (float) euler[1];		// rotation by Y
	machinePos[5] = (float) euler[2];		// rotation by Z
}

/*
 * Convert motorPos and stepsPerMm into angles (rotary joint) or mm (prismatic joint)
 *
 * \param motorPos
 *
 * \param stepsPerMm
 * 		steps per degree for rotary joint, steps per mm for prismatic joint
 *
 * 	\param angles
 * 		return value, mm or degrees respectively
 */

void RobotKinematics::motorPosToAngles(const int32_t motorPos[], const float stepsPerMm[], double angles[]) const noexcept {
	for(int i=0; i < numOfAxes; i++) {
		bool rotary = isRotational(i);
		if(rotary) {
			angles[i] = (double)((motorPos[i] * DegreesToRadians)/stepsPerMm[i]);
		}
		else {
			angles[i] = (double)(motorPos[i]/stepsPerMm[i]);
		}
	}

}

void RobotKinematics::anglesToMotorPos(const double angles[], const float stepsPerMm[], int32_t motorPos[])  const noexcept {
	for(int i=0; i < numOfAxes; i++) {
		bool rotary = isRotational(i);
		if(rotary) {
			motorPos[i] = (int32_t) (angles[i] * (double) stepsPerMm[i] / (double) DegreesToRadians);
		}
		else {
			motorPos[i] = angles[i] * (double) stepsPerMm[i];
		}
	}
}

// Return true if the specified XY position is reachable by the print head reference point.
bool RobotKinematics::IsReachable(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept
{
	return Kinematics::IsReachable(axesCoords, axes);
}

/*
 * Limit Cartesian position. Store planned move path for speed calculations.
 */

LimitPositionResult RobotKinematics::LimitPosition(float finalCoords[], const float * null initialCoords,
													size_t numAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept
{
	// store move path for speed calculations while moving in segments:
	if(initialCoords == nullptr) {		// for G2/G3 moves, there is no inialCoords, take last finalCoords instead
		for(int i=0; i < 6; i++) {
			plannedFrom[i] = plannedTo[i];
			plannedTo[i] = finalCoords[i];
			moveType = 2;
		}
	}
	else {
		for(int i=0; i < 6; i++) {
			plannedFrom[i] = initialCoords[i];
			plannedTo[i] = finalCoords[i];
			moveType = 0;
		}
	}

	segmentCounter = 0;	// start counting segments of the move


	const bool m208Limited = (applyM208Limits)
								? Kinematics::LimitPositionFromAxis(finalCoords, Z_AXIS, numAxes, axesToLimit)	// call base class function to limit Z and higher axes
								: false;
	return m208Limited ? LimitPositionResult::adjusted : LimitPositionResult::ok;
}

/*
 * Assumed position after switching to robot kinematics.
 *	Is called by GCodes::Reset(), after M669 and after M584 (drive mapping).
 *
 * \param positions
 * 		according to docu: cartesian X, Y, Z coordinates
 * 		but it seems to be coordinates of the axes => assume axes
 *
 */

void RobotKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept
{
	for(int i = 0; i < numOfAxes; i++) {
		positions[i] = 0.0;
	}
}

// Return the axes that we can assume are homed after executing a G92 command to set the specified axis coordinates
AxesBitmap RobotKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const noexcept
{
	return g92Axes;
}

// Return the set of axes that must be homed prior to regular movement of the specified axes
AxesBitmap RobotKinematics::MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept
{
	return axesMoving;
}

// This function is called when a request is made to home the axes in 'toBeHomed' and the axes in 'alreadyHomed' have already been homed.
// If we can proceed with homing some axes, return the name of the homing file to be called. Optionally, update 'alreadyHomed' to indicate
// that some additional axes should be considered not homed.
// If we can't proceed because other axes need to be homed first, return nullptr and pass those axes back in 'mustBeHomedFirst'.
AxesBitmap RobotKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept
{
	const AxesBitmap ret = Kinematics::GetHomingFileName(toBeHomed, alreadyHomed, numVisibleAxes, filename);
	return ret;
}

// This function is called from the step ISR when an endstop switch is triggered during homing.
// Return true if the entire homing move should be terminated, false if only the motor associated with the endstop switch should be stopped.
bool RobotKinematics::QueryTerminateHomingMove(size_t axis) const noexcept
{
	return false;
}

// This function is called from the step ISR when an endstop switch is triggered during homing after stopping just one motor or all motors.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate() and return false.
void RobotKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept
{
	double homingAngle = dh[axis][4];
	double countD = homingAngle * (double) stepsPerMm[axis];
	long count = long(countD);
	dda.SetDriveCoordinate(count, axis);
}

// Limit the speed and acceleration of a move to values that the mechanics can handle.
// The speeds in Cartesian space have already been limited.
void RobotKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes,
		bool continuousRotationShortcut) const noexcept
{
	// speed is limited in calculation CartesianToMotorSteps already
}

// Return true if the specified axis is a continuous rotation axis
bool RobotKinematics::IsContinuousRotationAxis(size_t axis) const noexcept
{
	return true;
}

// Return a bitmap of axes that move linearly in response to the correct combination of linear motor movements.
// This is called to determine whether we can babystep the specified axis independently of regular motion.
AxesBitmap RobotKinematics::GetLinearAxes() const noexcept
{
	return AxesBitmap::MakeFromBits(0);
}

// Update the derived parameters after the master parameters have been changed
void RobotKinematics::Recalc()
{
}


/*
 * Create transformation matrix TRRT from DH parameters and Z axis angle (if rotational) or mm (if prismatic).
 *
 * \param axisnr
 *
 * \param anglesOrDist
 * 	for rotational axis, added to theta. For prismatic axis, added to d.
 *
 * 	\param matrix
 * 		resulting RTTR matrix
 *
 */

void RobotKinematics::getMatrix(int matrixnr, double anglesOrDist, double matrix[]) const noexcept {
	double a = dh[matrixnr][0];
	double alpha = dh[matrixnr][1];
	double d = dh[matrixnr][2];
	double theta = dh[matrixnr][3];
	if(isRotational(matrixnr)) {		// rotatioal axis
		theta += anglesOrDist;
	}
	else {		// prismatic axis
		d += anglesOrDist;
	}
	getRotTrTrRotMatrix(theta, a, alpha, d, matrix);
}

bool RobotKinematics::isRotational(int matrixnr) const noexcept {
	if(axisConfig[matrixnr] == 'R') {
		return true;
	}
	else {
		return false;
	}
}

/*
 * Rotation matrix, rotate and transform by X first, then transform and rotate by Z.
 *
 * \param theta, a, alpha, d
 * 		the four DH parameters, theta and alpha are in degrees. a and d are in mm.
 *
 * \param matrix
 * 		result
 */
void RobotKinematics::getRotTrTrRotMatrix(double theta, double a, double alpha, double d, double matrix[]) const noexcept {
	double sinTh = sin(theta/(double)180.0*doublePi);
	double cosTh = cos(theta/(double)180.0*doublePi);
	double sinAl = sin(alpha/(double)180.0*doublePi);
	double cosAl = cos(alpha/(double)180.0*doublePi);

	matrix[0] = cosTh;
	matrix[1] = - cosAl * sinTh;
	matrix[2] = sinAl * sinTh;
	matrix[3] = a * cosTh;

	matrix[4] = sinTh;
	matrix[5] = cosAl * cosTh;
	matrix[6] = - sinAl * cosTh;
	matrix[7] = a * sinTh;

	matrix[8] = 0.0;
	matrix[9] = sinAl;
	matrix[10] = cosAl;
	matrix[11] = d;

	matrix[12] = 0.0;
	matrix[13] = 0.0;
	matrix[14] = 0.0;
	matrix[15] = 1.0;

	// if values are near 0.0, then set value to 0.0
//	for(int i=0; i < 16; i++) {
//		if(fabs(matrix[i]) < DOUBLETOZERO) matrix[i] = 0.0;
//	}
}

/*
 *	Calculate forward kinematics by multiplying the RTTR matrices and adding the endpoint offset and orientation.
 *
 *	\param angles
 *		angles for rotational axes, mm for prismatic ones.
 *
 *	\param matrixLast
 *		last matrix which contains the resulting end position and orientation.
 *
 */

void RobotKinematics::getForward(double angles[], double matrixLast[16]) const noexcept {
	double mxResult[16];
	double mxCurrent[16];
	double mxTemp[16];
	for(int i=0; i < numOfAxes; i++) {
		if(i == 0) {
			getMatrix(i, angles[i], mxResult);
		}
		else {
			for(int j=0; j < 16; j++) mxTemp[j] = mxResult[j];
			getMatrix(i, angles[i], mxCurrent);
			multiplyMatrix(4, 4, mxTemp, 4, 4, mxCurrent, mxResult);
		}
	}

	// add tool offsets and orientation
	for(int j=0; j < 16; j++) mxTemp[j] = mxResult[j];
	getMatrixToolEndpoint(mxCurrent);
	multiplyMatrix(4, 4, mxTemp, 4, 4, mxCurrent, mxResult);

	for(int i=0; i < 16; i++) {
		if(fabs(mxResult[i]) < DOUBLETOZERO) mxResult[i] = 0.0;
	}
	for(int i=0; i < 16; i++) matrixLast[i] = mxResult[i];
}


/*
 * Multiply two matrices and return result. The resulting matrix must be created with r1*c2 elements as
 * one-dimensional array.
 *
 * \param r1, c1, m1
 * 		first matrix with r1 rows and c1 columns
 * \param r2, c2, m2
 * 		second matrix with r2 rows and c2 columns
 * 	\param result
 * 		resulting matrix with r1 rows and c2 columns
 */

void RobotKinematics::multiplyMatrix(int r1, int c1, const double m1[], int r2, int c2, const double m2[],
		double result[]) const noexcept {
	for(int r=0; r < r1; r++) {
		for(int c=0; c < c2; c++) {
			result[r*c2 + c] = 0.0;
			for(int e=0; e < r2; e++) {
				result[r*c2 + c] += m1[r*c1 + e] * m2[e*c2 + c];
			}
		}
	}
}
/*
 *	Add current tool's G10 properties to the endpoint calculation
 *	The values of the current tool are fetched from platform directly.
 *	X, Y and Z offsets are read.
 *	U, V, W offsets are used as angle orientation, if used.
 *
 * 	\param matrixEndpoint
 * 		4x4 matrix of endpoint after adding Tool.
 */

void RobotKinematics::getMatrixToolEndpoint(double matrixEndpoint[]) const noexcept {
	// get coordinates of current tool:
	const Tool * tool = reprap.GetCurrentTool();
	double offsets[6];
	for(int i=0; i < 6; i++) {
		unsigned int axis = (size_t) i;
		offsets[i] = (double) Tool::GetOffset(tool, axis);
	}

	for(int i=0; i < 16; i++) matrixEndpoint[i] = 0.0;

	// X, Y, Z offsets:
	matrixEndpoint[3] = offsets[0];
	matrixEndpoint[7] = offsets[1];
	matrixEndpoint[11] = offsets[2];

	// set orientation to unchanged:
	matrixEndpoint[0] = (double) 1.0;
	matrixEndpoint[5] = (double) 1.0;
	matrixEndpoint[10] = (double) 1.0;

	matrixEndpoint[15] = 1;
}

/*
 * Calculate end position xyz and end point orientation.
 *
 * numOfAxes must be set.
 *
 * \param angles
 * 		stepper angles in degrees.
 * 		number of angles must match axisConfigNumeric
 *
 * 	\param endpoint
 * 		XYZ and orientation of endpoint. Must be 6-value array.
 *
 * 	\param jacobian
 * 		jacobian matrix, size is 6 rows and number-of-actuators columns
 * 		Upper three rows are x,y,z changes, lower ones are x,y,z orientation angle changes
 */

void RobotKinematics::getJacobian(double angles[], double endpoint[], double jacobian[]) const noexcept {
	double matrixLast[16];
	getForward(angles, matrixLast);
	endpoint[0] = matrixLast[3];
	endpoint[1] = matrixLast[7];
	endpoint[2] = matrixLast[11];

	double euler[3];
	getEulerAnglesZYX(matrixLast, euler);
	endpoint[3] = euler[0];
	endpoint[4] = euler[1];
	endpoint[5] = euler[2];

	// change angle of one axis and calculate result
	for(int i=0; i < numOfAxes; i++) {
		double anglesTest[numOfAxes];
		for(int j=0; j < numOfAxes; j++) anglesTest[j] = angles[j];

		anglesTest[i] = angles[i] + angleDiff;
		getForward(anglesTest, matrixLast);

		jacobian[         i] = matrixLast[3] - endpoint[0];		// difference of x
		jacobian[1*numOfAxes + i] = matrixLast[7] - endpoint[1];		// of y
		jacobian[2*numOfAxes + i] = matrixLast[11] - endpoint[2];	// of z

		double eulerTest[3];
		getEulerAnglesZYX(matrixLast, eulerTest);
		jacobian[3*numOfAxes + i] = eulerTest[0] - endpoint[3];		// difference of x orientation as angle
		jacobian[4*numOfAxes + i] = eulerTest[1] - endpoint[4];		// of y orientation as angle
		jacobian[5*numOfAxes + i] = eulerTest[2] - endpoint[5];		// of z orientation as angle
	}
}

/*
 * The angles were rotated Z first, then X (no Y rotation), so to calculate angles back,
 * the Euler variant ZYX must be used.
 *
 * \param mxForward
 * 		endpoint matrix, containing XYZ position and orientation information as coordinate vectors
 *
 * \param euler
 * 		result in degrees for the X, Y, Z axis
 */
void RobotKinematics::getEulerAnglesZYX(double mxForward[], double euler[]) const noexcept {
	double c11 = mxForward[0];
	double c21 = mxForward[4];
	double c31 = mxForward[8];
	double c32 = mxForward[9];
	double c33 = mxForward[10];
	euler[0] = atan2(c32, c33) / (double)2.0 / doublePi * (double)360.0;
	euler[1] = atan2(-c31, sqrt(c32*c32 + c33*c33)) / (double)2.0 / doublePi * (double)360.0;
	euler[2] = atan2(c21, c11) / (double)2.0 / doublePi * (double)360.0;
}

/*
 * Return inverse of quadratic Jacobian matrix.
 * If inverse is not possible, change values in jacobian a bit for values near 0 and try again
 * until an inverse can be calculated.
 *
 * \param axes
 * 	size of rows and columns
 *
 * 	\param jacobian
 * 		size must be axes*axes
 *
 * 	\param inverse
 * 		result with size axes*axes
 *
 * 	\return bool
 * 		true if successful calculated.
 */

bool RobotKinematics::getInverse(int axes, double jacobian[], double inverse[]) const noexcept {
	bool ok = getInverse2(axes, jacobian, inverse);
	if(! ok) {
		int ct =0;
		while(true) {
			for(int i=0; i < axes*axes; i++) {
				if(jacobian[i] < (double) 0.0001) {
					jacobian[i] += (double) 0.000001 * (double) i;
				}
			}
			bool ok = getInverse2(axes, jacobian, inverse);
			if(ok) {
				break;
			}
			ct++;
			if(ct > 500) {
				return false;
			}
		}
	}
	return true;
}

bool RobotKinematics::getInverse2(int axes, double jacobian[], double inverse[]) const noexcept {
	if(axes != 6) {	// only for a quadratic jacobian an inverse can be calculated (but can fail)
		return false;
	}

	double j[36];
	for(int i=0; i < 36; i++) j[i] = jacobian[i];

	for(int i=0; i < 36; i++) inverse[i] = 0.0;
	for(int i=0; i < 36; i+=7) inverse[i] = 1.0;

	for(int row = 0; row < 6; row++) {
		int col = row;

		double value = j[row*6 + row];
		if(value == (double) 0.0) {
			return false;		// probably not invertable matrix
		}

		j[row*6 + col] = 1.0;
		for(int i = col + 1; i < 6; i++) {
			j[row*6 + i] = j[row*6 + i] / value;
		}
		for(int i = 0; i < 6; i++) {
			inverse[row*6 + i] = inverse[row*6 + i] / value;
		}

		for(int r=0; r < 6; r++) {
			if(r != row) {
				double factor = j[r*6+col];
				j[r*6 + col] = 0.0;
				for(int i=col + 1; i < 6; i++) {
					j[r*6 + i] = j[r*6 + i] - factor * j[row*6 + i];
				}
				for(int i=0; i < 6; i++) {
					inverse[r*6 + i] = inverse[r*6 + i] - factor * inverse[row*6 + i];
				}
			}
		}
	}

	double determinant = j[0] * j[1*6+1] * j[2*6+2] * j[3*6+3] * j[4*6+4] * j[5*6+5];
	if(determinant == 0) {
		return false;
	}
	return true;
}

/*
 * Get target stepper angles by iteration, stop when error is small enough.
 *
 * \param lastKnownAngles
 * 		last stepper angles. Start calculation from here.
 * 	\param machinePos
 * 		coordinates and orientation which shall be reached
 * 	\param angles
 * 		resulting stepper angles
 * 	\return
 * 		number of iterations needed. -1 if goal was not reached.
 */

int RobotKinematics::findTarget(double lastKnownAngles0[], const float machinePos[], double anglesTarget[]) const noexcept {
	// get old coordinates and orientation:
	double endpoint[6];	// xyz, xyz angles
	double jacobian[6*6];
	getJacobian(lastKnownAngles0, endpoint, jacobian);
	double inverse[numOfAxes * 6];
	getInverse(numOfAxes, jacobian, inverse);

	// goal
	//double anglesTarget[numOfAxes];
	for(int i=0; i < 6; i++) anglesTarget[i] = lastKnownAngles0[i];

	double ep[6];
	for(int i=0; i < 6; i++) ep[i] = endpoint[i];

	int ct = 0;
	for(int i=0; i < 500; i++) {
		double xDiff = (double) machinePos[0] - ep[0];
		double yDiff = (double) machinePos[1] - ep[1];
		double zDiff = (double) machinePos[2] - ep[2];

		double alDiff = getDistance((double) machinePos[3],ep[3]);
		double beDiff = getDistance((double) machinePos[4],ep[4]);
		double gaDiff = getDistance((double) machinePos[5],ep[5]);

		if(xDiff > 10) xDiff = 10.0;
		if(xDiff < -10) xDiff = -10.0;
		if(yDiff > 10) yDiff = 10.0;
		if(yDiff < -10) yDiff = -10.0;
		if(zDiff > 10) zDiff = 10.0;
		if(zDiff < -10) zDiff = -10.0;
		if(alDiff > 10) alDiff = 10.0;
		if(alDiff < -10) alDiff = -10.0;
		if(beDiff > 10) beDiff = 10.0;
		if(beDiff < -10) beDiff = -10.0;
		if(gaDiff > 10) gaDiff = 10.0;
		if(gaDiff < -10) gaDiff = -10.0;

		// leave loop if difference of calculation is low enough
		if(fabs(xDiff) < precision && fabs(yDiff) < precision && fabs(zDiff) < precision
				&& fabs(alDiff) < precisionAngles && fabs(beDiff) < precisionAngles && fabs(gaDiff) < precisionAngles) {
			break;
		}

		double qDot[6];

		// get q velocities from inverse*X velocities
		for(int row = 0; row < 6; row++) {		// inverse rows
			qDot[row] = inverse[row*6+0]*xDiff + inverse[row*6+1]*yDiff + inverse[row*6+2]*zDiff
					+ inverse[row*6+3]*alDiff + inverse[row*6+4]*beDiff + inverse[row*6+5]*gaDiff;
		}

		double angleDiffReciproce = (double) 1.0 / angleDiff;
		for(int i=0; i < 6; i++) {
			anglesTarget[i] += qDot[i] / angleDiffReciproce;		// angleDiff 0.01 => correct by / 100.0 etc
		}

		getJacobian(anglesTarget, ep, jacobian);
		getInverse(numOfAxes, jacobian, inverse);
		ct++;
	}

	if(ct < 500) {
		return ct;
	}
	else {
		return -1;
	}
}

/*
 * Get angle distance, where 179 and -179 has difference 2 degrees.
 */

double RobotKinematics::getDistance(const double angle1, const double angle2) const noexcept {
	if(angle1 < -90 && angle2 > 90) {
		return angle1 - angle2 + (double) 360.0;
	}
	else if(angle1 > 90 && angle2 < -90) {
		return angle1 - angle2 - (double) 360.0;
	}
	else if(angle1 >= angle2) {
		return angle1 - angle2;
	}
	else if(angle2 > angle1 ){
		return angle1 - angle2;
	}
	else {
		return 0;
	}
}


#endif // SUPPORT_ROBOT

// End

/*
 * RobotKinematics.h
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

#ifndef SRC_MOVEMENT_KINEMATICS_ROBOTKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_ROBOTKINEMATICS_H_

#include "Kinematics.h"
#include <chrono>

#if SUPPORT_ROBOT

const double doublePi  = 3.141592653589793238463;


class RobotKinematics : public Kinematics
{
public:
	// common methods
	void Recalc();
	RobotKinematics() noexcept;
	const char *GetName(bool forStatusReport) const noexcept override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) override;

	// forward, inverse methods:
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept override;

	// reachability methods:
	bool IsReachable(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept override;
	LimitPositionResult LimitPosition(float finalCoords[], const float * null initialCoords, size_t numAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept override;

	// homing methods:
	void GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept override;
	HomingMode GetHomingMode() const noexcept override { return HomingMode::homeIndividualMotors; }
	AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const noexcept override;
	AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept override;
	AxesBitmap GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept override;
	bool QueryTerminateHomingMove(size_t axis) const noexcept override;
	void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept override;

	// speed methods:
	void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept override;

	// axis properties methods:
	bool IsContinuousRotationAxis(size_t axis) const noexcept override;
	AxesBitmap GetLinearAxes() const noexcept override;

protected:
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(dh)

private:
	const static int MAXNUMOFAXES = 6;		// maximum possible number of axes. After changing, source must be recompiled.

	void getMatrix(int matrixnr, double anglesOrDist, double matrix[]) const noexcept;
	bool isRotational(int matrixnr) const noexcept;
	void getRotTrTrRotMatrix(double theta, double a, double alpha, double d, double matrix[]) const noexcept;

	void motorPosToAngles(const int32_t motorPos[], const float stepsPerMm[], double angles[]) const noexcept;
	void anglesToMotorPos(const double angles[], const float stepsPerMm[], int32_t motorPos[]) const noexcept;
	void getForward(double angles[], double matrixLast[16]) const noexcept;
	void multiplyMatrix(int r1, int c1, const double m1[], int r2, int c2, const double m2[], double result[]) const noexcept;
	void getMatrixToolEndpoint(double matrixEndpoint[16]) const noexcept;

	void getJacobian(double angles[], double endpoint[], double jacobian[]) const noexcept;
	void getEulerAnglesZYX(double mxForward[], double euler[]) const noexcept;
	bool getInverse(int axes, double jacobian[], double inverse[]) const noexcept;
	bool getInverse2(int axes, double jacobian[], double inverse[]) const noexcept;

	double getDistance(double angle1, double angle2) const noexcept;
	int findTarget(double lastKnownAngles0[], const float machinePos[], double angles[]) const noexcept;

	void storePath(float from[], float to[]) const noexcept;

	void outToConsole(const char *_ecv_array format, ...) const noexcept;

	int axisMode;			// 0: XYZABC (default), 1: XYZIJKUVW, 2: XYZ
	int numOfAxes;			// number of axes, according to config.g setting
	char axisConfig[MAXNUMOFAXES];		// R are rotational, P prismatic axes
	double dh[MAXNUMOFAXES][7];	// [axes][DH parameters]: a, alpha, d, theta, home, min, max angles. index 0 is DH1
	double angleDiff = 0.001;	// angle difference for jacobian calculation
	static constexpr double DOUBLETOZERO = 0.00001;		// matrix values below this threshold are set to 0.0
	int pMode = 2;		// P0 vertical in 0 degree position, P2 vertical parallel to x axis, default
	int qMode = 1;		// quality, 1 worst, 5 best
	int reportingMode = 0;	// reporting mode, 0 means no reporting, 1 measure some times and output to console
	double offsets[3] = {0.0, 0.0, 0.0};		// X, Y, Z offsets of whole robot
	double precision = 0.005;		// how precise shall the iteration be for xyz coordinates
	double precisionAngles = 0.01; // how precise shall the iteration be for xyz orientation angles

	mutable float plannedFrom[6];	// x, y, z, rotx, roty, rotz
	mutable float plannedTo[6];	// x, y, z, rotx, roty, rotz
	mutable int moveType;	// 0 == G0/G1, (1 == G1), 2 == G2, 3 == G3
	mutable int segmentCounter = 0;		// count every CartesianToMotorSteps call until move finished
	mutable double lastKnownAngles[MAXNUMOFAXES];	// last known position calculation
};

#endif // SUPPORT_ROBOT

#endif /* SRC_MOVEMENT_KINEMATICS_ROBOTKINEMATICS_H_ */

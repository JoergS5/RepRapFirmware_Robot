To compile the firmware:
- get the Duet3D RepRapFirmware, one of 3.4.0 versions
- add RobotKinematics.h and .cpp into src/Movement/Kinematics directory

Change Kinematics.h:
replace robot5axis by robot in the enum of KinematicsType

Change Kinematics.cpp:
add:
#include "RobotKinematics.h"

add in Create() where the kinematics is decided:
#if SUPPORT_ROBOT
	case KinematicsType::robot:
		return new RobotKinematics();
#endif

Change Config/Pins.h:
Disable most kinematics to spare memory by setting from 1 to 0
e.g.
# define SUPPORT_FIVEBARSCARA	0

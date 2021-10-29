#include "ukf.h"
// Process noise standard deviation longitudinal acceleration in m/s^2
// For bycicle acceleration: (0.05g,0.2g)
#define std_a_ 1.25
// Process noise standard deviation yaw acceleration in rad/s^2
#define std_yawdd_ 1.0
// Laser measurement noise standard deviation position1 in m
#define std_laspx_  0.15
// Laser measurement noise standard deviation position2 in m
#define std_laspy_  0.15
// Radar measurement noise standard deviation radius in m
#define std_radr_  0.3
// Radar measurement noise standard deviation angle in rad
#define std_radphi_  0.03
// Radar measurement noise standard deviation radius change in m/s
#define std_radrd_  0.3

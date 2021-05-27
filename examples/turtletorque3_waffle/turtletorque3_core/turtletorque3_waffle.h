#ifndef TURTLETORQUE3_WAFFLE_H_
#define TURTLETORQUE3_WAFFLE_H_

#define NAME                             "Waffle or Waffle Pi"

#define WHEEL_RADIUS                     0.033           // meter
#define WHEEL_SEPARATION                 0.287           // meter (BURGER : 0.160, WAFFLE : 0.287)
#define TURNING_RADIUS                   0.1435          // meter (BURGER : 0.080, WAFFLE : 0.1435)
#define ROBOT_RADIUS                     0.220           // meter (BURGER : 0.105, WAFFLE : 0.220)
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw

#define CURRENT_INTEGER_RATIO            (1193 / 2.69)   //Ratio for XM430-W210
#define INTEGER_CURRENT_RATIO           (2.69 / 1193)   //Ratio for XM430-W210

#define MAX_LINEAR_VELOCITY              (WHEEL_RADIUS * 2 * 3.14159265359 * 77 / 60) // m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
#define MAX_ANGULAR_VELOCITY             (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s


#define MAX_CURRENT                         3.2
#define MIN_CURRENT                          -MAX_CURRENT

#define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY
#define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY


#endif  //TURTLEBOT3_WAFFLE_H_

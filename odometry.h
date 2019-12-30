
#define GEAR_RATIO            60 //for 1:60 motor//82
#define PPR                   5 //for 1:60 motor
#define DIAMETER              85 //150     //[mm]
#define TICKS_FOR_ROTATION    (GEAR_RATIO*PPR)

// Odometry constants
#define ODO_NO_MOVEMENT       (500.0*1000.0) // if no new pulse within the last 500ms -> velocity is zero
#define MIN_ODO_TICKS         5

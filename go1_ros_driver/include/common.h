
#ifndef GO1_DRIVER_COM_H
#define GO1_DRIVER_COM_H


typedef enum {
    MODE_IDDLE = 0,
    WALK_W_VEL = 2,
    WALK_W_POS = 3,
    STAND_DOWN = 5,
    STAND_UP = 6,
    DAMPING_MODE = 7,
} mode_enum;

// Gait enum as per unitree sdk
typedef enum {
    GAITYPE_IDDLE = 0,
    TROT = 1,
    TROT_RUNNING = 2,
    CLIMB_STAIR = 3,
    TROT_OBSTACLE = 4,
} gaitype_enum;

// 
typedef enum {
    LOW_SPEED = 0,
    MEDIUM_SPEED = 1,
    HIGH_SPEED = 2,
} speed_level_enum;


typedef struct {
    float x;
    float y;
    float z;
} position_3;


typedef struct {
    float x;
    float y;
    float z;
    float w;
} quarternion_4;


typedef struct {
    position_3 position;
    quarternion_4 orientation;
} pose;


typedef struct {
    float x;
    float y;
    float yaw;
} velocity_3;


typedef struct {
    pos pose;
    velocity_3 velocity;
} odom;

#endif 

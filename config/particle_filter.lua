map = "maps/GDC1.txt"
init_x = 14.7
init_y = 14.24
init_r = 0

-- simulator
-- GAMMA = 0.5;
-- MOTION_X_STD_DEV = 0.07;
-- MOTION_Y_STD_DEV = 0.02;
-- MOTION_A_STD_DEV = 0.02;
-- SENSOR_STD_DEV   = 0.05;
-- D_SHORT = 0.2
-- D_LONG = 0.1
-- P_OUTSIDE_RANGE = 0.0

-- log data
GAMMA = 0.02;
SENSOR_STD_DEV   = 0.5;
D_SHORT = SENSOR_STD_DEV * 1.5;
D_LONG = SENSOR_STD_DEV * 2.5;
P_OUTSIDE_RANGE = 1e-4;

MOTION_X_STD_DEV = 0.15;
MOTION_Y_STD_DEV = 0.15;
MOTION_A_STD_DEV = 0.01;

MOTION_DIST_K1 = 1.0;
MOTION_DIST_K2 = 0.5;
MOTION_A_K1 = 0.5;
MOTION_A_K2 = 1.5;

-- 22-44 go-straight --> turn --> go-straight
-- 23-38 go-straight --> turn --> go-straight (hard)
-- 25-18 go-straight (hard)
-- 27-15 go-straight
-- 09-24 go back and forth
-- 08-55 go back and forth
-- 14-01 go bach and forth


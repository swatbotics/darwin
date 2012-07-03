#ifndef _JOYSTICK_DEFINITIONS_H_
#define _JOYSTICK_DEFINITIONS_H_

#define A_BUTTON 0
#define B_BUTTON 1
#define X_BUTTON 2
#define Y_BUTTON 3

#define LB_BUTTON 4
#define RB_BUTTON 5
#define START_BUTTON 6
#define SELECT_BUTTON 7

#define L_STICK_BUTTON 8
#define R_STICK_BUTTON 9

#define LEFT_AXIS_LEFT_RIGHT 0
#define LEFT_AXIS_UP_DOWN 1

#define RIGHT_AXIS_LEFT_RIGHT 3
#define RIGHT_AXIS_UP_DOWN 4


/* Smaller numbers make the head move faster*/
#define HEAD_MOVEMENT_SCALING 10

#define LEFT_TRIGGER 2


#endif

/* Diagram of Right Stick with return values
             
          0,-32768
             |
-32768,0 -- 0,0 -- 32768,0
	     |
          0, 32768                        */

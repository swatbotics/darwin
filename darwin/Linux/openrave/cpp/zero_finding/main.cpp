/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <string>
#include <iostream>
#include <sstream>

#include "Body.h"
#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

using namespace std;

int getInput(string prompt){
  string input = "";
  int myNumber = 0;

  while (true) {
    cout << prompt ;
    getline(cin, input);

    // This code converts from string to number safely.
    stringstream myStream(input);
    if (myStream >> myNumber)
      break;
    cout << "Invalid number, please try again" << endl;
  }

  return myNumber;
}


int main(void)
{
	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730("/dev/ttyUSB0");
	CM730 cm730(&linux_cm730);
	if(MotionManager::GetInstance()->Initialize(&cm730) == false)
	{
		printf("Fail to initialize Motion Manager!\n");
			return 0;
	}
	MotionManager::GetInstance()->AddModule((MotionModule*)Body::GetInstance());	
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();

    //MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(false);
	MotionManager::GetInstance()->SetEnable(true);
	/////////////////////////////////////////////////////////////////////

    for (int i=1; i<=20; i++){
      Body::GetInstance()->m_Joint.SetEnable(i,true,true);
      Body::GetInstance()->m_Joint.SetAngle(i,0);
    }
    
    int motor_number, angle;

    while(1)
    { 
      motor_number = getInput("Enter Moter Number: ");
      angle = getInput("Enter Angle: ");
      Body::GetInstance()->m_Joint.SetAngle(motor_number,angle);
    }

    return 0;
}

/*
 *   MotionManager.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <stdio.h>
#include <sys/time.h>
#include <signal.h>
#include <pthread.h>
#include "MX28.h"
#include "MotionManager.h"

using namespace Robot;

MotionManager* MotionManager::m_UniqueInstance = new MotionManager();

MotionManager::MotionManager() :
        m_CM730(0),
        m_ProcessEnable(false),
        m_Enabled(false),
        m_IsRunning(false),
        DEBUG_PRINT(false)
{
}

MotionManager::~MotionManager()
{
}

bool MotionManager::Initialize(CM730 *cm730)
{
	int value, error;

	m_CM730 = cm730;
	m_Enabled = false;
	m_ProcessEnable = true;

	if(m_CM730->Connect() == false)
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "Fail to connect CM-730\n");
		return false;
	}

	for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "ID:%d initializing...", id);
		
		if(m_CM730->ReadWord(id, MX28::P_PRESENT_POSITION_L, &value, &error) == CM730::SUCCESS)
		{
			MotionStatus::m_CurrentJoints.SetValue(id, value);
			MotionStatus::m_CurrentJoints.SetEnable(id, true);

			if(DEBUG_PRINT == true)
				fprintf(stderr, "[%d] Success\n", value);
		}
		else
		{
			MotionStatus::m_CurrentJoints.SetEnable(id, false);

			if(DEBUG_PRINT == true)
				fprintf(stderr, " Fail\n");
		}
	}

	m_SensorCalibrated = false;
	m_CalibrationTime = 0;
	m_FBGyroCenter = 0;
	m_RLGyroCenter = 0;

	return true;
}

bool MotionManager::Reinitialize()
{
	m_ProcessEnable = false;

	m_CM730->DXLPowerOn();

	int value, error;
	for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "ID:%d initializing...", id);
		
		if(m_CM730->ReadWord(id, MX28::P_PRESENT_POSITION_L, &value, &error) == CM730::SUCCESS)
		{
			MotionStatus::m_CurrentJoints.SetValue(id, value);
			MotionStatus::m_CurrentJoints.SetEnable(id, true);

			if(DEBUG_PRINT == true)
				fprintf(stderr, "[%d] Success\n", value);
		}
		else
		{
			MotionStatus::m_CurrentJoints.SetEnable(id, false);

			if(DEBUG_PRINT == true)
				fprintf(stderr, " Fail\n");
		}
	}

	m_ProcessEnable = true;
	return true;
}

void MotionManager::StartThread()
{
    pthread_t motion_thread = 0;
    pthread_create(&motion_thread, NULL, ThreadFunc, NULL);
    pthread_detach(motion_thread);
}

void* MotionManager::ThreadFunc(void* args)
{
    sigset_t sigs;
    sigfillset(&sigs);
    pthread_sigmask(SIG_BLOCK, &sigs, NULL);

    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);

    while(1)
    {
        t.tv_nsec += 8*1000*1000;   // 8 ms
        if(t.tv_nsec > 1000*1000*1000)
        {
            t.tv_nsec = t.tv_nsec - 1000*1000*1000;
            t.tv_sec += 1;
        }
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &t, NULL);

        MotionManager::GetInstance()->Process();

        pthread_testcancel();
    }
    return 0;
}

#define WINDOW_SIZE 30
void MotionManager::Process()
{
    if(m_ProcessEnable == false || m_IsRunning == true)
        return;

    m_IsRunning = true;

    // calibrate gyro sensor
    if(m_SensorCalibrated == false)
    {
        if(m_CalibrationTime <= 20)
        {
            if(m_CM730->m_BulkReadData[CM730::ID_CM].error == 0)
            {
                m_FBGyroCenter += m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L);
                m_RLGyroCenter += m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L);
                m_CalibrationTime++;
            }
        }
        else
        {
            m_FBGyroCenter = (int)((double)m_FBGyroCenter / (double)m_CalibrationTime);
            m_RLGyroCenter = (int)((double)m_RLGyroCenter / (double)m_CalibrationTime);
            m_SensorCalibrated = true;
            if(DEBUG_PRINT == true)
                fprintf(stderr, "FBGyroCenter:%d , RLGyroCenter:%d \n", m_FBGyroCenter, m_RLGyroCenter);
        }
    }

    if(m_SensorCalibrated == true && m_Enabled == true)
    {
        static int fb_array[WINDOW_SIZE] = {512};
        static int buf_idx = 0;
        if(m_CM730->m_BulkReadData[CM730::ID_CM].error == 0)
        {
            MotionStatus::FB_GYRO = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L) - m_FBGyroCenter;
            MotionStatus::RL_GYRO = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L) - m_RLGyroCenter;
            MotionStatus::RL_ACCEL = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L);
            MotionStatus::FB_ACCEL = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L);
            fb_array[buf_idx] = MotionStatus::FB_ACCEL;
            if(++buf_idx >= WINDOW_SIZE) buf_idx = 0;
        }

        int sum = 0, avr = 512;
        for(int idx = 0; idx < WINDOW_SIZE; idx++)
            sum += fb_array[idx];
        avr = sum / WINDOW_SIZE;

        if(avr < MotionStatus::FALLEN_F_LIMIT)
            MotionStatus::FALLEN = FORWARD;
        else if(avr > MotionStatus::FALLEN_B_LIMIT)
            MotionStatus::FALLEN = BACKWARD;
        else
            MotionStatus::FALLEN = STANDUP;

        if(m_Modules.size() != 0)
        {
            for(std::list<MotionModule*>::iterator i = m_Modules.begin(); i != m_Modules.end(); i++)
            {
                (*i)->Process();
                for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
                {
                    if((*i)->m_Joint.GetEnable(id) == true)
                    {
                        MotionStatus::m_CurrentJoints.SetSlope(id, (*i)->m_Joint.GetCWSlope(id), (*i)->m_Joint.GetCCWSlope(id));
                        MotionStatus::m_CurrentJoints.SetValue(id, (*i)->m_Joint.GetValue(id));

                        MotionStatus::m_CurrentJoints.SetPGain(id, (*i)->m_Joint.GetPGain(id));
                        MotionStatus::m_CurrentJoints.SetIGain(id, (*i)->m_Joint.GetIGain(id));
                        MotionStatus::m_CurrentJoints.SetDGain(id, (*i)->m_Joint.GetDGain(id));
                    }
                }
            }
        }

        int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];
        int n = 0;
        int joint_num = 0;
        for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
        {
            if(MotionStatus::m_CurrentJoints.GetEnable(id) == true)
            {
                param[n++] = id;
#ifdef MX28_1024
                param[n++] = MotionStatus::m_CurrentJoints.GetCWSlope(id);
                param[n++] = MotionStatus::m_CurrentJoints.GetCCWSlope(id);
#else
                param[n++] = MotionStatus::m_CurrentJoints.GetPGain(id);
                param[n++] = MotionStatus::m_CurrentJoints.GetIGain(id);
                param[n++] = MotionStatus::m_CurrentJoints.GetDGain(id);
                param[n++] = 0;
#endif
                param[n++] = CM730::GetLowByte(MotionStatus::m_CurrentJoints.GetValue(id));
                param[n++] = CM730::GetHighByte(MotionStatus::m_CurrentJoints.GetValue(id));
                joint_num++;
            }

            if(DEBUG_PRINT == true)
                fprintf(stderr, "ID[%d] : %d \n", id, MotionStatus::m_CurrentJoints.GetValue(id));
        }

        if(joint_num > 0)
#ifdef MX28_1024
            m_CM730->SyncWrite(MX28::P_CW_COMPLIANCE_SLOPE, MX28::PARAM_BYTES, joint_num, param);
#else
            m_CM730->SyncWrite(MX28::P_P_GAIN, MX28::PARAM_BYTES, joint_num, param);
#endif
    }

    m_CM730->BulkRead();

    if(m_CM730->m_BulkReadData[CM730::ID_CM].error == 0)
        MotionStatus::BUTTON = m_CM730->m_BulkReadData[CM730::ID_CM].ReadByte(CM730::P_BUTTON);

    m_IsRunning = false;
}

void MotionManager::SetEnable(bool enable)
{
	m_Enabled = enable;
	if(m_Enabled == true)
		m_CM730->WriteWord(CM730::ID_BROADCAST, MX28::P_MOVING_SPEED_L, 0, 0);
}

void MotionManager::AddModule(MotionModule *module)
{
	module->Initialize();
	m_Modules.push_back(module);
}

void MotionManager::RemoveModule(MotionModule *module)
{
	m_Modules.remove(module);
}

void MotionManager::SetJointDisable(int index)
{
    if(m_Modules.size() != 0)
    {
        for(std::list<MotionModule*>::iterator i = m_Modules.begin(); i != m_Modules.end(); i++)
            (*i)->m_Joint.SetEnable(index, false);
    }
}

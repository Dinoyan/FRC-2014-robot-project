/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 * @author Cybernetics
 */
public class MotorBrake 
{
    MotorBrake()
    {
        m_speedController =  null;
        m_robotDrive = null;
        m_encoder = null;
        m_enabled = false;
        m_running = false;
        m_lastPidGetVal = 0;
        m_lastTime = 0;
        m_movingForward = false;
    }

    public void Init(SpeedController speedController,Encoder encoder)
    {
        m_robotDrive = null;
        m_speedController = speedController;
        m_encoder = encoder;
    }
    
    public void Init(RobotDrive robotDrive,Encoder encoder)
    {
        m_speedController = null;
        m_robotDrive = robotDrive;
        m_encoder = encoder;
    }
    
    public boolean isEnabled()
    {
        return m_enabled;
    }
    
    public void Start()
    {
        m_enabled = true;
    }
    
    public void Stop()
    {
        if (null != m_speedController)
        {
            m_speedController.set(0);
        }
        
        if (null != m_robotDrive)
        {
            m_robotDrive.drive(0, 0);
        }

        if (null != m_encoder)
        {
            m_encoder.reset();
        }
        m_running = false;
        m_enabled = false;
    }
    
    public void PeriodicFunc()
    {
        Logger.PrintLine("MotorBreak::PeriodicFunc: 1 m_enabled = " + m_enabled + " m_running = " + m_running, Logger.LOGGER_MESSAGE_LEVEL_INFO);
        if(!m_enabled)
            return;
        
        if(!m_running)
        {
            m_encoder.reset();
            m_encoder.start();
            m_lastPidGetVal = m_encoder.pidGet();
            if (null != m_speedController)
            {
                m_speedController.set(0);
            }

            if (null != m_robotDrive)
            {
                m_robotDrive.drive(0, 0);
            }
            m_lastTime = Timer.getFPGATimestamp();
            m_running = true;
            m_directionKnown = false;
            Logger.PrintLine("MotorBreak::PeriodicFunc: start ", Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
        }
        else if ( (Timer.getFPGATimestamp() - m_lastTime) > 0.2 )
        {
            m_lastTime = Timer.getFPGATimestamp();
            double pidGetVal = m_encoder.pidGet();
            double RotattionDiff = pidGetVal - m_lastPidGetVal;
            m_lastPidGetVal = pidGetVal;
            
            
            if (!m_directionKnown)
            {
                if (pidGetVal > 0)
                {
                    m_movingForward = true;
                }
                else
                {
                    m_movingForward = false;
                }
                m_directionKnown = true;
            }
            
            System.out.println("MotorBreak::PeriodicFunc: 2 pidGetVal = " + pidGetVal + " m_movingForward = " + m_movingForward + " RotattionDiff = " + RotattionDiff);
            
            if (m_movingForward)
            {
                if (RotattionDiff <= 0)
                {
                    Stop();
                }
                else
                {
                    if (null != m_speedController)
                    {
                        m_speedController.set(-BRAKE_SPEED);
                    }

                    if (null != m_robotDrive)
                    {
                        m_robotDrive.drive(-BRAKE_SPEED, 0);
                    }
                }
            }
            else
            {
                if (RotattionDiff >= 0)
                {
                    Stop();
                }
                else
                {
                    if (null != m_speedController)
                    {
                        m_speedController.set(BRAKE_SPEED);
                    }

                    if (null != m_robotDrive)
                    {
                        m_robotDrive.drive(BRAKE_SPEED, 0);
                    }
                }
            }
        }
    }
    
    private SpeedController m_speedController;
    private RobotDrive m_robotDrive;
    private Encoder m_encoder;
    private boolean m_enabled;
    private boolean m_running;
    private double m_lastPidGetVal;
    private double m_lastTime;
    private boolean m_directionKnown;
    private boolean m_movingForward;
    private static final double BRAKE_SPEED =0.1;
}

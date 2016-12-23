/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;


/**  
 *
 * @author Cybernetics
 */
public class EncoderRPM
{
    private SpeedController m_speedController;
    private Encoder m_encoder;
    private double m_targetRpm;
    private double m_maxRpm;
    private double m_errorPercent;
    private double m_lastValueSet;
    private boolean m_running;
    private double m_lastTime;
    private double m_lastPidGetVal;
    private double m_targetRotations;
    private DigitalInput m_switch;
    private boolean m_enabled;
    
    private static final double RPM_ADJUSTMENT_FACTOR = 0.05;
    private static final double MAX_SPEED_FORWARD = 1;
    private static final double MAX_SPEED_REVERSE = -1;
    private static final double INVALID_ROTATION_COUNT = -1;
    
    EncoderRPM()
    {
        m_speedController = null;
        m_encoder = null;
        m_targetRpm = 0;
        m_maxRpm = 0;
        m_errorPercent = 0;
        m_lastValueSet = 0;
        m_running = false;
        m_lastTime = 0;
        m_lastPidGetVal = 0;
        m_switch = null;
        m_enabled = false;
    }
    
    public void Init(SpeedController speedController,Encoder encoder, double rpm, double maxRpm, double error, double targetRotations)
    {
        Init(speedController,encoder,rpm,maxRpm,error, targetRotations, null);
    }
    
    public void Init(SpeedController speedController,Encoder encoder, double rpm, double maxRpm, double error, DigitalInput limitSwitch)
    {
         Init(speedController,encoder,rpm,maxRpm,error,INVALID_ROTATION_COUNT,limitSwitch);
    }
    
    public void Init(SpeedController speedController,Encoder encoder, double rpm, double maxRpm, double error, double targetRotations, DigitalInput limitSwitch)
    {
        
        m_speedController = speedController;
        m_encoder = encoder;
        m_targetRpm = rpm;
        m_maxRpm = maxRpm;
        m_errorPercent = error;
        m_lastValueSet = 0;
        m_targetRotations = targetRotations;
        m_switch = limitSwitch;
        m_encoder.reset();
        m_enabled = false;
        m_running = false;
    }
    
    public void SetRpm(double rpm)
    {
        m_targetRpm = rpm;
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
        m_lastValueSet = 0;
        if (null != m_encoder)
        {
            m_encoder.reset();
        }
        m_running = false;
        m_enabled = false;
    }
    
    public void PeriodicFunc()
    {
        Logger.PrintLine("EncoderRPM::PeriodicFunc: m_enabled = " + m_enabled, Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
            
        if(null != m_switch)
        {
            Logger.PrintLine("EncoderRPM::PeriodicFunc: m_switch = " + m_switch.get(), Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
        }
        
        if(!m_enabled)
            return;
        boolean isLimitSwitchTriggered = false; 
        
        if((null != m_switch) && (!m_switch.get()))
        {
            Logger.PrintLine("EncoderRPM::PeriodicFunc: m_switch.get() = " + m_switch.get(), Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
            isLimitSwitchTriggered = true;
        }
        
        if(isLimitSwitchTriggered || ((INVALID_ROTATION_COUNT != m_targetRotations) && (Math.abs(m_encoder.pidGet()) >= m_targetRotations)))
        {
            Logger.PrintLine("EncoderRPM::PeriodicFunc: pidGet = " + m_encoder.pidGet() + " m_targetRotations = " + m_targetRotations + " isLimitSwitchTriggered = " + isLimitSwitchTriggered, Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
            Stop();
        }
        else
        {
            if(!m_running)
            {
                m_encoder.reset();
                m_encoder.start();
                m_lastPidGetVal = m_encoder.pidGet();
                m_lastValueSet = m_targetRpm/m_maxRpm / 2;
                m_speedController.set(m_lastValueSet);
                m_lastTime = Timer.getFPGATimestamp();
                m_running = true;
                System.out.println("EncoderRPM::PeriodicFunc: m_lastValueSet = " + m_lastValueSet);
            }
            else if ( (Timer.getFPGATimestamp() - m_lastTime) > 0.1 )
            {
                double curreTime = Timer.getFPGATimestamp();
                double timeDiff = curreTime - m_lastTime;
                double pidGetVal = m_encoder.pidGet();
                double currentRpm = (pidGetVal - m_lastPidGetVal) / timeDiff * 60;
                double rpmDiff = currentRpm - m_targetRpm;
                double rpmDiffPerecent = rpmDiff / m_targetRpm;
                double setValue = m_lastValueSet;

                m_lastPidGetVal = pidGetVal;
                m_lastTime = curreTime;

                if(Math.abs(rpmDiffPerecent) > m_errorPercent)
                {
                    setValue = m_lastValueSet - (rpmDiff / m_maxRpm) * RPM_ADJUSTMENT_FACTOR;

                    if (m_targetRpm > 0)
                    {
                        if (setValue < 0)
                            setValue = 0;
                        else if (setValue > MAX_SPEED_FORWARD)
                            setValue = MAX_SPEED_FORWARD;
                    }
                    else if (m_targetRpm < 0)
                    {
                        if (setValue > 0)
                            setValue = 0;
                        else if (setValue < MAX_SPEED_REVERSE)
                            setValue = MAX_SPEED_REVERSE;
                    }
                }

                m_speedController.set(setValue);
                m_lastValueSet = setValue;
            } 
        }
            
    }
    
    
}

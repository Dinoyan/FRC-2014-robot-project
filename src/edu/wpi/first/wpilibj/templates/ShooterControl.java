/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 * @author Cybernetics
 */
public class ShooterControl
{
    ShooterControl(Encoder encoder, SpeedController pullBackSpeedController, double speedControllerMaxRpm,
                   DigitalInput limitSwitch, DoubleSolenoid gearControl, Servo latchServo,
                   Relay angleControl)
    {
        m_latchReleaseServo = latchServo;
        m_currentState = SHOOTER_CONTROL_STATE_WAIT;
        m_encoder = encoder;
        m_pullBackSpeedController = pullBackSpeedController;
        m_angleControl = angleControl;
        m_speedControllerMaxRpm = speedControllerMaxRpm;
        m_limitSwitch = limitSwitch;
        m_pullBackEncoderRpm = new EncoderRPM();
        m_pullBackEncoderRpm.Init(m_pullBackSpeedController, m_encoder, (-1)*m_speedControllerMaxRpm, m_speedControllerMaxRpm, 0.05, 100, m_limitSwitch);
        m_releaseFromMidptEncoderRpm = new EncoderRPM();
        m_releaseFromMidptEncoderRpm.Init(m_pullBackSpeedController, m_encoder,(-1)*m_speedControllerMaxRpm/4,m_speedControllerMaxRpm,0.05, 3);
        m_gearControl = gearControl;
        m_latchReleased = false;
        m_gearReleased = false;
        m_isPulledBack = false;
    }
    
    
    public void SetState(byte state)
    {
        //Logger.PrintLine("ShooterEncoder: m_currentState = " + m_currentState + " m_isPulledBack = " + m_isPulledBack + " state =" + state, Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
        if ( (SHOOTER_CONTROL_STATE_WAIT == m_currentState) /*&& 
              ( (m_isPulledBack && (SHOOTER_CONTROL_STATE_PULLBACK != state)) ||
                (!m_isPulledBack && (SHOOTER_CONTROL_STATE_PULLBACK == state)) )*/
            )
        {
            m_currentState = state;
            
            Logger.PrintLine("ShooterEncoder: SetState = " + state, Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
            
            if (SHOOTER_CONTROL_STATE_PULLBACK == m_currentState)
            {
                // engage the gear
                m_gearControl.set(DoubleSolenoid.Value.kReverse);
                // change angle 
                m_angleControl.set(Relay.Value.kReverse);
                // put the latch in a state where it's free to latch
                m_latchReleaseServo.set(0.5);
                Timer.delay(0.3);
                m_pullBackEncoderRpm.Stop();
                m_pullBackEncoderRpm.Start();
                m_pullBackStartTime = Timer.getFPGATimestamp();
            }
            else if(SHOOTER_CONTROL_STATE_PERFORM_GOAL_SHOT == m_currentState )
            {
                // change angle 
                m_angleControl.set(Relay.Value.kForward);
                Timer.delay(0.3);
                m_latchReleased = false;
                m_gearReleased = false;
            }
            else if(SHOOTER_CONTROL_STATE_PERFORM_TRUSS_SHOT == m_currentState )
            {
                // change angle
                m_angleControl.set(Relay.Value.kReverse);
                // release the gear
                m_gearControl.set(DoubleSolenoid.Value.kForward);
                Timer.delay(0.3);
                m_releaseFromMidptEncoderRpm.Stop();
                m_releaseFromMidptEncoderRpm.Start();
                m_latchReleased = false;
            }
            else if(SHOOTER_CONTROL_STATE_SLOW_RELEASE == m_currentState)
            {
                // engage the gear
                m_gearControl.set(DoubleSolenoid.Value.kReverse);
                Timer.delay(0.3);
                m_pullBackStartTime = Timer.getFPGATimestamp();
            }
        }
    }
    
    
    public byte GetState()
    {
        return m_currentState;
    }
    
    public boolean IsPulledBack()
    {
        return m_isPulledBack;
    }
    
    public void PeriodicFunc()
    {
        System.out.println("shooter control PeriodicFunc m_currentState = " + m_currentState);
        if (SHOOTER_CONTROL_STATE_PULLBACK == m_currentState)
        {
            HandleStatePullback();
        }
        else if (SHOOTER_CONTROL_STATE_PERFORM_GOAL_SHOT == m_currentState)
        {
            HandleStateRelease();
        }
        else if (SHOOTER_CONTROL_STATE_PERFORM_TRUSS_SHOT == m_currentState)
        {
            HandleStateReleaseFromMidPoint();
        }
        else if (SHOOTER_CONTROL_STATE_SLOW_RELEASE == m_currentState)
        {
            HandleStateSlowRelease();
        }
    }
    
    private void HandleStatePullback()
    {
        double currentTime = Timer.getFPGATimestamp();

        // assume pullback should finish within 5 seconds
        if (m_pullBackEncoderRpm.isEnabled() && (currentTime - m_pullBackStartTime <= 5))
        {
            Logger.PrintLine("ShooterEncoder: calling m_pullBackEncoderRpm.PeriodicFunc", Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
            
            m_pullBackEncoderRpm.PeriodicFunc();
        }
        else 
        {
            m_pullBackEncoderRpm.Stop();
            if (m_pullBackEncoderRpm.isEnabled())
            {
                Logger.PrintLine("ShooterEncoder: HandleStatePullback Timed out, moving to slow release state", Logger.LOGGER_MESSAGE_LEVEL_ERROR);
            
                // the pullback ncoder rpm object is still enabled (i.e. the
                // limit switch did not fire and we timed out. As a safty measure
                // do a slow release
                m_currentState = SHOOTER_CONTROL_STATE_SLOW_RELEASE;
                // engage the gear
                m_gearControl.set(DoubleSolenoid.Value.kReverse);
                Timer.delay(0.3);
                m_latchReleased = false;
                m_pullBackStartTime = Timer.getFPGATimestamp();
            }
            else
            {
                // lock the latch
                m_latchReleaseServo.set(0);
                Timer.delay(0.3);
                m_isPulledBack = true;

                m_currentState = SHOOTER_CONTROL_STATE_WAIT;
            }
        }
    }
    
    private void HandleStateReleaseFromMidPoint()
    {
        if(!m_latchReleased)
        {
            // release the latch
            m_latchReleaseServo.set(1);
            m_latchReleased = true;
        }
        
        Timer.delay(0.4);
        
        //engage the gear
        m_gearControl.set(DoubleSolenoid.Value.kReverse);
        
        m_isPulledBack = false;
        m_currentState = SHOOTER_CONTROL_STATE_WAIT;
         
        /*if(m_releaseFromMidptEncoderRpm.isEnabled())
        {   
            m_releaseFromMidptEncoderRpm.PeriodicFunc();
        }
        else
        {
            if (!m_gearReleased)
            {
                // set the gear in neutral
                m_gearControl.set(DoubleSolenoid.Value.kForward);
                m_gearReleased = true;
            }
            m_isPulledBack = false;
            m_currentState = SHOOTER_CONTROL_STATE_WAIT;
        }*/
    }
    
    public void Reset()
    {
        m_currentState = SHOOTER_CONTROL_STATE_WAIT;
    }
    
    private void HandleStateRelease()
    {
        if (!m_gearReleased)
        {
            // set the gear in neutral
            m_gearControl.set(DoubleSolenoid.Value.kForward);
            m_gearReleased = true;
        }
        
        if(!m_latchReleased)
        {
            //release the latch
            m_latchReleaseServo.set(1);
            Timer.delay(0.5);
            m_latchReleased = true;
        }
   
        m_isPulledBack = false;
        m_currentState = SHOOTER_CONTROL_STATE_WAIT;
    }
    
    private void HandleStateSlowRelease()
    {
        double currentTime = Timer.getFPGATimestamp();
        
        // assume we should finish within 5  seconds
        if (currentTime - m_pullBackStartTime > 5)
        {
            m_isPulledBack = false;
            m_currentState = SHOOTER_CONTROL_STATE_WAIT;   
        }
    }
    
    public void ChangeAngle(boolean forward)
    {
        if (forward)
        {
           m_angleControl.set(Relay.Value.kForward);
        }
        else
        {
            m_angleControl.set(Relay.Value.kReverse);
        }
    }
    
    public static final byte SHOOTER_CONTROL_STATE_WAIT = 0;
    public static final byte SHOOTER_CONTROL_STATE_PULLBACK = 1;
    public static final byte SHOOTER_CONTROL_STATE_PERFORM_GOAL_SHOT = 2;
    public static final byte SHOOTER_CONTROL_STATE_PERFORM_TRUSS_SHOT = 3;
    public static final byte SHOOTER_CONTROL_STATE_SLOW_RELEASE = 4;
    
    private Servo m_latchReleaseServo;
    private byte m_currentState;
    private Encoder m_encoder;
    private SpeedController m_pullBackSpeedController;
    private EncoderRPM m_pullBackEncoderRpm;
    private double m_speedControllerMaxRpm;
    private DigitalInput m_limitSwitch;
    private EncoderRPM m_releaseFromMidptEncoderRpm;
    private boolean m_latchReleased;
    private boolean m_gearReleased;
    private double m_pullBackStartTime;
    private DoubleSolenoid m_gearControl;
    private Relay m_angleControl;
    private boolean m_isPulledBack;
    private double m_slowReleaseStartTime;
}
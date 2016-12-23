/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;

/**
 *
 * @author Cybernetics
 */
public class IntakeControl
{
    IntakeControl(SpeedController speedController, boolean speedControllerReversed,
                    DoubleSolenoid angleControl)
    {
        m_speedController = speedController;
        m_angleControl = angleControl;
        if (speedControllerReversed)
        {
            m_speedControllerDirectionMult = -1;
        }
        else
        {
            m_speedControllerDirectionMult = 1;
        }
    }

    public void SetSpin(double value)
    {
        m_speedController.set(value * m_speedControllerDirectionMult);
    }
    
    public void ChangeAngle()
    {
        if(m_angleControl.get() == DoubleSolenoid.Value.kReverse)
        {
            Logger.PrintLine("IntrakeControl.ChangeAngle kForward", Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
            m_angleControl.set(DoubleSolenoid.Value.kForward);
        }
        else
        {
            Logger.PrintLine("IntrakeControl.ChangeAngle kReverse", Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
            m_angleControl.set(DoubleSolenoid.Value.kReverse);
        }
    }
    public void Extract()
    {
        m_angleControl.set(DoubleSolenoid.Value.kForward);
    }
    public void Retract()
    {
        m_angleControl.set(DoubleSolenoid.Value.kReverse);
    }
    
    private SpeedController m_speedController;
    private int m_speedControllerDirectionMult;
    private DoubleSolenoid m_angleControl;
}

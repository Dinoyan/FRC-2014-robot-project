/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.SpeedController;

/**
 *
 * @author Cybernetics
 */
public class MultiSpeedController implements SpeedController
{
    MultiSpeedController(SpeedController speedController1, boolean reverse1,
                        SpeedController speedController2, boolean reverse2,
                        String name)
    {
        m_speedControllerArray = new SpeedController[2];
        m_speedControllerArray[0] = speedController1;
        m_speedControllerArray[1] = speedController2;

        m_name = name;
        
        m_speedControllerDirectionMultiplier = new int[2];
        if (reverse1)
        {
            m_speedControllerDirectionMultiplier[0] = -1;
        }
        else
        {
            m_speedControllerDirectionMultiplier [0] = 1;
        }
        
        if (reverse2)
        {
            m_speedControllerDirectionMultiplier[1] = -1;
        }
        else
        {
            m_speedControllerDirectionMultiplier [1] = 1;
        }
    }
    
    MultiSpeedController(SpeedController speedController1, boolean reverse1,
                            SpeedController speedController2, boolean reverse2,
                            SpeedController speedController3, boolean reverse3,
                            String name)
    {
        m_speedControllerArray = new SpeedController[3];
        m_speedControllerArray[0] = speedController1;
        m_speedControllerArray[1] = speedController2;
        m_speedControllerArray[2] = speedController3;
        m_name = name;
        
        m_speedControllerDirectionMultiplier = new int[3];
        if (reverse1)
        {
            m_speedControllerDirectionMultiplier[0] = -1;
        }
        else
        {
            m_speedControllerDirectionMultiplier [0] = 1;
        }
        
        if (reverse2)
        {
            m_speedControllerDirectionMultiplier[1] = -1;
        }
        else
        {
            m_speedControllerDirectionMultiplier [1] = 1;
        }
        
        if (reverse3)
        {
            m_speedControllerDirectionMultiplier[2] = -1;
        }
        else
        {
            m_speedControllerDirectionMultiplier [2] = 1;
        }
    }
    
    public void SetBias(boolean applyBias, boolean applyBiasWhenForward)
    {
        m_applyBias = applyBias;
        m_applyBiasWhenForward = applyBiasWhenForward;
    }
    
    public void DisableBias()
    {
        m_applyBias = false;
    }
    
    public void disable()
    {
        int i = 0;
        
        for (i = 0; i < m_speedControllerArray.length; i ++)
        {
            m_speedControllerArray[i].disable();
        }
    }
    
    public void pidWrite(double output)
    {
        int i = 0;
        
        // hack test
        if (m_applyBias)
        {
            
            if ((m_applyBiasWhenForward && (output > 0)) ||
               (!m_applyBiasWhenForward && (output < 0)))
            {
                //System.out.println("777");
                output *= HACK_MULTIPLIER;
            }
        }
        
        for (i = 0; i < m_speedControllerArray.length; i ++)
        {
            m_speedControllerArray[i].pidWrite(output * m_speedControllerDirectionMultiplier[i]);
        }
    }
    
    public void set(double speed)
    {
        int i = 0;
        
        // hack test
        if (m_applyBias)
        {
            
            if ((m_applyBiasWhenForward && (speed > 0)) ||
               (!m_applyBiasWhenForward && (speed < 0)))
            {
                //System.out.println("888");
                speed *= HACK_MULTIPLIER;
            }
        }
        
        for (i = 0; i < m_speedControllerArray.length; i ++)
        {
            m_speedControllerArray[i].set(speed * m_speedControllerDirectionMultiplier[i]);
        }
    }
    
    public void set(double speed, byte syncGroup)
    {
        int i = 0;
        
        //System.out.println("set " + m_name + " speed=" + speed);
        
        // hack test
        if (m_applyBias)
        {
            
            if ((m_applyBiasWhenForward && (speed > 0)) ||
               (!m_applyBiasWhenForward && (speed < 0)))
            {
                //System.out.println("999");
                speed *= HACK_MULTIPLIER;
            }
        }
        
        for (i = 0; i < m_speedControllerArray.length; i ++)
        {
            m_speedControllerArray[i].set(speed * m_speedControllerDirectionMultiplier[i], syncGroup);
        }
    }
    
    public double get()
    {
        double totalSpeed = 0;
        int i = 0;
        
        for (i = 0; i < m_speedControllerArray.length; i ++)
        {
            totalSpeed += m_speedControllerArray[i].get();
        }
        
        return (totalSpeed / m_speedControllerArray.length);
    }
    
    private SpeedController[] m_speedControllerArray;
    private int[] m_speedControllerDirectionMultiplier;
    private boolean m_applyBias;
    private boolean m_applyBiasWhenForward;
    private String m_name;
    
    private static final double HACK_MULTIPLIER = 0.97;
}

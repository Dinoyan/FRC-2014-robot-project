/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author Cybernetics
 */
public class EncoderAverager implements PIDSource
{
    Encoder enc1;
    Encoder enc2;
    
    public EncoderAverager(Encoder encoder1, Encoder encoder2)
    {
        enc1 = encoder1;
        enc2 = encoder2;
    }
    
    public void start()
    {
        enc1.start();
        enc2.start();
    }
    
    public void reset(){
        enc1.reset();
        enc2.reset();
    }
    
    public void stop()
    {
        enc1.stop();
        enc2.stop();
    }
    
    public double pidGet() 
    {
        double enc1Val = enc1.pidGet();
        double enc2Val = enc2.pidGet();
        
        double avg = (enc1Val + enc2Val)/2;
        SmartDashboard.putNumber("Speed", enc1.getRate());
        Logger.PrintLine("encoder: avg: " + avg + " enc1Val: " + enc1Val + " enc2Val: " + enc2Val, Logger.LOGGER_MESSAGE_LEVEL_INFO);
        return avg;
    }
    
}

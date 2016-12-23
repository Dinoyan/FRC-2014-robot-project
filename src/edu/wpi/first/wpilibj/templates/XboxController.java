/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Joystick;

/**
 * XboxController class is an extension of the Joystick class featuring deadbands for all the axes.
 * @author Cybernetics
 */
public class XboxController extends Joystick {
    private double deadbnd;
    public XboxController(int port, double deadband) {
        super(port);
        deadbnd = deadband;
    }
    
    public double getRawAxis(int axis){
        double value;
        value = super.getRawAxis(axis);
        if(value < deadbnd && value > -deadbnd)
            value = 0;
        else if(value>=deadbnd)
            value = (value-deadbnd)/(1-deadbnd);
        else
            value = (value+deadbnd)/(1-deadbnd);
        return value;
        
    }
    /**
     * Set the deadband on all axes.
     * @param deadband Deadband value.
     */
    public void setDeadband(double deadband){
        deadbnd = deadband;
    }
    /**
     * Get the current deadband value.
     * @return Current deadband value.
     */
    public double getDeadband(){
        return deadbnd;
    }
}

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 * @author Cybernetics
 */
public class RobotDrivePID extends RobotDrive implements PIDOutput {
    private String getAngle;
    //RobotDrive constructors
    public RobotDrivePID(int leftMotorChannel, int rightMotorChannel,Gyro gyro) {   
        super(leftMotorChannel, rightMotorChannel);
        InitGyro(gyro);
        m_callCounter = 0;
    }
    
    public RobotDrivePID(int frontLeftMotor, int rearLeftMotor, int frontRightMotor, int rearRightMotor,Gyro gyro) {
        super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
        InitGyro(gyro);
        m_callCounter = 0;
    }
    public RobotDrivePID(SpeedController leftMotor, SpeedController rightMotor,Gyro gyro) {
        super(leftMotor, rightMotor);
        InitGyro(gyro);
        m_callCounter = 0;
    }
    public RobotDrivePID(SpeedController frontLeftMotor, SpeedController rearLeftMotor, SpeedController frontRightMotor, SpeedController rearRightMotor,Gyro gyro) {
        super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
        InitGyro(gyro);
        m_callCounter = 0;
    }
    
    private void InitGyro(Gyro gyro)
    {
        m_gyro = gyro;
        m_gyro.setSensitivity(0.007);
    }
    
    //PIDOutput methods
    public void pidWrite(double output) {
        double gyroAngle = m_gyro.getAngle();

        Logger.PrintLine("RobotDrivePID::pidWrite : output = " + output + " gyroAngle = " + gyroAngle, Logger.LOGGER_MESSAGE_LEVEL_ERROR);
        if (output < 0)
        {
            gyroAngle *= (-1);
            
        }
        
        if (m_callCounter%2 != 0)
        {
            gyroAngle = 0;
        }
        
        this.drive(output, gyroAngle * GYRO_ANGLE_MULTIPLIER);
        
        m_callCounter++;
        
        //this.drive(output, 0);
        //System.out.println("RobotDrivePID::pidWrite : output = " + output);      
    }
    
    public void resetGyro()
    {
        m_gyro.reset();
        m_callCounter = 0;
    }
    
    private static final double GYRO_ANGLE_MULTIPLIER = 0.00003;//0.03;
    
    private Gyro m_gyro;
    private int m_callCounter;
}

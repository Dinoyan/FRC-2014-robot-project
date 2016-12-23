/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.camera.AxisCamera;

/**
 *
 * @author Cybernetics
 */
public class AutonomousModeHandler {
    
    public AutonomousModeHandler(Encoder enc1, Encoder enc2, RobotDrivePID robotoDrive, AxisCamera cam, ConfigurableValues configurableValues,
                                 IntakeControl frontIntake, IntakeControl backIntake, ShooterControl shooterControl, DigitalInput frontIntakeArmAngleDetector)
    {
        m_configurableValues = configurableValues;
        m_shooterControl = shooterControl;
        m_robotDrivePid = robotoDrive;
        m_frontIntake = frontIntake;
        m_backIntake = backIntake;
        m_frontIntakeArmAngleDetector = frontIntakeArmAngleDetector;
        m_driveEncoder1 = enc1;
        m_driveEncoder2 = enc2;
        m_encoderAverager = new EncoderAverager(m_driveEncoder1, m_driveEncoder2);
 
        m_autonomousImageDetector = new AutonomousImageDetector(cam,m_configurableValues);
        m_nextStateArray= new byte[256];
        m_motorBrake = new MotorBrake();
        
        SetCurrentState(AUTONOMOUS_HANDLER_STATE_WAIT);
        SetNextStateArray(AUTONOMOUS_MODE_1_BALL_SHOOTING);
        
        m_overrideCoefficients = false;
        
        m_pidController = null;
        
        m_disabled = true;
        m_shootingBall = false;
        m_driving = false;
        m_braking = false;
        m_detectingImage = false;
        m_currentStateIndex = 0;
        m_loadingBall = false;
        m_shooterPullingBack = false;
    }
    
    public void SetNextStateArray(byte mode)
    {
        byte stateCounter = 0;
       if(AUTONOMOUS_MODE_1_BALL_SHOOTING == mode )  
       {
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_FRONT_INTAKE_EXTRACT;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_IMAGE_DETECTION;
           stateCounter++;
           m_nextStateArray[stateCounter]  = AUTONOMOUS_HANDLER_STATE_DRIVE_FORWARD;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_BRAKE_ROBOT;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_WAIT_FOR_HOT_GOAL;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_SHOOT_BALL;
           stateCounter++;
           //m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_FRONT_INTAKE_RETRACT;
           //stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_DRIVE_BACK;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_BRAKE_ROBOT;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_WAIT;
       }
       else if (AUTONOMOUS_MODE_2_BALL_SHOOTING == mode)
       {
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_BACK_INTAKE_EXTRACT;
           stateCounter ++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_BACK_INTAKE_LOAD_BALL_HALFWAY;
           stateCounter++;
           m_nextStateArray[stateCounter]  = AUTONOMOUS_HANDLER_STATE_DRIVE_FORWARD;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_BRAKE_ROBOT;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_FRONT_INTAKE_EXTRACT;
           stateCounter ++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_SHOOT_BALL;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_SHOOTER_PULLBACK;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_BACK_INTAKE_RETRACT;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_BACK_INTAKE_LOAD_BALL_HALFWAY;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_SHOOT_BALL;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_DRIVE_BACK;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_BRAKE_ROBOT;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_WAIT; 
       }
       else if (AUTONOMOUS_MODE_3_BALL_SHOOTING == mode)
       {
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_BACK_INTAKE_EXTRACT;
           stateCounter ++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_FRONT_INTAKE_EXTRACT;
           stateCounter ++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_BOTH_INTAKES_LOAD_BALL_HALFWAY;
           stateCounter++;
           m_nextStateArray[stateCounter]  = AUTONOMOUS_HANDLER_STATE_DRIVE_FORWARD;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_BRAKE_ROBOT;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_SHOOT_BALL;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_SHOOTER_PULLBACK;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_BACK_INTAKE_RETRACT;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_BACK_INTAKE_LOAD_BALL_HALFWAY;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_SHOOT_BALL;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_SHOOTER_PULLBACK;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_FRONT_INTAKE_RETRACT;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_FRONT_INTAKE_LOAD_BALL_HALFWAY;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_FRONT_INTAKE_EXTRACT;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_SHOOT_BALL;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_DRIVE_BACK;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_BRAKE_ROBOT;
           stateCounter++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_WAIT; 
       }
       else if(AUTONOMOUS_MODE_DRIVE_FORWARD == mode)
       {
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_DRIVE_FORWARD;
           stateCounter ++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_BRAKE_ROBOT;
           stateCounter ++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_WAIT;
       }
       else if(AUTONOMOUS_MODE_DRIVE_BACK == mode)
       {
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_DRIVE_BACK;
           stateCounter ++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_BRAKE_ROBOT;
           stateCounter ++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_WAIT;
       }
       else if(AUTONOMOUS_MODE_DRIVE_BOTH_DIRECTIONS == mode)
       {
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_DRIVE_FORWARD;
           stateCounter ++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_BRAKE_ROBOT;
           stateCounter ++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_DRIVE_BACK;
           stateCounter ++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_BRAKE_ROBOT;
           stateCounter ++;
           m_nextStateArray[stateCounter] = AUTONOMOUS_HANDLER_STATE_WAIT;
       }
    }
    

    
    
    public PIDController GetPidController()
    {
        return m_pidController;
    }
    
    public void OverrideCoefficients(double kp, double ki, double kd)
    {
        m_Kp = kp;
        m_Ki = ki;
        m_Kd = kd;
        
        m_overrideCoefficients = true;
    }
    
    public void Disable()
    {
        m_disabled = true;
        Logger.PrintLine("Disable 1",Logger.LOGGER_MESSAGE_LEVEL_INFO);
        if (null != m_pidController)
        {
            m_pidController.disable();
            m_encoderAverager.reset();
            m_pidController.reset();
        }
        
        Logger.PrintLine("Disable 2",Logger.LOGGER_MESSAGE_LEVEL_INFO);
        
         SetCurrentState(AUTONOMOUS_HANDLER_STATE_WAIT);
    }

    public void Init()
    {
        Logger.PrintLine("Init 1",Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
        
        m_pidController = new PIDController(10,10,10,m_encoderAverager,m_robotDrivePid);
        m_pidController.setOutputRange(-0.8,0.8);
        //m_pidController.setOutputRange(-0.4,0.4);
        Logger.PrintLine("Init 4",Logger.LOGGER_MESSAGE_LEVEL_ERROR);

        m_currentStateIndex = 0;
        SetCurrentState(m_nextStateArray[m_currentStateIndex]);
        m_disabled = false;
        
        m_shootingBall = false;
        m_driving = false;
        m_braking = false;
        m_detectingImage = false;
        m_currentStateIndex = 0;
        m_loadingBall = false;
        m_shooterPullingBack = false;
        
        m_autonomousStartTime = Timer.getFPGATimestamp();
        
        m_robotDrivePid.resetGyro();
    }
    
    public void PeriodicActionHandler()
    {
        if (!m_disabled)
        {
            if (AUTONOMOUS_HANDLER_STATE_DRIVE_FORWARD == m_currentState)
            {
                HandleDriveForward();
            }
            else if (AUTONOMOUS_HANDLER_STATE_IMAGE_DETECTION == m_currentState)
            {
                HandleImageDetection();
            }
            else if (AUTONOMOUS_HANDLER_STATE_SHOOT_BALL == m_currentState)
            {
                HandleShootBall();
            }
            else if (AUTONOMOUS_HANDLER_STATE_DRIVE_BACK == m_currentState)
            {
                HandleDriveBack();
            }
            else if (AUTONOMOUS_HANDLER_STATE_BRAKE_ROBOT == m_currentState)
            {
                HandleBrakeRobot();
            }
            else if (AUTONOMOUS_HANDLER_STATE_WAIT_FOR_HOT_GOAL == m_currentState)
            {
                HandleWaitForHotGoal();
            }
            else if (AUTONOMOUS_HANDLER_STATE_FRONT_INTAKE_EXTRACT == m_currentState)
            {
                HandleIntakeExtract(true);
            }
            else if(AUTONOMOUS_HANDLER_STATE_FRONT_INTAKE_RETRACT == m_currentState)
            {
                HandleIntakeRetract(true);
            }
            else if (AUTONOMOUS_HANDLER_STATE_BACK_INTAKE_EXTRACT == m_currentState)
            {
                HandleIntakeExtract(false);
            }
            else if(AUTONOMOUS_HANDLER_STATE_BACK_INTAKE_RETRACT == m_currentState)
            {
                HandleIntakeRetract(false);
            }
            else if (AUTONOMOUS_HANDLER_STATE_BACK_INTAKE_LOAD_BALL_HALFWAY == m_currentState)
            {
                HandleLoadBallHalfway(false);
            }
            else if (AUTONOMOUS_HANDLER_STATE_BOTH_INTAKES_LOAD_BALL_HALFWAY == m_currentState)
            {
                HandleLoadBallHalfway(true);
                HandleLoadBallHalfway(false);
            }
            else if (AUTONOMOUS_HANDLER_STATE_SHOOTER_PULLBACK == m_currentState)
            {
                HandleShooterPullback();
            }
        }
    }
    
    private void HandleLoadBallHalfway(boolean frontIntake)
    {
        IntakeControl intakeControl;
        
        if (frontIntake)
        {
            intakeControl = m_frontIntake;
        }
        else
        {
            intakeControl = m_backIntake;
        }
        
        if (!m_loadingBall)
        {
            m_ballLoadingStartTime = Timer.getFPGATimestamp();
            m_loadingBall = true;
        }
        
        // load the ball for 3 seconds
        if ((Timer.getFPGATimestamp() - m_ballLoadingStartTime) < 3)
        {
            intakeControl.SetSpin(-1);
        }
        else
        {
            // done loading the ball
            m_loadingBall = false;
            // When done move to next state
            m_currentStateIndex ++;
            SetCurrentState(m_nextStateArray[m_currentStateIndex]);
        }
    }
    
    private void HandleIntakeRetract(boolean frontIntake)
    {
        if (frontIntake)
        {
            m_frontIntake.Retract();    
        }
        else
        {
            m_backIntake.Retract();
        }
        Timer.delay(0.5);
        // When done move to next state
        m_currentStateIndex ++;
        SetCurrentState(m_nextStateArray[m_currentStateIndex]);
    }
    
    private void HandleIntakeExtract(boolean frontIntake)
    {
        if (frontIntake)
        {
            m_frontIntake.Extract();
        }
        else
        {
            m_backIntake.Extract();
            
        }
    
        Timer.delay(0.5);
       // When done move to next state
        m_currentStateIndex ++;
       SetCurrentState(m_nextStateArray[m_currentStateIndex]);
    }
    private void HandleWaitForHotGoal()
    {
        if ((m_autonomousImageDetector.CurrentDetectionState() != AutonomousImageDetector.DETECTION_STATE_GOAL_UNLIT) ||
                ((m_autonomousImageDetector.CurrentDetectionState() == AutonomousImageDetector.DETECTION_STATE_GOAL_UNLIT) &&
                 ((Timer.getFPGATimestamp() - m_autonomousStartTime) > 5.0)) 
           )
        {
            // When done move to next state
            m_currentStateIndex ++;
            SetCurrentState(m_nextStateArray[m_currentStateIndex]);
        }
    }
    
    
    private void HandleBrakeRobot()
    {
        if (!m_braking)
        {
             m_motorBrake.Init(m_robotDrivePid, m_driveEncoder1);
             m_motorBrake.Start();
             m_braking = true;
        }
        
        m_motorBrake.PeriodicFunc();
                
        if (!m_motorBrake.isEnabled())
        {
            m_motorBrake.Stop();
            m_braking = false;
        }
        
        if (!m_braking)
        {
            // When done move to next state
            m_currentStateIndex ++;
            SetCurrentState(m_nextStateArray[m_currentStateIndex]);
        }
    }
    
    private void HandleDriveForward()
    {
        if (!m_driving)
        {
            m_driveDistance = m_configurableValues.m_autonomousDriveForwardDistance;
            
            if (!m_overrideCoefficients)
            {
                m_Kp = m_configurableValues.m_autonomousDriveForwardPidControllerP;//115//1000;
                m_Ki = m_configurableValues.m_autonomousDriveForwardPidControllerI;//0
                m_Kd = m_configurableValues.m_autonomousDriveForwardPidControllerD;//60//500;
            }
            m_driveTolerance = m_configurableValues.m_autonomousDriveForwardPidControllerTolerance;//6//40;
        }
        
        Logger.PrintLine("calling HandleDrive m_driving = " + m_driving,Logger.LOGGER_MESSAGE_LEVEL_DEBUG );
        
        HandleDrive();
        
        Logger.PrintLine("after HandleDrive m_driving = " + m_driving,Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
         
        if (!m_driving)
        {
            // When done move to next state
            m_currentStateIndex ++;
             SetCurrentState(m_nextStateArray[m_currentStateIndex]);
        }
    }
    
    private void HandleDriveBack()
    {
        if (!m_driving)
        {
            m_driveDistance = m_configurableValues.m_autonomousDriveBackDistance;//-1000;
            if (!m_overrideCoefficients)
            {
                m_Kp = m_configurableValues.m_autonomousDriveBackPidControllerP;//1000;
                m_Ki = m_configurableValues.m_autonomousDriveBackPidControllerI;
                m_Kd = m_configurableValues.m_autonomousDriveBackPidControllerD;//700;
            }
            m_driveTolerance = m_configurableValues.m_autonomousDriveBackPidControllerTolerance;//20;
        }
        
        HandleDrive();
         
        if (!m_driving)
        {
            // When done move to next state
            m_currentStateIndex ++;
            SetCurrentState(m_nextStateArray[m_currentStateIndex]);
        }
    }
    
    private void HandleDrive()
    {
        Logger.PrintLine("HandleDrive: m_driving: " + m_driving, Logger.LOGGER_MESSAGE_LEVEL_ERROR);
        if (!m_driving)
        {
            Logger.PrintLine("HandleDrive: m_driveTolerance: " + m_driveTolerance,Logger.LOGGER_MESSAGE_LEVEL_ERROR);
            
            m_pidController.setSetpoint(m_driveDistance);
            m_pidController.setPID(m_Kp, m_Ki, m_Kd);
            m_pidController.setAbsoluteTolerance(m_driveTolerance);
            //m_robotDrivePid.setExpiration(0.5);
            //m_robotDrivePid.setSafetyEnabled(false);

            //m_robotDrivePid.resetGyro();
            
            m_encoderAverager.reset();
            m_encoderAverager.start();
            m_pidController.enable();
            
            m_driving = true;
        }
        else
        {
            Logger.PrintLine("HandleDrive: pid ctrl getError: " + m_pidController.getError(),Logger.LOGGER_MESSAGE_LEVEL_ERROR);
            if(m_pidController.onTarget())
            {
                m_pidController.disable();
                m_encoderAverager.reset();
                m_pidController.reset();
                
                m_driving = false;
            }
        }
    }
    

    private void HandleImageDetection()
    {
        double timeDiff = 0;
                
        if (!m_detectingImage)
        {
            m_detectingImage = true;
            m_autonomousImageDetector.Init();
            m_imageDetectionStartTime = Timer.getFPGATimestamp();
        }

        m_autonomousImageDetector.PeriodicFunc();
        
        timeDiff = Timer.getFPGATimestamp() - m_imageDetectionStartTime;
                
        if ( ( timeDiff >= 1.5) ||
             (m_autonomousImageDetector.CurrentDetectionState() != AutonomousImageDetector.DETECTION_STATE_DETECTING))
        {
            Logger.PrintLine("AutonomousModeHandler::HandleImageDetection: move to next state. CurrentDetectionState= " + m_autonomousImageDetector.CurrentDetectionState() + " timediff = " + timeDiff,Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
            
            m_currentStateIndex ++;
            SetCurrentState(m_nextStateArray[m_currentStateIndex]);
            
            m_detectingImage = false;
        }
        
    }
    
    private void SetCurrentState(byte state)
    {
        Logger.PrintLine("AutonomousModeHandler::SetCurrentState: m_currentState= " + m_currentState + "State= " + state,Logger.LOGGER_MESSAGE_LEVEL_INFO);
        m_currentState = state;
        
    }
            
    private void HandleShooterPullback()
    {
        if(!m_shooterPullingBack)
        {
            m_shooterControl.SetState(ShooterControl.SHOOTER_CONTROL_STATE_PULLBACK);
            m_shooterPullingBack = true;
        }
        m_shooterControl.PeriodicFunc();
        if(m_shooterControl.GetState()==ShooterControl.SHOOTER_CONTROL_STATE_WAIT)
        {
            m_shooterPullingBack = false;
            // When done move to next state
            m_currentStateIndex ++;
            SetCurrentState(m_nextStateArray[m_currentStateIndex]);
        }
    }
    
    private void HandleShootBall()
    {
        boolean frontArmExtended = true;
        /*if (m_frontIntakeArmAngleDetector.get())
        {
            frontArmExtended = false;
        }*/
        
        // Only shoot if the front arm is extended, otherwise we
        // could break the arm
        if (frontArmExtended)
        {
            if(!m_shootingBall)
            {
                m_shooterControl.SetState(ShooterControl.SHOOTER_CONTROL_STATE_PERFORM_GOAL_SHOT);
                m_shootingBall = true;
            }
            m_shooterControl.PeriodicFunc();
            if(m_shooterControl.GetState() == ShooterControl.SHOOTER_CONTROL_STATE_WAIT)
            {
                m_shootingBall = false;
                // When done move to next state
                m_currentStateIndex ++;
                SetCurrentState(m_nextStateArray[m_currentStateIndex]);
            }
        }
        else
        {
            Logger.PrintLine("AutonomousHandler HandleShootBall: error: front intake arm not extended", Logger.LOGGER_MESSAGE_LEVEL_ERROR);
            SetCurrentState(AUTONOMOUS_HANDLER_STATE_WAIT);
        }
    }
    
    
    
    private static final byte AUTONOMOUS_HANDLER_STATE_UNDEFINED = 0;
    private static final byte AUTONOMOUS_HANDLER_STATE_WAIT = 1;
    private static final byte AUTONOMOUS_HANDLER_STATE_DRIVE_FORWARD = 2;
    private static final byte AUTONOMOUS_HANDLER_STATE_IMAGE_DETECTION = 3;
    private static final byte AUTONOMOUS_HANDLER_STATE_SHOOT_BALL = 4;
    private static final byte AUTONOMOUS_HANDLER_STATE_DRIVE_BACK = 5;
    private static final byte AUTONOMOUS_HANDLER_STATE_BRAKE_ROBOT  = 6;
    private static final byte AUTONOMOUS_HANDLER_STATE_WAIT_FOR_HOT_GOAL = 7;
    private static final byte AUTONOMOUS_HANDLER_STATE_FRONT_INTAKE_EXTRACT = 8;
    private static final byte AUTONOMOUS_HANDLER_STATE_FRONT_INTAKE_RETRACT = 9;
    private static final byte AUTONOMOUS_HANDLER_STATE_BACK_INTAKE_EXTRACT = 10;
    private static final byte AUTONOMOUS_HANDLER_STATE_BACK_INTAKE_RETRACT = 11;
    private static final byte AUTONOMOUS_HANDLER_STATE_FRONT_INTAKE_LOAD_BALL_HALFWAY = 12;
    private static final byte AUTONOMOUS_HANDLER_STATE_BACK_INTAKE_LOAD_BALL_HALFWAY = 13;
    private static final byte AUTONOMOUS_HANDLER_STATE_SHOOTER_PULLBACK = 14;
    private static final byte AUTONOMOUS_HANDLER_STATE_BOTH_INTAKES_LOAD_BALL_HALFWAY = 15;
    
    
    private static final byte AUTONOMOUS_HANDLER_STATE_MAX  = (byte)255;
    
    
    private static final byte AUTONOMOUS_MODE_1_BALL_SHOOTING = 1;
    private static final byte AUTONOMOUS_MODE_2_BALL_SHOOTING = 2;
    private static final byte AUTONOMOUS_MODE_3_BALL_SHOOTING = 3;
    private static final byte AUTONOMOUS_MODE_DRIVE_FORWARD = 4;
    private static final byte AUTONOMOUS_MODE_DRIVE_BACK = 5;
    private static final byte AUTONOMOUS_MODE_DRIVE_BOTH_DIRECTIONS = 6;
    

    private byte m_currentState;
    private byte m_currentStateIndex;

    private boolean m_driving;
    private boolean m_braking;
    
    private double m_driveDistance;
    private double m_Kp;
    private double m_Ki;
    private double m_Kd;
    private double m_driveTolerance;
    
    private ShooterControl m_shooterControl;
    private boolean m_shootingBall;
    private boolean m_overrideCoefficients;
    private boolean m_disabled;
    private boolean m_detectingImage;
    private double m_imageDetectionStartTime;
    private double m_autonomousStartTime;
    private final RobotDrivePID m_robotDrivePid;
    private final Encoder m_driveEncoder1;
    private final Encoder m_driveEncoder2;
    private final EncoderAverager m_encoderAverager;
    private PIDController m_pidController;
    //public static PIDController m_pidController;
    private final AutonomousImageDetector m_autonomousImageDetector;
    
    private final MotorBrake m_motorBrake;
    private final ConfigurableValues m_configurableValues;
    private byte[] m_nextStateArray;
    private IntakeControl m_frontIntake;
    private IntakeControl m_backIntake;
    
    private boolean m_loadingBall;
    private double m_ballLoadingStartTime;
    
    private boolean m_shooterPullingBack;
    private DigitalInput m_frontIntakeArmAngleDetector;
}

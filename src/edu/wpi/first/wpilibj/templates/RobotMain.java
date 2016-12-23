/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotMain extends IterativeRobot {  
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
        SmartDashboard.putString("Label1", "Code:");
        SmartDashboard.putString("Label2", "Deploy Date:");
        Logger.SetLoggingLevel(Logger.LOGGER_MESSAGE_LEVEL_ALL);
        m_rampControl = new Relay(3);
        m_configurableValues = new ConfigurableValues();
        m_compressor = new Compressor(5,1);
        m_driveStick = new XboxController(1,0.2);
        m_shootStick = new XboxController(2,0.2);
        m_rightDriveSpeedController = new MultiSpeedController(new Talon(4), false, new Talon(5), false, new Talon(6), false, "right");
        m_leftDriveSpeedController = new MultiSpeedController(new Talon(1), false, new Talon(2), false, new Talon(3), false, "left");
        m_latchServo = new Servo(9);
        m_frontIntakeArmAngleDetector = new DigitalInput(12);
        
        m_rightDriveSpeedController.SetBias(true, false);
        //m_leftDriveSpeedController.SetBias(false, true);
        m_gyro = new Gyro(1);
        m_robotDrivePid = new RobotDrivePID(m_leftDriveSpeedController, m_rightDriveSpeedController,m_gyro);
        m_driveRightEncoder = new Encoder(1,3,1,4,false,CounterBase.EncodingType.k2X);
        m_driveLeftEncoder = new Encoder(1,1,1,2,true,CounterBase.EncodingType.k2X);
        m_shooterMotor = new Jaguar(10);
        m_shooterMotor.setExpiration(0.2);
        m_shooterLimitSwitch = new DigitalInput(8);
        m_frontCamera = AxisCamera.getInstance("10.9.7.11"); 
        m_shooterEncoder = new Encoder(1,10,1,11,true,CounterBase.EncodingType.k2X);
        m_shooterEncoder.setDistancePerPulse(1d/(256d*3d));
        m_shooterEncoder.setPIDSourceParameter(Encoder.PIDSourceParameter.kDistance);
        m_shooterEncoder.setMinRate(0.0000001);
        m_shooterEncoder.setMaxPeriod(100000000);
        m_shooterGearControl = new DoubleSolenoid(7,8);
        m_shooterAngleControl = new Relay(2);
        m_shooterControl = new ShooterControl(m_shooterEncoder,
                                                m_shooterMotor,
                                                5000,
                                                m_shooterLimitSwitch,
                                                m_shooterGearControl,
                                                m_latchServo,
                                                m_shooterAngleControl);
        m_driveGearControl = new DoubleSolenoid(1,2);
        m_frontIntakeWheelControl = new Talon(7);
        m_backIntakeWheelControl = new Talon(8);
        m_frontIntakeAngleControl = new DoubleSolenoid(5,6);
        m_backIntakeAngleControl = new DoubleSolenoid(3,4);
        m_frontIntakeControl = new IntakeControl(m_frontIntakeWheelControl,false,m_frontIntakeAngleControl);
        m_backIntakeControl = new IntakeControl(m_backIntakeWheelControl,false,m_backIntakeAngleControl);
        m_autonomousModeHandler = new AutonomousModeHandler(m_driveLeftEncoder, m_driveRightEncoder, m_robotDrivePid,
                                                            m_frontCamera, m_configurableValues, m_frontIntakeControl,
                                                            m_backIntakeControl, m_shooterControl, m_frontIntakeArmAngleDetector);

        //m_driveLeftEncoder.setDistancePerPulse(8d*Math.PI/256d);
        m_driveLeftEncoder.setDistancePerPulse(4.125d*Math.PI/(256d*5.5d));
        m_driveLeftEncoder.setPIDSourceParameter(Encoder.PIDSourceParameter.kDistance);
        m_driveLeftEncoder.setMinRate(0.0000001);
        m_driveLeftEncoder.setMaxPeriod(100000000);
        
        //m_driveRightEncoder.setDistancePerPulse(8d*Math.PI/256d);
        m_driveRightEncoder.setDistancePerPulse(4.125d*Math.PI/(256d*5.5d));
        m_driveRightEncoder.setPIDSourceParameter(Encoder.PIDSourceParameter.kDistance);
        m_driveRightEncoder.setMinRate(0.0000001);
        m_driveRightEncoder.setMaxPeriod(100000000);
        
        m_driverStation = DriverStation.getInstance();
        
        // Intake angle - intakes are folded
        m_frontIntakeAngleControl.set(DoubleSolenoid.Value.kReverse);
        m_frontIntakeAngleControl.set(DoubleSolenoid.Value.kForward);
        m_frontIntakeAngleControl.set(DoubleSolenoid.Value.kReverse);
        
        m_backIntakeAngleControl.set(DoubleSolenoid.Value.kReverse);
        m_backIntakeAngleControl.set(DoubleSolenoid.Value.kForward);
        m_backIntakeAngleControl.set(DoubleSolenoid.Value.kReverse);
        
        m_rampControl.set(Relay.Value.kReverse);
        m_shooterAngleControl.set(Relay.Value.kReverse);
        m_latchServo.set(0.5);
        
        m_compressor.start();

    }
    
    public void autonomousInit()
    {
        // high drive gear be default
        /*m_driveGearControl.set(DoubleSolenoid.Value.kReverse);
        m_rampControl.set(Relay.Value.kForward);
        //m_rightDriveSpeedController.DisableBias();
        //m_leftDriveSpeedController.DisableBias();*/
        m_autonomousModeHandler.Init();
    }
    
    public void teleopInit()
    {
        // high drive gear be default
        /*m_driveGearControl.set(DoubleSolenoid.Value.kReverse);
        m_rightDriveSpeedController.SetBias(true, false);
        //m_leftDriveSpeedController.SetBias(false, true);
        
        // Intake angle - intakes are folded
        m_frontIntakeAngleControl.set(DoubleSolenoid.Value.kReverse);
        m_backIntakeAngleControl.set(DoubleSolenoid.Value.kReverse);
        
        // high drive gear be default
        m_driveGearControl.set(DoubleSolenoid.Value.kReverse);
        
        // forward shooter angle
        m_shooterControl.ChangeAngle(true);*/
        m_shooterControl.Reset();
        m_latchServo.set(0.5);
    }
    
    public void disabledInit()
    {
        if (null != m_autonomousModeHandler)
        {
            m_autonomousModeHandler.Disable();
            
        }
        
        m_frontIntakeChangeAngleReleased = true;
        m_backIntakeChangeAngleReleased = true;
        m_flippereChangeAngleReleased = true;
        
        
        
        //m_shooterControl.SetState(ShooterControl.SHOOTER_CONTROL_STATE_PULLBACK);
        
    }
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic()
    {
        this.upateDashboard();
        m_autonomousModeHandler.PeriodicActionHandler();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic()
    {
        this.upateDashboard();
        
        if(isEnabled())
        {
            ///-------------------Drive---------------------------
            m_robotDrivePid.arcadeDrive(m_driveStick.getRawAxis(2),m_driveStick.getRawAxis(4));
            
            //----------------------------------------------------
            //----------------Control drive gear------------------
            if(m_driveStick.getRawAxis(3)>= 0.85)
            {
                //low gear
                m_driveGearControl.set(DoubleSolenoid.Value.kForward);
            }
            else if(m_driveStick.getRawButton(5))
            {
                //high gear
                m_driveGearControl.set(DoubleSolenoid.Value.kReverse);
            }
            //----------------------------------------------------
            //-------------------control intake angle-------------
            if(m_shootStick.getRawButton(7) && m_frontIntakeChangeAngleReleased)
            {
                Logger.PrintLine("m_frontIntakeControl.ChangeAngle", Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
                m_frontIntakeControl.ChangeAngle();
                m_frontIntakeChangeAngleReleased = false;
            }
            else if (!m_shootStick.getRawButton(7))
            {
                m_frontIntakeChangeAngleReleased = true;
            }

            if(m_shootStick.getRawButton(8) && m_backIntakeChangeAngleReleased)
            {
                Logger.PrintLine("m_backIntakeControl.ChangeAngle", Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
                m_backIntakeControl.ChangeAngle();
                m_backIntakeChangeAngleReleased = false;
            }
            else if (!m_shootStick.getRawButton(8))
            {
                m_backIntakeChangeAngleReleased = true;
            }

            //----------------------------------------------------
            //--------------flipper control-----------------------
            if(m_shootStick.getRawAxis(6) == 1)
            {
                m_rampControl.set(Relay.Value.kForward);
            }
            else if(m_shootStick.getRawAxis(6) == -1)
            {
                m_rampControl.set(Relay.Value.kReverse);
            }

            //----------------------------------------------------
            //--------------shooter control-----------------------
            if(m_shootStick.getRawButton(1))
            {
                Logger.PrintLine("got m_shootStick button 6 right", Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
                // control shooter pullback
                m_shooterControl.SetState(ShooterControl.SHOOTER_CONTROL_STATE_PULLBACK);
            }
            else if(m_shootStick.getRawButton(2))
            {
                Logger.PrintLine("got m_shootStick button 2", Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
                // control shooter release
                m_shooterControl.SetState(ShooterControl.SHOOTER_CONTROL_STATE_PERFORM_GOAL_SHOT);
            }
            else if(m_shootStick.getRawButton(3))
            {
                Logger.PrintLine("got m_shootStick button 6 left", Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
                // control shooter release from midpoint
                m_shooterControl.SetState(ShooterControl.SHOOTER_CONTROL_STATE_PERFORM_TRUSS_SHOT);
            }
            
            // Call the shooter control's periodic function
            // to perform the operations related to its current state
            m_shooterControl.PeriodicFunc();
            
            //-------------------------------------------------------
            //--------------control intake spin----------------------
            m_frontIntakeControl.SetSpin(m_shootStick.getRawAxis(2));
            m_backIntakeControl.SetSpin(m_shootStick.getRawAxis(5));
            
            //-------------------------------------------------------
            //--------------------Drive------------------------------
            m_robotDrivePid.arcadeDrive(m_driveStick.getRawAxis(2),m_driveStick.getRawAxis(4));
            //-------------------------------------------------------
        }
    }
    
    public void testInit()
    {
        TestModeUpateDashboard();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic()
    {
        if (isEnabled())
        {
            if(m_driveStick.getRawButton(1))
            {
                TestModeReadDashboard();
            }
            
            m_shooterMotor.set(-m_shootStick.getRawAxis(2));
            
            System.out.println("m_shootStick axis2 value = " + -m_shootStick.getRawAxis(2));
            System.out.println("m_shooterEncoder.getDistance = " + m_shooterEncoder.getDistance());
            System.out.println("m_shooterLimitSwitch = " + m_shooterLimitSwitch.get());
            
            
            if(m_shootStick.getRawButton(2))
            {
                System.out.println("set shooter servo to 0 (lock)");
                m_latchServo.set(0);
            }
            else if(m_shootStick.getRawButton(3))
            {
                System.out.println("set shooter servo to 0.5 (latch)");
                m_latchServo.set(0.5);
            }
            
            else if(m_shootStick.getRawButton(4))
            {
                System.out.println("set shooter servo to 1 (open)");
                m_latchServo.set(1);
            }
            
            if(m_shootStick.getRawButton(5))
            {
                System.out.println("m_shooterGearControl.setDirection forward (neutral)");
                m_shooterGearControl.set(DoubleSolenoid.Value.kForward);
            }
            else if (m_shootStick.getRawButton(6))
            {
                System.out.println("m_shooterGearControl.setDirection reverse (engaged)");
                m_shooterGearControl.set(DoubleSolenoid.Value.kReverse);
            }
            
            if(m_shootStick.getRawButton(7))
            {
                System.out.println("m_shooterAngle.set forward (goal)");
                m_shooterAngleControl.set(Relay.Value.kForward);
            }
            else if (m_shootStick.getRawButton(8))
            {
                System.out.println("m_shooterAngle.set reverse (truss)");
                m_shooterAngleControl.set(Relay.Value.kReverse);
            }
            
            if(m_driveStick.getRawButton(1))
            {
                System.out.println("m_frontIntakeAngleControl.set forward");
                m_frontIntakeAngleControl.set(DoubleSolenoid.Value.kForward);
            }
            else if (m_driveStick.getRawButton(2))
            {
                System.out.println("m_frontIntakeAngleControl.set reverse");
                m_frontIntakeAngleControl.set(DoubleSolenoid.Value.kReverse);
            }
            
            if(m_driveStick.getRawButton(3))
            {
                System.out.println("m_backIntakeAngleControl.set forward");
                m_backIntakeAngleControl.set(DoubleSolenoid.Value.kForward);
            }
            else if (m_driveStick.getRawButton(4))
            {
                System.out.println("m_backIntakeAngleControl.set reverse");
                m_backIntakeAngleControl.set(DoubleSolenoid.Value.kReverse);
            }
            
            if(m_driveStick.getRawButton(7))
            {
                System.out.println("m_rampControl.setDirection forward");
                m_rampControl.set(Relay.Value.kForward);
            }
            else if (m_driveStick.getRawButton(8))
            {
                System.out.println("m_rampControl.setDirection reverse");
                m_rampControl.set(Relay.Value.kReverse);
            }
            
            m_frontIntakeControl.SetSpin(m_driveStick.getRawAxis(2));
            m_backIntakeControl.SetSpin(m_driveStick.getRawAxis(5));
        }
    }
    public void disabledPeriodic()
    {
        this.upateDashboard();
        
        // pull back the shooter winch
        /*do
        {
            m_shooterControl.PeriodicFunc();
        } while (!m_shooterControl.IsPulledBack());*/
    }
    private void readDashboard(){
    }
    private void upateDashboard(){//New method. Update dashboard.
        SmartDashboard.putNumber("Voltage", m_driverStation.getBatteryVoltage());
        SmartDashboard.putNumber("Speed",m_driveRightEncoder.getRate());
        SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
        SmartDashboard.putString("Code", "Robot907Code2014");
        SmartDashboard.putString("Date", "4:05 13/02/2014");
    }
    
    private void TestModeReadDashboard(){
        m_configurableValues.m_autonomousDriveForwardDistance = SmartDashboard.getNumber("m_autonomousDriveForwardDistance", ConfigurableValues.FORWARD_DISTANCE);
        m_configurableValues.m_autonomousDriveForwardPidControllerP = SmartDashboard.getNumber("m_autonomousDriveForwardPidControllerP", ConfigurableValues.FORWARD_PID_P);
        m_configurableValues.m_autonomousDriveForwardPidControllerI = SmartDashboard.getNumber("m_autonomousDriveForwardPidControllerI", ConfigurableValues.FORWARD_PID_I);
        m_configurableValues.m_autonomousDriveForwardPidControllerI = SmartDashboard.getNumber("m_autonomousDriveForwardPidControllerD", ConfigurableValues.FORWARD_PID_D);
        m_configurableValues.m_autonomousDriveForwardPidControllerTolerance = SmartDashboard.getNumber("m_autonomousDriveForwardPidControllerTolerance", ConfigurableValues.FORWARD_PID_TOLERANCE);
       
        m_configurableValues.m_autonomousDriveBackDistance = SmartDashboard.getNumber("m_autonomousDriveBackDistance", ConfigurableValues.BACK_DISTANCE);
        m_configurableValues.m_autonomousDriveBackPidControllerP = SmartDashboard.getNumber("m_autonomousDriveBackPidControllerP", ConfigurableValues.BACK_PID_P);
        m_configurableValues.m_autonomousDriveBackPidControllerI = SmartDashboard.getNumber("m_autonomousDriveBackPidControllerI", ConfigurableValues.BACK_PID_I);
        m_configurableValues.m_autonomousDriveBackPidControllerI = SmartDashboard.getNumber("m_autonomousDriveBackPidControllerD", ConfigurableValues.BACK_PID_D);
        m_configurableValues.m_autonomousDriveBackPidControllerTolerance = SmartDashboard.getNumber("m_autonomousDriveBackPidControllerTolerance", ConfigurableValues.BACK_PID_TOLERANCE);
        
        m_configurableValues.m_rectangleFilterHeightMax = SmartDashboard.getNumber("m_rectangleFilterHeightMax", ConfigurableValues.RECTANGLE_FILTER_HEIGHT_MAX);
        m_configurableValues.m_rectangleFilterHeightMin = SmartDashboard.getNumber("m_rectangleFilterHeightMin", ConfigurableValues.RECTANGLE_FILTER_HEIGHT_MIN);
        m_configurableValues.m_rectangleFilterWidthMin = SmartDashboard.getNumber("m_rectangleFilterWidthMin ", ConfigurableValues.RECTANGLE_FILTER_WIDTH_MIN);
        m_configurableValues.m_rectangleFilterWidthMax = SmartDashboard.getNumber("m_rectangleFilterWidthMax ", ConfigurableValues.RECTANGLE_FILTER_WIDTH_MAX);
        m_configurableValues.m_cameraFilterRMin = SmartDashboard.getNumber("m_cameraFilterRMin", ConfigurableValues.CAMERA_FILTER_R_MIN);
        m_configurableValues.m_cameraFilterRMax = SmartDashboard.getNumber("m_cameraFilterRMax", ConfigurableValues.CAMERA_FILTER_R_MAX);
        m_configurableValues.m_cameraFilterGMin = SmartDashboard.getNumber("m_cameraFilterGMin", ConfigurableValues.CAMERA_FILTER_G_MIN);
        m_configurableValues.m_cameraFilterGMax = SmartDashboard.getNumber("m_cameraFilterGMax", ConfigurableValues.CAMERA_FILTER_G_MAX);
        m_configurableValues.m_cameraFilterBMin = SmartDashboard.getNumber("m_cameraFilterBMin", ConfigurableValues.CAMERA_FILTER_B_MIN);
        m_configurableValues.m_cameraFilterBMax = SmartDashboard.getNumber("m_cameraFilterBMax", ConfigurableValues.CAMERA_FILTER_B_MAX);

    }
    
    private void TestModeUpateDashboard(){//New method. Update dashboard.
        SmartDashboard.putNumber("Voltage", m_driverStation.getBatteryVoltage());
        SmartDashboard.putNumber("Speed",m_driveRightEncoder.getRate());
        SmartDashboard.putNumber("m_autonomousDriveForwardDistance", m_configurableValues.m_autonomousDriveForwardDistance);
        SmartDashboard.putNumber("m_autonomousDriveForwardPidControllerP", m_configurableValues.m_autonomousDriveForwardPidControllerP);
        SmartDashboard.putNumber("m_autonomousDriveForwardPidControllerI", m_configurableValues.m_autonomousDriveForwardPidControllerI);
        SmartDashboard.putNumber("m_autonomousDriveForwardPidControllerD", m_configurableValues.m_autonomousDriveForwardPidControllerD);
        SmartDashboard.putNumber("m_autonomousDriveForwardPidControllerTolerance", m_configurableValues.m_autonomousDriveForwardPidControllerTolerance);
        
        SmartDashboard.putNumber("m_autonomousDriveBackDistance", m_configurableValues.m_autonomousDriveBackDistance);
        SmartDashboard.putNumber("m_autonomousDriveBackPidControllerP", m_configurableValues.m_autonomousDriveBackPidControllerP);
        SmartDashboard.putNumber("m_autonomousDriveBackPidControllerI", m_configurableValues.m_autonomousDriveBackPidControllerI);
        SmartDashboard.putNumber("m_autonomousDriveBackPidControllerD", m_configurableValues.m_autonomousDriveBackPidControllerD);
        SmartDashboard.putNumber("m_autonomousDriveBackPidControllerTolerance", m_configurableValues.m_autonomousDriveBackPidControllerTolerance);
        
        SmartDashboard.putNumber("m_rectangleFilterHeightMax", m_configurableValues.m_rectangleFilterHeightMax);
        SmartDashboard.putNumber("m_rectangleFilterHeightMin", m_configurableValues.m_rectangleFilterHeightMin);
        SmartDashboard.putNumber("m_rectangleFilterWidthMin ", m_configurableValues.m_rectangleFilterWidthMin);
        SmartDashboard.putNumber("m_rectangleFilterWidthMax ", m_configurableValues.m_rectangleFilterWidthMax);
        SmartDashboard.putNumber("m_cameraFilterRMin", m_configurableValues.m_cameraFilterRMin);
        SmartDashboard.putNumber("m_cameraFilterRMax", m_configurableValues.m_cameraFilterRMax);
        SmartDashboard.putNumber("m_cameraFilterGMin", m_configurableValues.m_cameraFilterGMin);
        SmartDashboard.putNumber("m_cameraFilterGMax", m_configurableValues.m_cameraFilterGMax);
        SmartDashboard.putNumber("m_cameraFilterBMin", m_configurableValues.m_cameraFilterBMin);
        SmartDashboard.putNumber("m_cameraFilterBMax", m_configurableValues.m_cameraFilterBMax);

    }
    private IntakeControl m_frontIntakeControl;
    private IntakeControl m_backIntakeControl;
    private Talon m_frontIntakeWheelControl;
    private Talon m_backIntakeWheelControl;
    private DoubleSolenoid m_frontIntakeAngleControl;
    private DoubleSolenoid m_backIntakeAngleControl;
    private DoubleSolenoid m_driveGearControl;
    private DoubleSolenoid m_shooterGearControl;
    private Relay m_shooterAngleControl;
    private XboxController m_driveStick;
    private RobotDrivePID m_robotDrivePid;
    private Encoder m_driveRightEncoder;
    private Encoder m_driveLeftEncoder;
    private Jaguar m_shooterMotor;
    private AutonomousModeHandler m_autonomousModeHandler;
    private AxisCamera m_frontCamera;
    private DigitalInput m_shooterLimitSwitch;
    private DriverStation m_driverStation;
    private ShooterControl m_shooterControl;
    private XboxController m_shootStick;
    private ConfigurableValues m_configurableValues;
    private Encoder m_shooterEncoder;
    private Compressor m_compressor;
    private MultiSpeedController m_rightDriveSpeedController;
    private MultiSpeedController m_leftDriveSpeedController;
    private Gyro m_gyro;
    private Servo m_latchServo;
    private boolean m_frontIntakeChangeAngleReleased;
    private boolean m_backIntakeChangeAngleReleased;
    private boolean m_flippereChangeAngleReleased;
    private Relay m_rampControl;
    private DigitalInput m_frontIntakeArmAngleDetector;
}

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author Cybernetics
 */
public class ConfigurableValues
{
    ConfigurableValues()
    {
        m_autonomousDriveForwardDistance = FORWARD_DISTANCE;
        m_autonomousDriveForwardPidControllerP = FORWARD_PID_P;
        m_autonomousDriveForwardPidControllerI = FORWARD_PID_I;
        m_autonomousDriveForwardPidControllerD = FORWARD_PID_D;
        m_autonomousDriveForwardPidControllerTolerance = FORWARD_PID_TOLERANCE;
        
        m_autonomousDriveBackDistance = BACK_DISTANCE;
        m_autonomousDriveBackPidControllerP = BACK_PID_P;
        m_autonomousDriveBackPidControllerI = BACK_PID_I;
        m_autonomousDriveBackPidControllerD = BACK_PID_D;
        m_autonomousDriveBackPidControllerTolerance = BACK_PID_TOLERANCE;
        
        m_rectangleFilterWidthMin = RECTANGLE_FILTER_WIDTH_MIN;
        m_rectangleFilterWidthMax = RECTANGLE_FILTER_WIDTH_MAX;
        m_rectangleFilterHeightMin = RECTANGLE_FILTER_HEIGHT_MIN;
        m_rectangleFilterHeightMax = RECTANGLE_FILTER_HEIGHT_MAX;
        m_cameraFilterRMin = CAMERA_FILTER_R_MIN;
        m_cameraFilterRMax = CAMERA_FILTER_R_MAX;
        m_cameraFilterGMin = CAMERA_FILTER_G_MIN;
        m_cameraFilterGMax = CAMERA_FILTER_G_MAX;
        m_cameraFilterBMin = CAMERA_FILTER_B_MIN;
        m_cameraFilterBMax = CAMERA_FILTER_B_MAX;
    }        
    public double m_rectangleFilterWidthMin;
    public double m_rectangleFilterWidthMax;
    public double m_rectangleFilterHeightMin;
    public double m_rectangleFilterHeightMax;
    public double m_cameraFilterRMin;
    public double m_cameraFilterRMax;
    public double m_cameraFilterGMin;
    public double m_cameraFilterGMax;
    public double m_cameraFilterBMin;
    public double m_cameraFilterBMax;
    
    public double m_autonomousDriveForwardDistance;
    public double m_autonomousDriveForwardPidControllerP;
    public double m_autonomousDriveForwardPidControllerI;
    public double m_autonomousDriveForwardPidControllerD;
    public double m_autonomousDriveForwardPidControllerTolerance;
    
    public double m_autonomousDriveBackDistance;
    public double m_autonomousDriveBackPidControllerP;
    public double m_autonomousDriveBackPidControllerI;
    public double m_autonomousDriveBackPidControllerD;
    public double m_autonomousDriveBackPidControllerTolerance;
    
    public static final double FORWARD_DISTANCE = -120;
    public static final double FORWARD_PID_P = 115;
    public static final double FORWARD_PID_I = 0;
    public static final double FORWARD_PID_D = 60;
    public static final double FORWARD_PID_TOLERANCE = 6;
    
    public static final double BACK_DISTANCE = 140;
    public static final double BACK_PID_P = 135;
    public static final double BACK_PID_I = 0;
    public static final double BACK_PID_D = 65;
    public static final double BACK_PID_TOLERANCE = 6;
    
    public static final double RECTANGLE_FILTER_WIDTH_MIN = 40;
    public static final double RECTANGLE_FILTER_WIDTH_MAX = 400;
    public static final double RECTANGLE_FILTER_HEIGHT_MIN = 10;
    public static final double RECTANGLE_FILTER_HEIGHT_MAX = 25;
    
    public static final double CAMERA_FILTER_R_MIN = 0;
    public static final double CAMERA_FILTER_R_MAX = 20;
    public static final double CAMERA_FILTER_G_MIN = 20;
    public static final double CAMERA_FILTER_G_MAX = 255;
    public static final double CAMERA_FILTER_B_MIN = 0;
    public static final double CAMERA_FILTER_B_MAX = 20;

}

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.MonoImage;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision.MeasurementType;

/**
 *
 * @author Cybernetics
 */
public class AutonomousImageDetector {
    
    public AutonomousImageDetector(AxisCamera cam, ConfigurableValues configurableValues)
    {
        m_configurableValues = configurableValues;
        m_camera = cam;
    }
    
    public void Init()
    {
        /*m_camera.writeBrightness(50);
        m_camera.writeWhiteBalance(AxisCamera.WhiteBalanceT.automatic);p
        m_camera.writeColorLevel(50);
        m_camera.writeExposureControl(AxisCamera.ExposureT.automatic);
        m_camera.writeExposurePriority(AxisCamera.ExposurePriorityT.none);
        m_camera.writeResolution(AxisCamera.ResolutionT.k640x480);
        m_camera.writeCompression(30);
        m_camera.writeMaxFPS(30);
        m_camera.writeRotation(AxisCamera.RotationT.k0);*/
       
        m_consecutiveDetectionCount = 0;
        m_detectionState = DETECTION_STATE_DETECTING;
        m_firstImage = true;
    }
    
    public byte CurrentDetectionState()
    {
        byte detectionState = DETECTION_STATE_DETECTING;
        
        if (m_consecutiveDetectionCount >= REQUIRED_DETECTION_COUNT)
        {
            detectionState = m_detectionState;
        }
        
        return detectionState;
    }
    
    public void PeriodicFunc()
    {
        System.out.println("AutonomousImageDetector::PeriodicFunc: getting camera image freshImage: " + m_camera.freshImage() + " m_consecutiveDetectionCount: " + m_consecutiveDetectionCount);
        
        if ((m_consecutiveDetectionCount < REQUIRED_DETECTION_COUNT) && m_camera.freshImage())
        {
            Logger.PrintLine("AutonomousImageDetector::PeriodicFunc: getting camera image ", Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
          
            try
            {
                ColorImage image = m_camera.getImage();
                
                if (null != image)
                {
                    CriteriaCollection criteriaCollection = new CriteriaCollection();
                    
                    criteriaCollection.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH, (int)m_configurableValues.m_rectangleFilterWidthMin,
                                                                                                (int)m_configurableValues.m_rectangleFilterWidthMax,
                                                                                                false);
                    criteriaCollection.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_HEIGHT, (int)m_configurableValues.m_rectangleFilterHeightMin,
                                                                                                (int)m_configurableValues.m_rectangleFilterHeightMax,
                                                                                                false);

                    // Return only green objects
                    BinaryImage thresholdImage = image.thresholdRGB((int)m_configurableValues.m_cameraFilterRMin,
                                                                    (int)m_configurableValues.m_cameraFilterRMax,
                                                                    (int)m_configurableValues.m_cameraFilterGMin,
                                                                    (int)m_configurableValues.m_cameraFilterGMax,
                                                                    (int)m_configurableValues.m_cameraFilterBMin,
                                                                    (int)m_configurableValues.m_cameraFilterBMax);

                    thresholdImage.removeSmallObjects(false, 2);

                    BinaryImage rectImage = thresholdImage.convexHull(false);
                    
                    rectImage.removeSmallObjects(false, 2);
                    
                    BinaryImage filteredImage = rectImage.particleFilter(criteriaCollection);

                    
                    int numOfParticles = filteredImage.getNumberParticles();
                    if(numOfParticles == 1)
                    {
                        if (DETECTION_STATE_GOAL_LIT == m_detectionState)
                        {
                            m_consecutiveDetectionCount++;
                        }
                        else
                        {
                            m_consecutiveDetectionCount = 1;
                        }
                        m_detectionState = DETECTION_STATE_GOAL_LIT;
                    }
                    else if(numOfParticles == 0)
                    {
                        if (DETECTION_STATE_GOAL_UNLIT == m_detectionState)
                        {
                            m_consecutiveDetectionCount++;
                        }
                        else
                        {
                            m_consecutiveDetectionCount = 1;
                        }
                        m_detectionState = DETECTION_STATE_GOAL_UNLIT;
                    }
                    else
                    {
                        m_consecutiveDetectionCount = 0;
                        m_detectionState = DETECTION_STATE_DETECTING;
                    }
                    
                    Logger.PrintLine("AutonomousImageDetector::PeriodicFunc: filteredImage.getNumberParticles: " + numOfParticles + " m_consecutiveDetectionCount: " + m_consecutiveDetectionCount + " m_detectionState: " + m_detectionState,Logger.LOGGER_MESSAGE_LEVEL_DEBUG);
                    
                    
                    /*if (m_firstImage)
                    {
                        m_firstImage = false;
                        image.write("/capturedImg.jpg");
                        thresholdImage.write("/thresholdImg.bmp");
                        rectImage.write("/rectImg.bmp");
                        filteredImage.write("/filtewredImg.bmp");
                    }*/
                    
                    filteredImage.free();
                    filteredImage = null;
                    rectImage.free();
                    rectImage = null;
                    thresholdImage.free();
                    thresholdImage = null;
                    image.free();
                    image = null;
                }
            }
            catch (NIVisionException ex)
            {
              ex.printStackTrace();
              System.exit(1);
            }       
            catch (AxisCameraException ex)
            {
              ex.printStackTrace();
              System.exit(1);
            }
        }
        else
            m_detectionState = DETECTION_STATE_GOAL_LIT;
    }
    
    public static final byte DETECTION_STATE_DETECTING = 0x1;
    public static final byte DETECTION_STATE_GOAL_LIT = 0x2;
    public static final byte DETECTION_STATE_GOAL_UNLIT = 0x3;
    
    
    private static final byte REQUIRED_DETECTION_COUNT = 2;
    
    private final AxisCamera m_camera;
    private int m_consecutiveDetectionCount;
    private byte m_detectionState;
    private boolean m_firstImage;
    private final ConfigurableValues m_configurableValues;
    
}

/*  Sup krisztian  :P
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author Cybernetics
 */
public class Logger {
     public static final byte LOGGER_MESSAGE_LEVEL_DEBUG = 0x1;
     public static final byte LOGGER_MESSAGE_LEVEL_INFO = 0x2;
     public static final byte LOGGER_MESSAGE_LEVEL_ERROR = 0x4;
     public static final byte LOGGER_MESSAGE_LEVEL_ALL = (byte)0xFF;
     
     private static byte s_loggingLevel = 0;
     
     public static void SetLoggingLevel(byte loggingLevel)
     {
         s_loggingLevel = loggingLevel;
        
     }
     
     public static void PrintLine (String logMessage, byte logLevel)
     {
         if (0 != (logLevel & s_loggingLevel))
         {
             System.out.println(logMessage);
         }
     }
}

package frc.robot.Shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterConfig {
    
    // **** LEFT SHOOTER CONFIG ****

    //CAN bus ID
    public static int left_ID = 32; //Master ID

    //Type configs
    public static MotorType left_motorType = MotorType.kBrushless;
    public static IdleMode left_idleMode = IdleMode.kCoast; //Brake for position, coast for percent

    //MAKE SURE TO SET CORRECT CONTROL MODE BY CALLING CORRECT API!!! IMPORTANT!!!

    //PIDF Values
    public static double left_p = 0.01; //.01 good start value
    public static double left_i = 0;
    public static double left_d = 0;
    public static double left_f = 0;
    public static double left_IZone = 0;
    public static double left_DFilter = 0;

    //Min + Max Output
    public static double left_outputMin = -1;
    public static double left_outputMax = 1;

    //Ramp Rate
    public static double left_openRampRate = 0.1; //Time in seconds (1 is good for percent based stuff) (percent mode)
    public static double left_closedRampRate = 0.1; // IF IN POSITION MODE, THIS IS THE RAMP RATE SETTING!!!

    //Follower ID
    public static int left_followerID = 0; //Follower ID

    //Inverted
    public static boolean left_kInverted = true;

    //Current Limits
    public static int left_smartCurrentStallLimit = 20;  //40 for big neo, 20 for small neo
    public static int left_smartCurrentFreeLimit = 15; //30 for big neo, 15 for small neo

    //Conversion Factors
    public static double left_positionConversionFactor = 1; //360 / gear ratio
    public static double left_velocityConversionFactor = 1;

    //Soft Limits
    public static float left_softLimitFwd = 0;
    public static float left_softLimitRev = 0;

        //Soft Limits Enabled
    public static boolean left_softLimitFwdEnabled = false;
    public static boolean left_softLimitRevEnabled = false;    

    //Analog for Hollow Bore
    public static double left_analogPositionConversion = 1;
    public static double left_analogVelocityConversion = 1;
    public static int left_analogSensorMore = 0; // 0 = absolute; 1 = relative
    public static int left_analogInverted = 0;

        //left control method
    public static ControlType left_controlType = ControlType.kVelocity;




    // **** RIGHT SHOOTER CONFIG ****

    //CAN bus ID
    public static int ID = 33;

    //Type configs
    public static MotorType right_motorType = MotorType.kBrushless;
    public static IdleMode right_idleMode = IdleMode.kCoast; //Brake for position, coast for percent

    //MAKE SURE TO SET CORRECT CONTROL MODE BY CALLING CORRECT API!!! IMPORTANT!!!

    //PIDF Values
    public static double right_p = 0.01; //.01 good start value
    public static double right_i = 0;
    public static double right_d = 0;
    public static double right_f = 0;
    public static double right_IZone = 0;
    public static double right_DFilter = 0;

    //Min + Max Output
    public static double right_outputMin = -1;
    public static double right_outputMax = 1;

    //Ramp Rate
    public static double right_openRampRate = 0.1; //Time in seconds (1 is good for percent based stuff) (percent mode)
    public static double right_closedRampRate = 0.1; // IF IN POSITION MODE, THIS IS THE RAMP RATE SETTING!!!

    //Follower ID
    public static int right_followerID = 0;

    //Inverted
    public static boolean right_kInverted = false;

    //Current Limits
    public static int right_smartCurrentStallLimit = 20;  //40 for big neo, 20 for small neo
    public static int right_smartCurrentFreeLimit = 15; //30 for big neo, 15 for small neo

    //Conversion Factors
    public static double right_positionConversionFactor = 1; //360 / gear ratio
    public static double right_velocityConversionFactor = 1;

    //Soft Limits
    public static float right_softLimitFwd = 0;
    public static float right_softLimitRev = 0;

        //Soft Limits Enabled
    public static boolean right_softLimitFwdEnabled = false;
    public static boolean right_softLimitRevEnabled = false;    



    //Analog for Hollow Bore
    public static double right_analogPositionConversion = 1;
    public static double right_analogVelocityConversion = 1;
    public static int right_analogSensorMore = 0; // 0 = absolute; 1 = relative
    public static int right_analogInverted = 0;


    //right control method
    public static ControlType right_controlType = ControlType.kVelocity;

}

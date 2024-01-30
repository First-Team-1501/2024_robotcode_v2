package frc.robot.Intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeConfig {
    
    // **** INTAKE TOP CONFIG ****

    //CAN bus ID
    public static int top_ID = 20;

    //Type configs
    public static MotorType top_motorType = MotorType.kBrushless;
    public static IdleMode top_idleMode = IdleMode.kCoast; //Brake for position, coast for percent

    //MAKE SURE TO SET CORRECT CONTROL MODE BY CALLING CORRECT API!!! IMPORTANT!!!

    //PIDF Values
    public static double top_p = 0.01; //.01 good start value
    public static double top_i = 0;
    public static double top_d = 0;
    public static double top_f = 0;
    public static double top_IZone = 0;
    public static double top_DFilter = 0;

    //Min + Max Output
    public static double top_outputMin = -1;
    public static double top_outputMax = 1;

    //Climber -> .25 output speed

    //Ramp Rate
    public static double top_openRampRate = 0; //Time in seconds (1 is good for percent based stuff) (percent mode)
    public static double top_closedRampRate = 0; // IF IN POSITION MODE, THIS IS THE RAMP RATE SETTING!!!

    //Follower ID
    public static int top_followerID = 0;

    //Inverted
    public static Boolean top_kInverted = false;

    //Current Limits
    public static int top_smartCurrentStallLimit = 20;  //40 for big neo, 20 for small neo
    public static int top_smartCurrentFreeLimit = 15; //30 for big neo, 15 for small neo

    //Conversion Factors
    public static double top_positionConversionFactor = 1; //360 / gear ratio
    public static double top_velocityConversionFactor = 1;

    //Soft Limits Forward
    public static Boolean top_softLimitFwdEnabled = true;
    public static float top_softLimitFwd = 0;

    //Soft Limits Reverse
    public static Boolean top_softLimitRevEnabled = true;
    public static float top_softLimitRev = 0;

    //Analog for Hollow Bore
    public static double top_analogPositionConversion = 1;
    public static double top_analogVelocityConversion = 1;
    public static int top_analogSensorMore = 0; // 0 = absolute; 1 = relative
    public static int top_analogInverted = 0;

    

    // **** INTAKE BOTTOM CONFIG ****

    //CAN bus ID
    public static int bottom_ID = 21;

    //Type configs
    public static MotorType bottom_motorType = MotorType.kBrushless;
    public static IdleMode bottom_idleMode = IdleMode.kCoast; //Brake for position, coast for percent

    //MAKE SURE TO SET CORRECT CONTROL MODE BY CALLING CORRECT API!!! IMPORTANT!!!

    //PIDF Values
    public static double bottom_p = 0.01; //.01 good start value
    public static double bottom_i = 0;
    public static double bottom_d = 0;
    public static double bottom_f = 0;
    public static double bottom_IZone = 0;
    public static double bottom_DFilter = 0;

    //Min + Max Output
    public static double bottom_outputMin = -1;
    public static double bottom_outputMax = 1;

    //Climber -> .25 output speed

    //Ramp Rate
    public static double bottom_openRampRate = 1; //Time in seconds (1 is good for percent based stuff) (percent mode)
    public static double bottom_closedRampRate = 0; // IF IN POSITION MODE, THIS IS THE RAMP RATE SETTING!!!

    //Follower ID
    public static int bottom_followerID = 0;

    //Inverted
    public static Boolean bottom_kInverted = true;

    //Current Limits
    public static int bottom_smartCurrentStallLimit = 20;  //40 for big neo, 20 for small neo
    public static int bottom_smartCurrentFreeLimit = 15; //30 for big neo, 15 for small neo

    //Conversion Factors
    public static double bottom_positionConversionFactor = 1; //360 / gear ratio
    public static double bottom_velocityConversionFactor = 1;

    //Soft Limits Forward
    public static Boolean bottom_softLimitFwdEnabled = true;
    public static float bottom_softLimitFwd = 0;

    //Soft Limits Reverse
    public static Boolean bottom_softLimitRevEnabled = true;
    public static float bottom_softLimitRev = 0;

    //Analog for Hollow Bore
    public static double bottom_analogPositionConversion = 1;
    public static double bottom_analogVelocityConversion = 1;
    public static int bottom_analogSensorMore = 0; // 0 = absolute; 1 = relative
    public static int bottom_analogInverted = 0;



    // **Running Speeds for Top in All instances

    public static double top_inSpeed = 1;
    public static double top_outSpeed = -1;

    // **Running Speeds for Bottom in All instances

    public static double bottom_inSpeed = 1;
    public static double bottom_outSpeed = -1;



}

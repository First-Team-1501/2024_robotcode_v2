package frc.robot.Elevator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ElevatorConfig {
    
    // **** ELEVATOR CONFIG ****

    //CAN bus ID
    public static int ID = 22;

    //Type configs
    public static MotorType motorType = MotorType.kBrushless;
    public static IdleMode idleMode = IdleMode.kCoast; //Brake for position, coast for percent

    //MAKE SURE TO SET CORRECT CONTROL MODE BY CALLING CORRECT API!!! IMPORTANT!!!

    //PIDF Values
    public static double p = 0.01; //.01 good start value
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;
    public static double IZone = 0;
    public static double DFilter = 0;

    //Min + Max Output
    public static double outputMin = -0.75;
    public static double outputMax = 0.75;

    //Ramp Rate
    public static double openRampRate = 0.5; //Time in seconds (1 is good for percent based stuff) (percent mode)
    public static double closedRampRate = 1; // IF IN POSITION MODE, THIS IS THE RAMP RATE SETTING!!!

    //Follower ID
    public static int followerID = 0; //no follower

    //Inverted
    public static boolean kInverted = true;

    //Current Limits
    public static int smartCurrentStallLimit = 20;  //40 for big neo, 20 for small neo
    public static int smartCurrentFreeLimit = 15; //30 for big neo, 15 for small neo

    //Conversion Factors
    public static double positionConversionFactor = 1; 
    public static double velocityConversionFactor = 1;

    //Soft Limits
    public static float softLimitFwd = 58;
    public static float softLimitRev = 0;

        //Soft Limits Enabled
    public static boolean softLimitFwdEnabled = true;
    public static boolean softLimitRevEnabled = true;    

    //Analog for Hollow Bore
    public static double analogPositionConversion = 1;
    public static double analogVelocityConversion = 1;
    public static int analogSensorMore = 0; // 0 = absolute; 1 = relative
    public static int analogInverted = 0;

    //Control type to position
    public static ControlType controlType = ControlType.kPosition;


}

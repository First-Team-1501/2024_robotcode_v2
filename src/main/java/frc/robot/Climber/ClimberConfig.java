package frc.robot.Climber;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberConfig {
    
    
    
    // **** CLIMBER CONFIG ****

    // Climber motors are master/slave so all configs will be the same

    //CAN bus ID
    public static int ID = 40; //Master ID (left side)

    //Type configs
    public static MotorType motorType = MotorType.kBrushless;
    public static IdleMode idleMode = IdleMode.kBrake; //Brake for position, coast for percent

    //MAKE SURE TO SET CORRECT CONTROL MODE BY CALLING CORRECT API!!! IMPORTANT!!!

    //PIDF Values
    public static double p = 0.01; //.01 good start value
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;
    public static double IZone = 0;
    public static double DFilter = 0;

    //Min + Max Output
    public static double outputMin = -1;
    public static double outputMax = 1;

    //Ramp Rate
    public static double openRampRate = 0; //Time in seconds (1 is good for percent based stuff) (percent mode)
    public static double closedRampRate = 0; // IF IN POSITION MODE, THIS IS THE RAMP RATE SETTING!!!

    //Follower ID
    public static int followerID = 41; //Slave ID (right side)

    //Inverted
    public static boolean kInverted = false;

    //Current Limits
    public static int smartCurrentStallLimit = 20;  //40 for big neo, 20 for small neo
    public static int smartCurrentFreeLimit = 15; //30 for big neo, 15 for small neo

    //Conversion Factors
    public static double positionConversionFactor = 1; //360 / gear ratio
    public static double velocityConversionFactor = 1;

    //Soft Limits Enabled
    public static boolean softLimitFwdEnabled = true;
    public static boolean softLimitRevEnabled = true;
    
    //Soft Limits
    public static float softLimitFwd = 0;
    public static float softLimitRev = 0;

    //Analog for Hollow Bore
    public static double analogPositionConversion = (1/127)/360;//125:1 gear box, 16 small gear, 22 big gear = 0.00581818181/360
    public static double analogVelocityConversion = 1;
    public static int analogSensorMore = 0; // 0 = absolute; 1 = relative
    public static int analogInverted = 0;

    //Follower Inversion
    public static boolean follow_isInverted = true;

    //Master ControlType
    public static ControlType controlType = ControlType.kPosition;

    


}

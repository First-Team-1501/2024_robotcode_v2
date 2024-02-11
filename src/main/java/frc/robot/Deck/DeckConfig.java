package frc.robot.Deck;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DeckConfig {
    
    // **** DECK CONFIG ****

    // Deck motors are master/slave so all configs will be the same

    //CAN bus ID
    public static int ID = 30; //Master ID

    //Type configs
    public static MotorType motorType = MotorType.kBrushless;
    public static IdleMode idleMode = IdleMode.kCoast; //Brake for position, coast for percent

    //MAKE SURE TO SET CORRECT CONTROL MODE BY CALLING CORRECT API!!! IMPORTANT!!!

    //PIDF Values
    public static double p = 0.01; //.01 good start value
    public static double i = 0;
    public static double d = 0;
    public static double f = 0.1;
    public static double IZone = 0;
    public static double DFilter = 0;

    //Idle Mode

    //Min + Max Output
    public static double outputMin = -0.75;
    public static double outputMax = 0.75;

    //Ramp Rate
    public static double openRampRate = 0.5; //Time in seconds (1 is good for percent based stuff) (percent mode)
    public static double closedRampRate = .5; // IF IN POSITION MODE, THIS IS THE RAMP RATE SETTING!!!

    //Follower ID
    public static int followerID = 31; //Slave ID//no follower

    //Inverted
    public static boolean kInverted = false;

    //Current Limits
    public static int smartCurrentStallLimit = 40;  //40 for big neo, 20 for small neo
    public static int smartCurrentFreeLimit = 30; //30 for big neo, 15 for small neo

    //Conversion Factors
    public static double positionConversionFactor = 1;
    public static double velocityConversionFactor = 1;

    //Soft Limits Enabled
    public static boolean softLimitFwdEnabled = true;
    public static boolean softLimitRevEnabled = true;

    //Soft Limits
    public static float softLimitFwd = 75;
    public static float softLimitRev = 0;

    //Analog for Hollow Bore
    public static double analogPositionConversion = 1;  //1/125*16/64/360;//125:1 (0.008), 16 small, 64 T (0.25), (0.032)/360 / gear ratio
    public static double analogVelocityConversion = 1;
    public static int analogSensorMore = 0; // 0 = absolute; 1 = relative
    public static int analogInverted = 0;

    //Follower Inversion
    public static boolean follow_isInverted = true; //no follower

    //Master ControlType
    public static ControlType controlType = ControlType.kPosition;
    



}

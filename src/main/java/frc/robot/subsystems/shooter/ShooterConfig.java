package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterConfig {

    // Shooter side speeds
    public static double leftSpeed = 0.6;
    public static double rightSpeed = 0.3;

    // **** LEFT SHOOTER CONFIG ****

    // CAN bus ID
    public static int left_ID = 32;

    // Type configs
    public static MotorType left_motorType = MotorType.kBrushless;
    public static IdleMode left_idleMode = IdleMode.kCoast; // Brake for position, coast for percent

    // PIDF Values
    public static double left_p = 0.01; // .01 good start value
    public static double left_i = 0;
    public static double left_d = 0;
    public static double left_f = 0;
    public static double left_IZone = 0;
    public static double left_DFilter = 0;

    // Min + Max Output
    public static double left_outputMin = -1;
    public static double left_outputMax = 1;

    // Ramp Rate
    public static double left_openRampRate = 0; // Time in seconds (1 is good for percent based stuff) (percent mode)

    // Inverted
    public static boolean left_kInverted = true;

    // Current Limits
    public static int left_smartCurrentStallLimit = 40; // 40 for big neo, 20 for small neo
    public static int left_smartCurrentFreeLimit = 25; // 30 for big neo, 15 for small neo

    // Conversion Factors
    public static double left_velocityConversionFactor = 1;

    // **** RIGHT SHOOTER CONFIG ****

    // CAN bus ID
    public static int right_ID = 33;

    // Type configs
    public static MotorType right_motorType = MotorType.kBrushless;
    public static IdleMode right_idleMode = IdleMode.kCoast; // Brake for position, coast for percent

    // PIDF Values
    public static double right_p = 0.01; // .01 good start value
    public static double right_i = 0;
    public static double right_d = 0;
    public static double right_f = 0;
    public static double right_IZone = 0;
    public static double right_DFilter = 0;

    // Min + Max Output
    public static double right_outputMin = -1;
    public static double right_outputMax = 1;

    // Ramp Rate
    public static double right_openRampRate = 0.5; // Time in seconds (1 is good for percent based stuff) (percent mode)
    public static double right_closedRampRate = 0.5; // IF IN POSITION MODE, THIS IS THE RAMP RATE SETTING!!!

    // Inverted
    public static boolean right_kInverted = false;

    // Current Limits
    public static int right_smartCurrentStallLimit = 40; // 40 for big neo, 20 for small neo
    public static int right_smartCurrentFreeLimit = 25; // 30 for big neo, 15 for small neo

    

}

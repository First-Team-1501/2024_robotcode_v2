package frc.robot.subsystems.IntakeSubsystem;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeConfig {

    // **** INTAKE TOP CONFIG ****

    // CAN bus ID
    public static int top_ID = 20;

    // Type configs
    public static MotorType top_motorType = MotorType.kBrushless;
    public static IdleMode top_idleMode = IdleMode.kCoast; // Brake for position, coast for percent

    // PIDF Values
    public static double top_p = 0.01; // .01 good start value
    public static double top_i = 0;
    public static double top_d = 0;
    public static double top_f = 0;
    public static double top_IZone = 0;
    public static double top_DFilter = 0;

    // Min + Max Output
    public static double top_outputMin = -1;
    public static double top_outputMax = 1;

    // Ramp Rate
    public static double top_openRampRate = 0; // Time in seconds (1 is good for percent based stuff) (percent mode)

    // Inverted
    public static Boolean top_kInverted = true;

    // Current Limits
    public static int top_smartCurrentStallLimit = 20; // 40 for big neo, 20 for small neo
    public static int top_smartCurrentFreeLimit = 15; // 30 for big neo, 15 for small neo

    // **** INTAKE BOTTOM CONFIG ****

    // CAN bus ID
    public static int bottom_ID = 21;

    // Type configs
    public static MotorType bottom_motorType = MotorType.kBrushless;
    public static IdleMode bottom_idleMode = IdleMode.kCoast; // Brake for position, coast for percent

    // PIDF Values
    public static double bottom_p = 0.01; // .01 good start value
    public static double bottom_i = 0;
    public static double bottom_d = 0;
    public static double bottom_f = 0;
    public static double bottom_IZone = 0;
    public static double bottom_DFilter = 0;

    // Min + Max Output
    public static double bottom_outputMin = -1;
    public static double bottom_outputMax = 1;

    // Ramp Rate
    public static double bottom_openRampRate = 0; // Time in seconds (1 is good for percent based stuff) (percent mode)

    // Inverted
    public static Boolean bottom_kInverted = true;

    // Current Limits
    public static int bottom_smartCurrentStallLimit = 20; // 40 for big neo, 20 for small neo
    public static int bottom_smartCurrentFreeLimit = 15; // 30 for big neo, 15 for small neo

    // SENSOR CHANNELS
    public static int intakeSensorID = 5;
    public static int outtakeSensorID = 6;

}

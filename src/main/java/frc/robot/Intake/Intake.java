package frc.robot.Intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

public class Intake {
    
    //Intake will be percent mode
    

        // ***  DECLARING MOTORS & SENSORS
    //DECLARE INTAKE MOTORS
    private CANSparkMax m_intakeTop;
    private CANSparkMax m_intakeBottom;

    //DECLARE INTAKE ELEVATOR
    private CANSparkMax m_intakeElevator;
    //potential REV Thourough Bore Encoder >>HERE<<

    //DECLARE Photoeyes
    private DigitalInput pe_notePresent;
    private DigitalInput pe_noteInQueue;

    //CONSTRUCTOR INTAKE
    public Intake()
    {

        //INITIALIZE & CONFIGURE m_topIntake
        m_intakeTop = new CANSparkMax(IntakeConfig.top_ID, IntakeConfig.top_motorType);
        m_intakeTop.setIdleMode(IntakeConfig.top_idleMode);
        
        //pid
        m_intakeTop.getPIDController().setP(IntakeConfig.top_p);
        m_intakeTop.getPIDController().setI(IntakeConfig.top_i);
        m_intakeTop.getPIDController().setD(IntakeConfig.top_d);
        m_intakeTop.getPIDController().setIZone(IntakeConfig.top_IZone);
        m_intakeTop.getPIDController().setDFilter(IntakeConfig.top_DFilter);
        
        //min/max output
        m_intakeTop.getPIDController().setOutputRange(IntakeConfig.top_outputMin, IntakeConfig.top_outputMax);

        //Ramp Rate
        m_intakeTop.setOpenLoopRampRate(IntakeConfig.top_openRampRate);
        m_intakeTop.setClosedLoopRampRate(IntakeConfig.top_closedRampRate);

        //current limits
        m_intakeTop.setSmartCurrentLimit(IntakeConfig.top_smartCurrentStallLimit, IntakeConfig.top_smartCurrentFreeLimit);
        
        //TODO posConversionFactor

    }


    




}

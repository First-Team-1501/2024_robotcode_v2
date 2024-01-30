package frc.robot.Intake;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

public class Intake {
    
    //Intake will be percent mode
    

        // ***  DECLARING MOTORS & SENSORS
    //DECLARE INTAKE MOTORS
    private CANSparkMax m_topIntake;
    private CANSparkMax m_bottomIntake;

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
        m_topIntake = new CANSparkMax(IntakeConfig.top_ID, IntakeConfig.top_motorType);
        m_topIntake.setIdleMode(IntakeConfig.top_idleMode);

        //top motor Encoder for ease of access
        RelativeEncoder e_topIntake = m_topIntake.getEncoder();
        
        //pid
        m_topIntake.getPIDController().setP(IntakeConfig.top_p);
        m_topIntake.getPIDController().setI(IntakeConfig.top_i);
        m_topIntake.getPIDController().setD(IntakeConfig.top_d);
        m_topIntake.getPIDController().setIZone(IntakeConfig.top_IZone);
        m_topIntake.getPIDController().setDFilter(IntakeConfig.top_DFilter);
        
        //min/max output
        m_topIntake.getPIDController().setOutputRange(IntakeConfig.top_outputMin, IntakeConfig.top_outputMax);

        //Ramp Rate
        m_topIntake.setOpenLoopRampRate(IntakeConfig.top_openRampRate);
        m_topIntake.setClosedLoopRampRate(IntakeConfig.top_closedRampRate);

        //current limits
        m_topIntake.setSmartCurrentLimit(IntakeConfig.top_smartCurrentStallLimit, IntakeConfig.top_smartCurrentFreeLimit);
        
        //Conversion Factor
        e_topIntake.setPositionConversionFactor(IntakeConfig.top_positionConversionFactor);
        e_topIntake.setVelocityConversionFactor(IntakeConfig.top_velocityConversionFactor);

        //soft limits Forward
        m_topIntake.enableSoftLimit(SoftLimitDirection.kForward, IntakeConfig.top_softLimitFwdEnabled);
        m_topIntake.setSoftLimit(SoftLimitDirection.kForward, IntakeConfig.top_softLimitFwd);

        //soft limits Reverse
        m_topIntake.enableSoftLimit(SoftLimitDirection.kReverse, IntakeConfig.top_softLimitRevEnabled);
        m_topIntake.setSoftLimit(SoftLimitDirection.kReverse, IntakeConfig.top_softLimitRev);
        
        //TODO: Maybe add HOLLOW BORE



            //INITIALIZE & CONFIGURE m_bottomIntake
            
        m_bottomIntake = new CANSparkMax(IntakeConfig.bottom_ID, IntakeConfig.bottom_motorType);
        m_bottomIntake.setIdleMode(IntakeConfig.bottom_idleMode);

        //bottom motor Encoder for ease of access
        RelativeEncoder e_bottomIntake = m_bottomIntake.getEncoder();
        
        //pid
        m_bottomIntake.getPIDController().setP(IntakeConfig.bottom_p);
        m_bottomIntake.getPIDController().setI(IntakeConfig.bottom_i);
        m_bottomIntake.getPIDController().setD(IntakeConfig.bottom_d);
        m_bottomIntake.getPIDController().setIZone(IntakeConfig.bottom_IZone);
        m_bottomIntake.getPIDController().setDFilter(IntakeConfig.bottom_DFilter);
        
        //min/max output
        m_bottomIntake.getPIDController().setOutputRange(IntakeConfig.bottom_outputMin, IntakeConfig.bottom_outputMax);

        //Ramp Rate
        m_bottomIntake.setOpenLoopRampRate(IntakeConfig.bottom_openRampRate);
        m_bottomIntake.setClosedLoopRampRate(IntakeConfig.bottom_closedRampRate);

        //current limits
        m_bottomIntake.setSmartCurrentLimit(IntakeConfig.bottom_smartCurrentStallLimit, IntakeConfig.bottom_smartCurrentFreeLimit);
        
        //Conversion Factor
        e_bottomIntake.setPositionConversionFactor(IntakeConfig.bottom_positionConversionFactor);
        e_bottomIntake.setVelocityConversionFactor(IntakeConfig.bottom_velocityConversionFactor);

        //soft limits Forward
        m_bottomIntake.enableSoftLimit(SoftLimitDirection.kForward, IntakeConfig.bottom_softLimitFwdEnabled);
        m_bottomIntake.setSoftLimit(SoftLimitDirection.kForward, IntakeConfig.bottom_softLimitFwd);

        //soft limits Reverse
        m_bottomIntake.enableSoftLimit(SoftLimitDirection.kReverse, IntakeConfig.bottom_softLimitRevEnabled);
        m_bottomIntake.setSoftLimit(SoftLimitDirection.kReverse, IntakeConfig.bottom_softLimitRev);
        
        //TODO: Maybe add HOLLOW BORE



        

    }


    




}

package frc.robot.Climber;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Intake.IntakeConfig;

public class Climber {
    
    //Climber will be position mode

    //declare climber Master & Slave
    private CANSparkMax m_climberMaster;
    private CANSparkMax m_climberFollower;
    
    //declare climbermaster encoder
    private RelativeEncoder e_climberMaster;

    //declare master's pid controller
    private SparkPIDController pid_climberMaster;

    public Climber()
    {
        // **INITIALIZE MOTORS**
        m_climberMaster = new CANSparkMax(ClimberConfig.ID, ClimberConfig.motorType);
        m_climberFollower = new CANSparkMax(ClimberConfig.followerID, ClimberConfig.motorType);
                
        // //Master motor Encoder for ease of access
        e_climberMaster = m_climberMaster.getEncoder();

        //pid climber master
        pid_climberMaster = m_climberMaster.getPIDController();
        
        // //pid
        pid_climberMaster.setP(ClimberConfig.p);
        pid_climberMaster.setI(ClimberConfig.i);
        pid_climberMaster.setD(ClimberConfig.d);
        pid_climberMaster.setIZone(ClimberConfig.IZone);
        pid_climberMaster.setDFilter(ClimberConfig.DFilter);
        
        //min/max output
        pid_climberMaster.setOutputRange(ClimberConfig.outputMin, ClimberConfig.outputMax);

        //Ramp Rate
        m_climberMaster.setOpenLoopRampRate(ClimberConfig.openRampRate);
        m_climberMaster.setClosedLoopRampRate(ClimberConfig.closedRampRate);
        
        // Is Follower needed for Master Climber?

        //Inverted
        m_climberMaster.setInverted(ClimberConfig.kInverted);

        //current limits
        m_climberMaster.setSmartCurrentLimit(ClimberConfig.smartCurrentStallLimit, ClimberConfig.smartCurrentFreeLimit);
        
        //Conversion Factor
        e_climberMaster.setPositionConversionFactor(ClimberConfig.positionConversionFactor);
        e_climberMaster.setVelocityConversionFactor(ClimberConfig.velocityConversionFactor);

        //soft limits Forward
        m_climberMaster.enableSoftLimit(SoftLimitDirection.kForward, ClimberConfig.softLimitFwdEnabled);
        m_climberMaster.setSoftLimit(SoftLimitDirection.kForward, ClimberConfig.softLimitFwd);

        //soft limits Reverse
        m_climberMaster.enableSoftLimit(SoftLimitDirection.kReverse, ClimberConfig.softLimitRevEnabled);
        m_climberMaster.setSoftLimit(SoftLimitDirection.kReverse, ClimberConfig.softLimitRev);
        
        // //TODO: Maybe add HOLLOW BORE

        //control type to position
        pid_climberMaster.setReference(m_climberMaster.getEncoder().getPosition(), ControlType.kPosition);

        //BURN MASTER FLASH!!
        m_climberMaster.burnFlash();


        //m_climberFollower is follower and inverted
        m_climberFollower.follow(m_climberMaster);
        m_climberFollower.setInverted(ClimberConfig.follow_isInverted);
        //burn follower
        m_climberFollower.burnFlash();

        //set position to startup
        m_climberMaster.getEncoder().setPosition(ClimberPositions.zero);
        //control type to position
        pid_climberMaster.setReference(m_climberMaster.getEncoder().getPosition(), ControlType.kPosition);

        //BURN MASTER FLASH!!
        m_climberMaster.burnFlash();



    }//end Climber() constructor



    //returns encoder position reading
    public double getPosition()
    {
        return e_climberMaster.getPosition();

    }//end getPosition


    public void setPosition(double position)
    {
        pid_climberMaster.setReference(position, ClimberConfig.controlType);
    }





}

package frc.robot.Elevator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Elevator.ElevatorConfig;
import frc.robot.Elevator.ElevatorPositions;

public class Elevator extends SubsystemBase{
    
    //Elevator will be position mode

    //Elevator will be position mode

    //declare deck
    private CANSparkMax m_deckMaster;
    
    //declare deckmaster encoder
    private RelativeEncoder e_deckMaster;

    //declare master's pid controller
    private SparkPIDController pid_deckMaster;

    //Elevator Constructor method
    public Elevator()
    {

        // **INITIALIZE MOTORS**
        m_deckMaster = new CANSparkMax(ElevatorConfig.ID, ElevatorConfig.motorType);
                
        //Master motor Encoder for ease of access
        e_deckMaster = m_deckMaster.getEncoder();

        //pid deck master
        pid_deckMaster = m_deckMaster.getPIDController();
        
        //pid
        pid_deckMaster.setP(ElevatorConfig.p);
        pid_deckMaster.setI(ElevatorConfig.i);
        pid_deckMaster.setD(ElevatorConfig.d);
        pid_deckMaster.setIZone(ElevatorConfig.IZone);
        pid_deckMaster.setDFilter(ElevatorConfig.DFilter);
        
        //min/max output
        pid_deckMaster.setOutputRange(ElevatorConfig.outputMin, ElevatorConfig.outputMax);

        //Ramp Rate
        m_deckMaster.setOpenLoopRampRate(ElevatorConfig.openRampRate);
        m_deckMaster.setClosedLoopRampRate(ElevatorConfig.closedRampRate);
        
        // Is Follower needed for Master deck?

        //Inverted
        m_deckMaster.setInverted(ElevatorConfig.kInverted);

        //current limits
        m_deckMaster.setSmartCurrentLimit(ElevatorConfig.smartCurrentStallLimit, ElevatorConfig.smartCurrentFreeLimit);
        
        //Conversion Factor
        e_deckMaster.setPositionConversionFactor(ElevatorConfig.positionConversionFactor);
        e_deckMaster.setVelocityConversionFactor(ElevatorConfig.velocityConversionFactor);

        //soft limits Forward
        m_deckMaster.enableSoftLimit(SoftLimitDirection.kForward, ElevatorConfig.softLimitFwdEnabled);
        m_deckMaster.setSoftLimit(SoftLimitDirection.kForward, ElevatorConfig.softLimitFwd);

        //soft limits Reverse
        m_deckMaster.enableSoftLimit(SoftLimitDirection.kReverse, ElevatorConfig.softLimitRevEnabled);
        m_deckMaster.setSoftLimit(SoftLimitDirection.kReverse, ElevatorConfig.softLimitRev);
        
        //TODO: Maybe add HOLLOW BORE

        //control type to position
        pid_deckMaster.setReference(m_deckMaster.getEncoder().getPosition(), ControlType.kPosition);

        //BURN MASTER FLASH!!
        m_deckMaster.burnFlash();

        //set position to startup
        m_deckMaster.getEncoder().setPosition(ElevatorPositions.zero);
        //control type to position
        pid_deckMaster.setReference(m_deckMaster.getEncoder().getPosition(), ControlType.kPosition);

        //BURN MASTER FLASH!!
        m_deckMaster.burnFlash();



    }//end deck constructor





    //returns encoder position reading
    public double getPosition()
    {
        return e_deckMaster.getPosition();

    }//end getPosition


    //sets the goal position that it will move to
    public void setPosition(double position)
    {
        pid_deckMaster.setReference(position, ElevatorConfig.controlType);
    }

}

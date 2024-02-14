package frc.robot.Elevator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Elevator.Commands.AdoptTargetDistance;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;


public class Elevator extends SubsystemBase{
    

    //Elevator will be position mode

    //declare elevator
    private CANSparkMax m_elevatorMaster;
    
    //declare elevatormaster encoder
    private RelativeEncoder e_elevatorMaster;

    //declare master's pid controller
    private SparkPIDController pid_elevatorMaster;

    //whether intake is out or in
    public boolean intakeOut = false;


    //Elevator Constructor method
    public Elevator()
    {

        // **INITIALIZE MOTORS**
        m_elevatorMaster = new CANSparkMax(ElevatorConfig.ID, ElevatorConfig.motorType);
        m_elevatorMaster.setIdleMode(ElevatorConfig.idleMode);
                
        //Master motor Encoder for ease of access
        e_elevatorMaster = m_elevatorMaster.getEncoder();

        //pid elevator master
        pid_elevatorMaster = m_elevatorMaster.getPIDController();
        
        //pid
        pid_elevatorMaster.setP(ElevatorConfig.p);
        pid_elevatorMaster.setI(ElevatorConfig.i);
        pid_elevatorMaster.setD(ElevatorConfig.d);
        pid_elevatorMaster.setIZone(ElevatorConfig.IZone);
        pid_elevatorMaster.setDFilter(ElevatorConfig.DFilter);
        
        //min/max output
        pid_elevatorMaster.setOutputRange(ElevatorConfig.outputMin, ElevatorConfig.outputMax);

        //Ramp Rate
        m_elevatorMaster.setOpenLoopRampRate(ElevatorConfig.openRampRate);
        m_elevatorMaster.setClosedLoopRampRate(ElevatorConfig.closedRampRate);
        
        // Is Follower needed for Master elevator?

        //Inverted
        m_elevatorMaster.setInverted(ElevatorConfig.kInverted);

        //current limits
        m_elevatorMaster.setSmartCurrentLimit(ElevatorConfig.smartCurrentStallLimit, ElevatorConfig.smartCurrentFreeLimit);
        
        //Conversion Factor
        e_elevatorMaster.setPositionConversionFactor(ElevatorConfig.positionConversionFactor);
        e_elevatorMaster.setVelocityConversionFactor(ElevatorConfig.velocityConversionFactor);

        //soft limits Forward
        m_elevatorMaster.enableSoftLimit(SoftLimitDirection.kForward, ElevatorConfig.softLimitFwdEnabled);
        m_elevatorMaster.setSoftLimit(SoftLimitDirection.kForward, ElevatorConfig.softLimitFwd);

        //soft limits Reverse
        m_elevatorMaster.enableSoftLimit(SoftLimitDirection.kReverse, ElevatorConfig.softLimitRevEnabled);
        m_elevatorMaster.setSoftLimit(SoftLimitDirection.kReverse, ElevatorConfig.softLimitRev);
        
        //TODO: Maybe add HOLLOW BORE

        //control type to position
        pid_elevatorMaster.setReference(m_elevatorMaster.getEncoder().getPosition(), ControlType.kPosition);

        //BURN MASTER FLASH!!
        m_elevatorMaster.burnFlash();

        //set position to startup
        m_elevatorMaster.getEncoder().setPosition(ElevatorPositions.zero);
        //control type to position
        pid_elevatorMaster.setReference(m_elevatorMaster.getEncoder().getPosition(), ControlType.kPosition);

        //BURN MASTER FLASH!!
        m_elevatorMaster.burnFlash();



    }//end elevator constructor





    //returns encoder position reading
    public double getPosition()
    {
        return e_elevatorMaster.getPosition();

    }//end getPosition


    //sets the goal position that it will move to
    public void setPosition(double position)
    {
        pid_elevatorMaster.setReference(position, ElevatorConfig.controlType);
    }


    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Elevator AtPosition", AdoptTargetDistance.finished);
        SmartDashboard.putNumber("Elevator Position", getPosition());
    }



}

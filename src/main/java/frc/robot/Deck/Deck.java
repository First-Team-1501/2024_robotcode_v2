package frc.robot.Deck;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Deck.Commands.AdoptSetAngle;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

public class Deck extends SubsystemBase{
    
    //Deck will be position mode

    //declare deck Master & Slave
    private CANSparkMax m_deckMaster;
    
    //declare deckmaster encoder
    private RelativeEncoder e_deckMaster;

    //declare master's pid controller
    private SparkPIDController pid_deckMaster;

    //Deck Constructor method
    public Deck()
    {

        // **INITIALIZE MOTORS**
        m_deckMaster = new CANSparkMax(DeckConfig.ID, DeckConfig.motorType);
        

        m_deckMaster.setIdleMode(DeckConfig.idleMode);
        
                
        //Master motor Encoder for ease of access
        e_deckMaster = m_deckMaster.getEncoder();

        //pid deck master
        pid_deckMaster = m_deckMaster.getPIDController();
        
        //pid
        pid_deckMaster.setP(DeckConfig.p);
        pid_deckMaster.setI(DeckConfig.i);
        pid_deckMaster.setD(DeckConfig.d);
        pid_deckMaster.setIZone(DeckConfig.IZone);
        pid_deckMaster.setDFilter(DeckConfig.DFilter);
        
        //min/max output
        pid_deckMaster.setOutputRange(DeckConfig.outputMin, DeckConfig.outputMax);

        //Ramp Rate
        m_deckMaster.setOpenLoopRampRate(DeckConfig.openRampRate);
        m_deckMaster.setClosedLoopRampRate(DeckConfig.closedRampRate);
        
        // Is Follower needed for Master deck?

        //Inverted
        m_deckMaster.setInverted(DeckConfig.kInverted);

        //current limits
        m_deckMaster.setSmartCurrentLimit(DeckConfig.smartCurrentStallLimit, DeckConfig.smartCurrentFreeLimit);
        
        //Conversion Factor
        e_deckMaster.setPositionConversionFactor(DeckConfig.positionConversionFactor);

        //soft limits Forward
        m_deckMaster.enableSoftLimit(SoftLimitDirection.kForward, DeckConfig.softLimitFwdEnabled);
        m_deckMaster.setSoftLimit(SoftLimitDirection.kForward, DeckConfig.softLimitFwd);

        //soft limits Reverse
        m_deckMaster.enableSoftLimit(SoftLimitDirection.kReverse, DeckConfig.softLimitRevEnabled);
        m_deckMaster.setSoftLimit(SoftLimitDirection.kReverse, DeckConfig.softLimitRev);

        //ATTEMPT: set position as 0
        e_deckMaster.setPosition(DeckPositions.zero);

        //BURN MASTER FLASH!!
        m_deckMaster.burnFlash();
        
        //control type to position
        pid_deckMaster.setReference(e_deckMaster.getPosition(), ControlType.kPosition);

        



    }//end deck constructor





    //returns encoder position reading
    public double getPosition()
    {
        return e_deckMaster.getPosition();

    }//end getPosition


    //sets the goal position that it will move to
    public void setAngle(double position)
    {
        pid_deckMaster.setReference(DeckPositions.zero, DeckConfig.controlType);
    }


    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Deck AtPosition", AdoptSetAngle.finished);
        SmartDashboard.putNumber("Deck Position", getPosition());
        //System.out.println("DeckPeriodic run");
    }



}

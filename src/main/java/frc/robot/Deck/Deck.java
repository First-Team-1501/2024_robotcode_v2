package frc.robot.Deck;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

public class Deck extends SubsystemBase{
    
    //Deck will be position mode

    //declare deck Master & Slave
    private CANSparkMax m_deckMaster;
    private CANSparkMax m_deckFollower;
    
    //declare deckmaster encoder
    private RelativeEncoder e_deckMaster;

    //declare master's pid controller
    private SparkPIDController pid_deckMaster;

    //Deck Constructor method
    public Deck()
    {

        // **INITIALIZE MOTORS**
        m_deckMaster = new CANSparkMax(DeckConfig.ID, DeckConfig.motorType);
        m_deckFollower = new CANSparkMax(DeckConfig.followerID, DeckConfig.motorType);
                
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
        e_deckMaster.setVelocityConversionFactor(DeckConfig.velocityConversionFactor);

        //soft limits Forward
        m_deckMaster.enableSoftLimit(SoftLimitDirection.kForward, DeckConfig.softLimitFwdEnabled);
        m_deckMaster.setSoftLimit(SoftLimitDirection.kForward, DeckConfig.softLimitFwd);

        //soft limits Reverse
        m_deckMaster.enableSoftLimit(SoftLimitDirection.kReverse, DeckConfig.softLimitRevEnabled);
        m_deckMaster.setSoftLimit(SoftLimitDirection.kReverse, DeckConfig.softLimitRev);
        
        //TODO: Maybe add HOLLOW BORE

        //control type to position
        pid_deckMaster.setReference(m_deckMaster.getEncoder().getPosition(), ControlType.kPosition);

        //BURN MASTER FLASH!!
        m_deckMaster.burnFlash();


        //m_deckFollower is follower and inverted
        m_deckFollower.follow(m_deckMaster);
        m_deckFollower.setInverted(DeckConfig.follow_isInverted);
        //burn follower
        m_deckFollower.burnFlash();

        //set position to startup
        m_deckMaster.getEncoder().setPosition(DeckPositions.zero);
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
        pid_deckMaster.setReference(position, DeckConfig.controlType);
    }




}

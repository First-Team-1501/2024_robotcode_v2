package frc.robot.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    
    //Intake will be percent mode
    

        // ***  DECLARING MOTORS & SENSORS
    //DECLARE INTAKE MOTORS
    private CANSparkMax m_topIntake;
    private CANSparkMax m_bottomIntake;

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
        
        // Is Follower needed for Top Or Bottom Intake?

        //Inverted
        m_topIntake.setInverted(IntakeConfig.top_kInverted);

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

        //BURN FLASH!!
        m_topIntake.burnFlash();



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

        // Is Follower needed for Top Or Bottom Intake?

        //Inverted
        m_bottomIntake.setInverted(IntakeConfig.bottom_kInverted);

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

        //BURN FLASH!
        m_bottomIntake.burnFlash();


        //      INITIALIZE PHOTOEYES
        pe_noteInQueue = new DigitalInput(2);
        pe_notePresent = new DigitalInput(1);

    }//End Constructor for Intake




    //FIX? the photoeye functions below are assuming the sensor returns false when a piece is present

    //returns true if piece present, else it returns false
    public boolean is_piecePresent()
    { 
        
        if (!pe_notePresent.get())
        {
            return true;
        }
        else
        {
            return false;
        }//end if...piecePresent Method

    }//end piecePresent method

    
    //returns true if piece in queue, else false
    public boolean is_pieceInQueue()
    {
       if (!pe_noteInQueue.get())
        {
            return true;
        }
        else
        {
            return false;
        }//end if...pieceInQue Method
    }//end pieceInQueue method

    //end of photoeye sensing


    //**RUN TOP INTAKE Commands */

    //RUN Top intake IN
    public void run_topIntake()
    {
        m_topIntake.set(IntakeConfig.top_inSpeed);
        
    }//end run top intake method


    //STOP Top intake method
    public void stop_topIntake()
    {
        m_topIntake.set(0);
    }//end stop top intake method

    //OUT Top intake method
    public void out_topIntake()
    {
        m_topIntake.set(IntakeConfig.top_outSpeed);

    }//end top out method



    //**RUN TOP INTAKE Commands */

    
    //RUN Bottom intake method
    public void run_bottomIntake()
    {
        
        m_bottomIntake.set(IntakeConfig.bottom_inSpeed);

    }//end RUN Bottom intake in method

    //STOP Bottom intake method
    public void stop_bottomIntake()
    {
        m_bottomIntake.set(0);
    }//end STOP intake method

    //OUT Bottom intake
    public void out_bottomIntake()
    {
        m_bottomIntake.set(IntakeConfig.bottom_outSpeed);
    }//end Out intake Method



}

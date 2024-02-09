package frc.robot.Shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


public class Shooter extends SubsystemBase{
    
    //Shooter will be velocity mode (if we can get it working, if not then percent mode)


        // ***  DECLARING MOTORS & SENSORS
    //DECLARE Shooter MOTORS
    private CANSparkMax m_leftShooter;
    private CANSparkMax m_rightShooter;

    //DECLARE Photoeyes

    //CONSTRUCTOR Shooter
    public Shooter()
    {

        //INITIALIZE & CONFIGURE m_leftShooter
        m_leftShooter = new CANSparkMax(ShooterConfig.left_ID, ShooterConfig.left_motorType);
        m_leftShooter.setIdleMode(ShooterConfig.left_idleMode);

        //left motor Encoder for ease of access
        RelativeEncoder e_leftShooter = m_leftShooter.getEncoder();
        
        //pid
        m_leftShooter.getPIDController().setP(ShooterConfig.right_p);
        m_leftShooter.getPIDController().setI(ShooterConfig.left_i);
        m_leftShooter.getPIDController().setD(ShooterConfig.left_d);
        m_leftShooter.getPIDController().setIZone(ShooterConfig.left_IZone);
        m_leftShooter.getPIDController().setDFilter(ShooterConfig.left_DFilter);
        
        //min/max output
        m_leftShooter.getPIDController().setOutputRange(ShooterConfig.left_outputMin, ShooterConfig.left_outputMax);

        //Ramp Rate
        m_leftShooter.setOpenLoopRampRate(ShooterConfig.left_openRampRate);
        m_leftShooter.setClosedLoopRampRate(ShooterConfig.left_closedRampRate);
        
        // Is Follower needed for left Or right Shooter?

        //Inverted
        m_leftShooter.setInverted(ShooterConfig.left_kInverted);

        //current limits
        m_leftShooter.setSmartCurrentLimit(ShooterConfig.left_smartCurrentStallLimit, ShooterConfig.left_smartCurrentFreeLimit);
        
        //Conversion Factor
        e_leftShooter.setPositionConversionFactor(ShooterConfig.left_positionConversionFactor);
        e_leftShooter.setVelocityConversionFactor(ShooterConfig.left_velocityConversionFactor);

        //soft limits Forward
        m_leftShooter.enableSoftLimit(SoftLimitDirection.kForward, ShooterConfig.left_softLimitFwdEnabled);
        m_leftShooter.setSoftLimit(SoftLimitDirection.kForward, ShooterConfig.left_softLimitFwd);

        //soft limits Reverse
        m_leftShooter.enableSoftLimit(SoftLimitDirection.kReverse, ShooterConfig.left_softLimitRevEnabled);
        m_leftShooter.setSoftLimit(SoftLimitDirection.kReverse, ShooterConfig.left_softLimitRev);
        
        //TODO: Maybe add HOLLOW BORE

        //control type
                m_leftShooter.getPIDController().setReference(m_leftShooter.getEncoder().getPosition(), ControlType.kVelocity);


        //BURN FLASH!!
        m_leftShooter.burnFlash();



        //INITIALIZE & CONFIGURE m_rightShooter
            
        m_rightShooter = new CANSparkMax(ShooterConfig.ID, ShooterConfig.right_motorType);
        m_rightShooter.setIdleMode(ShooterConfig.right_idleMode);

        //right motor Encoder for ease of access
        RelativeEncoder e_rightShooter = m_rightShooter.getEncoder();
        
        //pid
        m_rightShooter.getPIDController().setP(ShooterConfig.right_p);
        m_rightShooter.getPIDController().setI(ShooterConfig.right_i);
        m_rightShooter.getPIDController().setD(ShooterConfig.right_d);
        m_rightShooter.getPIDController().setIZone(ShooterConfig.right_IZone);
        m_rightShooter.getPIDController().setDFilter(ShooterConfig.right_DFilter);
        
        //min/max output
        m_rightShooter.getPIDController().setOutputRange(ShooterConfig.right_outputMin, ShooterConfig.right_outputMax);

        //Ramp Rate
        m_rightShooter.setOpenLoopRampRate(ShooterConfig.right_openRampRate);
        m_rightShooter.setClosedLoopRampRate(ShooterConfig.right_closedRampRate);

        // Is Follower needed for left Or right Shooter?

        //Inverted
        m_rightShooter.setInverted(ShooterConfig.right_kInverted);

        //current limits
        m_rightShooter.setSmartCurrentLimit(ShooterConfig.right_smartCurrentStallLimit, ShooterConfig.right_smartCurrentFreeLimit);
        
        //Conversion Factor
        e_rightShooter.setPositionConversionFactor(ShooterConfig.right_positionConversionFactor);
        e_rightShooter.setVelocityConversionFactor(ShooterConfig.right_velocityConversionFactor);

        //soft limits Forward
        m_rightShooter.enableSoftLimit(SoftLimitDirection.kForward, ShooterConfig.right_softLimitFwdEnabled);
        m_rightShooter.setSoftLimit(SoftLimitDirection.kForward, ShooterConfig.right_softLimitFwd);

        //soft limits Reverse
        m_rightShooter.enableSoftLimit(SoftLimitDirection.kReverse, ShooterConfig.right_softLimitRevEnabled);
        m_rightShooter.setSoftLimit(SoftLimitDirection.kReverse, ShooterConfig.right_softLimitRev);
        
        //TODO: Maybe add HOLLOW BORE

        //control type
        m_rightShooter.getPIDController().setReference(0, ControlType.kVelocity);


        //BURN FLASH!
        m_rightShooter.burnFlash();

    }//End Constructor for Shooter


    //get velocity
    public double getVelocity()
    {
        return getVelocity();

    }//end get velocity


    //set velocity
    public void setSpeed(CANSparkMax motor, double shooterVelocity)
    {
        motor.set(shooterVelocity);
    }//end set velocity

    //auto-generated stub

    public void shoot(double leftShooterVelocity, double rightShooterVelocity)
    {
        setSpeed(m_leftShooter, leftShooterVelocity);
        setSpeed(m_rightShooter, rightShooterVelocity);

    }
    public void stop_shooter()
    {
        setSpeed(m_leftShooter, 0);
        setSpeed(m_rightShooter, 0);
    }



}

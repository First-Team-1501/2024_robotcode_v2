package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.Shooter;
import frc.robot.Shooter.ShooterSpeeds;
import frc.robot.Shooter.ShotList;

public class RevShooter extends Command
{
    private Shooter s_shooter;
    private boolean finished = false;
    private double rightSpeed;
    private double leftSpeed;

    public RevShooter(Shooter s_shooter, ShotList speed)
    {
        this.s_shooter = s_shooter;
        addRequirements(s_shooter);
        finished = false;

        switch (speed) {
            case CLOSEUP:
                rightSpeed = ShooterSpeeds.closeup_rightSpeed;
                leftSpeed = ShooterSpeeds.closeup_leftSpeed;
                break;
        
            case PODIUM:
                rightSpeed = ShooterSpeeds.podium_rightSpeed;
                leftSpeed = ShooterSpeeds.podium_leftSpeed;
                break;

            case BACKLINE:
                rightSpeed = ShooterSpeeds.podium_rightSpeed;
                leftSpeed = ShooterSpeeds.podium_leftSpeed;

            default:
                rightSpeed = ShooterSpeeds.closeup_rightSpeed;
                leftSpeed = ShooterSpeeds.closeup_leftSpeed;
                break;
        }


        System.out.println("Shooter Constructor");
    }//end shooter constructor

    @Override
    public void initialize() 
    {}//end initialize

    @Override
    public void execute() 
    {
        
        s_shooter.shoot(leftSpeed, rightSpeed);
        System.out.println("shooter run");

    }


   @Override
   public void end(boolean interrupted) {
       // TODO Auto-generated method stub
       s_shooter.stop_shooter();
       System.out.println("Shoot Stopped");
   }


    //TODO add finished
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub

        System.out.println("Shoot Finished");
        return false;
    }

    
}

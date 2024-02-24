package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShootParams;
import frc.robot.commands.deck.AutoDeckAim;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.intake.ShootNote;
import frc.robot.commands.sequential.IntakeSequence;
import frc.robot.commands.sequential.RetractIntakeSequence;
import frc.robot.commands.sequential.AutoShoot;
import frc.robot.commands.shooter.RevShooter;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.shooter.ShooterConfig;
import frc.robot.subsystems.shooter.ShooterSubsystem;


public class AutoCommands 
{
  
    private RobotContainer robot;

    public AutoCommands(RobotContainer robot)
    {
        this.robot = robot;

        // legacy commands

        


        NamedCommands.registerCommand("AutoAimRotate", robot.getDrivebase().driveCommand(
            ()->0, ()->0,
            () -> -robot.limelight_aim_proportional()));


        NamedCommands.registerCommand("getPiece",
           new IntakeSequence(robot.getIntake(), robot.getDeck(),robot.getElevator()));
        NamedCommands.registerCommand("shootCloseUp", 
            new AutoShoot(robot.getShooter(), robot.getDeck(), robot.getIntake(), ShootParams.CloseUp));
        NamedCommands.registerCommand("shootAuto2",
            new AutoShoot(robot.getShooter(), robot.getDeck(), robot.getIntake(), ShootParams.Auto2));
        NamedCommands.registerCommand("shootAuto1",     
                new AutoShoot(robot.getShooter(), robot.getDeck(), robot.getIntake(), ShootParams.Auto1));
        NamedCommands.registerCommand("shootAuto3",     
                new AutoShoot(robot.getShooter(), robot.getDeck(), robot.getIntake(), ShootParams.Auto3));
        NamedCommands.registerCommand("finishAuto", new InstantCommand(()->robot.getShooter().stop()));
        

        NamedCommands.registerCommand("shootAutoAim",
              
            new ParallelRaceGroup(
                new AutoDeckAim(robot.getDeck()),
                new RevShooter(robot.getShooter(), ShootParams.Podium.getLeftSpeed(), ShootParams.Podium.getRightSpeed()),
                new WaitCommand(1.5).andThen(new ShootNote(robot.getIntake())),
                new WaitCommand(3.5)
            ).andThen(
                new SetDeckPosition(robot.getDeck(), DeckPositions.home))
        );
        
        NamedCommands.registerCommand("shootPodium", 
            new AutoShoot(robot.getShooter(), robot.getDeck(), robot.getIntake(), ShootParams.Podium));
            //PathPlannerPath.fromPathFile("null")
        // NamedCommands.registerCommand("runIntake", 
        //  new RunIntakeCommand(robot.getIntake()));
        // NamedCommands.registerCommand("retractIntake",
        //  new RetractIntakeSequence(robot.getDeck(), robot.getElevator()));
       
    }

    public Command SelectAuto() 
    {
        switch(robot.getThumbwheel().getValue()%8)
        {

            case 1:               
                return robot.getDrivebase().getAutoCommand("Auto1");
            case 2:
                return robot.getDrivebase().getAutoCommand("Auto2");
            case 3:
                //return robot.getDrivebase().getAutoPath("BasicPathTest");
               return robot.getDrivebase().getAutoCommand("Auto3");
            case 4:
                return robot.getDrivebase().getAutoCommand("Auto4");
            case 6:
                return robot.getDrivebase().getAutoCommand("Auto6");
   
            default:
                return new InstantCommand();
   
        }
    }


  
}

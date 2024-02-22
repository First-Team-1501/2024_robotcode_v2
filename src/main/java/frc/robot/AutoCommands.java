package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Drivebase;
import frc.robot.Constants.ShootParams;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.intake.ShootNote;
import frc.robot.commands.sequential.IntakeSequence;
import frc.robot.commands.sequential.RetractIntakeSequence;
import frc.robot.commands.sequential.AutoShoot;
import frc.robot.commands.shooter.RevShooter;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.shooter.ShooterConfig;


public class AutoCommands 
{
  
    private RobotContainer robot;

    public AutoCommands(RobotContainer robot)
    {
        this.robot = robot;

        // legacy commands
        NamedCommands.registerCommand("revShooterCloseUp",
          new RevShooter(robot.getShooter(), ShooterConfig.closeLeftSpeed, ShooterConfig.closeRightSpeed)); 
        NamedCommands.registerCommand("deckHome",
          new SetDeckPosition(robot.getDeck(), DeckPositions.home));
        NamedCommands.registerCommand("deckCloseUp",
          new SetDeckPosition(robot.getDeck(), DeckPositions.closeup));
        NamedCommands.registerCommand("shootNote", 
           new ShootNote(robot.getIntake()));


        NamedCommands.registerCommand("getPiece",
           new IntakeSequence(robot.getIntake(), robot.getDeck(),robot.getElevator()));
        NamedCommands.registerCommand("shootCloseUp", 
            new AutoShoot(robot.getShooter(), robot.getDeck(), robot.getIntake(), ShootParams.CloseUp));
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
                return robot.getDrivebase().getAutoCommand("auto1");
            case 2:
                return robot.getDrivebase().getAutoCommand("testauto");
            case 3:
                //return robot.getDrivebase().getAutoPath("BasicPathTest");
                

            default:
                return new InstantCommand();
            
        }
    }


  
}

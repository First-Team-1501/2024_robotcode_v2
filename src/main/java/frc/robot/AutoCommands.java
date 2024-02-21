package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.intake.ShootNote;
import frc.robot.commands.sequential.IntakePositionSequence;
import frc.robot.commands.sequential.RetractIntakeSequence;
import frc.robot.commands.shooter.RevShooter;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.shooter.ShooterConfig;


public class AutoCommands 
{
  
    private RobotContainer robot;

    public AutoCommands(RobotContainer robot)
    {
        this.robot = robot;

        // register namedcommands here.
        NamedCommands.registerCommand("deckPreClimb",
         new SetDeckPosition(robot.getDeck(), DeckPositions.preClimb));
        NamedCommands.registerCommand("revShooterCloseup",
          new RevShooter(robot.getShooter(), ShooterConfig.closeLeftSpeed, ShooterConfig.closeRightSpeed)); 
        NamedCommands.registerCommand("deckHome",
          new SetDeckPosition(robot.getDeck(), DeckPositions.home));
        NamedCommands.registerCommand("deckCloseUp",
          new SetDeckPosition(robot.getDeck(), DeckPositions.closeup));
        NamedCommands.registerCommand("shootNote", 
           new ShootNote(robot.getIntake()));
        // NamedCommands.registerCommand("deployIntake",
        //  new IntakePositionSequence(robot.getDeck(), robot.getElevator()));
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
                return robot.getDrivebase().getAutoCommand("testauto");
            case 2:
               return robot.getDrivebase().getAutoCommand("Auto1");
            case 3:
                return robot.getDrivebase().getAutoPath("BasicPathTest");
            
            default:
                return new InstantCommand();
            
        }
    }


  
}

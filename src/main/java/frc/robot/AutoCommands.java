package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.subsystems.deck.DeckPositions;


public class AutoCommands 
{
  
    private RobotContainer robot;

    public AutoCommands(RobotContainer robot)
    {
        this.robot = robot;
        
       
    }

    public Command SelectAuto() 
    {
        switch(robot.getThumbwheel().getValue()%8)
        {
            
            case 1:
                return new SetDeckPosition(robot.getDeck(), DeckPositions.preClimb)
                    .andThen(new SetDeckPosition(robot.getDeck(), DeckPositions.home));
            case 2:
                return robot.getDrivebase().getAutonomousCommand("Path1", true);
            default:
                return new InstantCommand();
            
        }
    }


  
}

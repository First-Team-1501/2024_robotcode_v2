package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

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

        // register namedcommands here.
        NamedCommands.registerCommand("deckPreClimb",
         new SetDeckPosition(robot.getDeck(), DeckPositions.preClimb)); 
        NamedCommands.registerCommand("deckHome",
         new SetDeckPosition(robot.getDeck(), DeckPositions.home));

       
    }

    public Command SelectAuto() 
    {
        switch(robot.getThumbwheel().getValue()%8)
        {
            
            case 1:
                return new SetDeckPosition(robot.getDeck(), DeckPositions.preClimb)
                    .andThen(new SetDeckPosition(robot.getDeck(), DeckPositions.home));
            case 2:
                return new PathPlannerAuto("Auto1");
            default:
                return new InstantCommand();
            
        }
    }


  
}

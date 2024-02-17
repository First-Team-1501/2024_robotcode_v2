package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.subsystems.Thumbwheel;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.deck.DeckSubsystem;

public class AutoCommands 
{
    private Thumbwheel thumb;
    private RobotContainer robot;

    public AutoCommands(RobotContainer robot)
    {
        this.robot = robot;
        thumb = new Thumbwheel();
       
    }

    public Command SelectAuto() 
    {
        switch(thumb.getValue())
        {
            
            case 1:
                return new SetDeckPosition(robot.getDeck(), DeckPositions.preClimb)
                    .andThen(new SetDeckPosition(robot.getDeck(), DeckPositions.home));
            case 2:
                return test();
            default:
                return new InstantCommand();
            
        }
    }


    private Command test()
    {
        return null;
        //AutoBuilder.configureHolonomic(null, null, null, null, null, null, deck);
    }
}

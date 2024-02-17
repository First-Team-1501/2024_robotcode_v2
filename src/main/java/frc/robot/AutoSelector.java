package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.subsystems.Thumbwheel;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.deck.DeckSubsystem;

public class AutoSelector 
{
    private Thumbwheel thumb;
    

    public AutoSelector()
    {
        thumb = new Thumbwheel();
       
    }

    public Command SelectAuto( DeckSubsystem deck)
    {
        switch(thumb.getValue())
        {
            
            case 1:
                //
                return new SetDeckPosition(deck, DeckPositions.preClimb)
                    .andThen(new SetDeckPosition(deck, DeckPositions.home));
            default:
                return new InstantCommand();
            
        }
    }
}

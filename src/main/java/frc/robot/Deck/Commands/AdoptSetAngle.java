package frc.robot.Deck.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Deck.Deck;
import frc.robot.Deck.DeckPositions;
import frc.robot.Deck.PositionList;

public class AdoptSetAngle extends Command
{


    private Deck s_deck;
    private double deg_Position;

    public AdoptSetAngle(Deck s_deck, PositionList position)
    {
        this.s_deck = s_deck;
        switch (position) {
            case CLOSEUP:
                deg_Position = DeckPositions.closeup;
                break;



            case PODIUM:
                deg_Position = DeckPositions.podium;
                break;
            

            case BACKLINE:
                deg_Position = DeckPositions.backline;
                break;



            case INTAKE:
                deg_Position = DeckPositions.intake;
                break;
            
            
            
            case HOME:
                deg_Position = DeckPositions.home;
                break;
            
        
            default:
                deg_Position = DeckPositions.intake;
                break;



        }//end switch for position



        addRequirements(s_deck);
        
    }//constructor


    @Override
    public void initialize() {
        //TODO: is Finished, execute, and end methods
        
        s_deck.setPosition(deg_Position);
        
    }



}
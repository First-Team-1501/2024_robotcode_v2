package frc.robot.Deck.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Deck.Deck;
import frc.robot.Deck.DeckPositions;
import frc.robot.Deck.PositionList;

public class AdoptSetAngle extends Command
{


    private Deck s_deck;
    private double target_deg;
    private boolean finished = false;

    public AdoptSetAngle(Deck s_deck, PositionList position)
    {
        this.s_deck = s_deck;
        switch (position) {
            case CLOSEUP:
                target_deg = DeckPositions.closeup;
                break;



            case PODIUM:
                target_deg = DeckPositions.podium;
                break;
            

            case BACKLINE:
                target_deg = DeckPositions.backline;
                break;



            case INTAKE:
                target_deg = DeckPositions.intake;
                break;
            
            
            
            case HOME:
                target_deg = DeckPositions.home;
                break;
            
            case CLIMB:
                target_deg = DeckPositions.climb;
                break;
            
        
            default:
                target_deg = DeckPositions.intake;
                break;



        }//end switch for position



        addRequirements(s_deck);
        finished = false;
        
    }//constructor


    @Override
    public void initialize() {
        //TODO: is Finished, execute, and end methods
        
        s_deck.setPosition(target_deg);
        
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        //if difference between target position
        finished = Math.abs(target_deg - s_deck.getPosition()) < DeckPositions.tolerance;
        
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        
    }

    



}
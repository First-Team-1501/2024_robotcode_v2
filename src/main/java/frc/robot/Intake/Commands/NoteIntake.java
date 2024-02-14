package frc.robot.Intake.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Deck.Deck;
import frc.robot.Deck.Commands.AdoptSetAngle;
import frc.robot.Intake.Intake;

public class NoteIntake extends Command
{
    
    private Intake s_intake;
    private boolean finished = false;
    Deck s_DECK;

    public NoteIntake(Intake s_intake, Deck s_DECK)
    {
        
        this.s_intake = s_intake;
        addRequirements(s_intake);
        finished = false;
        this.s_DECK = s_DECK;

    }
    
    @Override
    public void initialize() 
    {}//end initialize

    @Override
    public void execute() 
    {
        /*if(!s_intake.is_pieceInQueue())
        {
            s_intake.run_intake();
        }
        else
        {
            finished = true;
        }*/
        if (!s_DECK.onBattery) {
            s_intake.run_intake();

        }
        else{
            
            //AdoptSetAngle moveDeck = new AdoptSetAngle(s_DECK, PositionList.PRE_INTAKE);
        }


    }


   @Override
   public void end(boolean interrupted) {
       // TODO Auto-generated method stub
       s_intake.stop_intake();
   }


    //TODO add finished
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub


        return finished;
    }



}

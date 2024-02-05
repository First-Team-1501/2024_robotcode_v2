package frc.robot.Intake.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.Intake;

public class NoteIntake extends Command
{
    
    private Intake s_intake;
    private boolean finished = false;

    public NoteIntake(Intake s_intake)
    {
        
        this.s_intake = s_intake;
        addRequirements(s_intake);
        finished = false;

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
        s_intake.run_intake();


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

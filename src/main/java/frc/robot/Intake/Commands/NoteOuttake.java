package frc.robot.Intake.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.Intake;

public class NoteOuttake extends Command
{
    
    private Intake s_intake;

    public NoteOuttake(Intake s_intake)
    {
        
        this.s_intake = s_intake;
        addRequirements(s_intake);

    }
    
    @Override
    public void initialize() 
    {
        
        

    }//end initialize

    @Override
    public void execute() 
    {

        s_intake.out_intake();        



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
        return super.isFinished();
    }



}

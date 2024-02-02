package frc.robot.Intake.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Intake.Intake;

public class NoteIntake extends Command
{
    
    private Intake s_intake;


    public NoteIntake(Intake s_intake)
    {
        
        this.s_intake = s_intake;
        addRequirements(s_intake);

    }
    
    @Override
    public void initialize() 
    {
        
        //Start the intake
        s_intake.run_bottomIntake();
        s_intake.run_topIntake();

    }//end initialize

    @Override
    public void execute() { }

    //TODO add finished

}

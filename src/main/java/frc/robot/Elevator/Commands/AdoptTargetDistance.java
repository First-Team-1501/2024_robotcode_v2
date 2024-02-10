package frc.robot.Elevator.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Elevator.DistanceList;
import frc.robot.Elevator.Elevator;
import frc.robot.Elevator.ElevatorPositions;

public class AdoptTargetDistance extends Command{
    
    Elevator s_Elevator;
    private double target_dist;
    public static boolean finished = false;

    
    public AdoptTargetDistance(Elevator s_Elevator, DistanceList distance)
    {

        this.s_Elevator = s_Elevator;
        //assign target distance based on desired state
        switch (distance) {
            case HOME:
                target_dist = ElevatorPositions.home;

                break;


            case INTAKE:
                target_dist = ElevatorPositions.intake;

                break;

            
            case AMP:
                target_dist = ElevatorPositions.amp;
                break;


            case CLIMB:
                target_dist = ElevatorPositions.climb;
                break;
            

            case TRAP_AIM:
                target_dist = ElevatorPositions.trap_aim;
                break;


            case TRAP_SCORE:
                target_dist = ElevatorPositions.trap_score;
                break;

            
            case ZERO:
                target_dist = ElevatorPositions.zero;
                break;

        
            default:
                target_dist = ElevatorPositions.home;
                break;
        }
        finished = false;
        System.out.println("Elevator position initialized");


    }//end constructor

    @Override
    public void initialize() 
    {
        s_Elevator.setPosition(target_dist);
        System.out.println("Elevator position set");

    }


    @Override
    public void execute() 
    {
        finished = Math.abs(target_dist-s_Elevator.getPosition())<ElevatorPositions.tolerance;
        System.out.println("Elevator position checked");

    }
    @Override
    public boolean isFinished() {
        return finished;

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Elevator At position");
    }


}

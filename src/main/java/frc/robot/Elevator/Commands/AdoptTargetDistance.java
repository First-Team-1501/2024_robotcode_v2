package frc.robot.Elevator.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Elevator.DistanceList;
import frc.robot.Elevator.Elevator;
import frc.robot.Elevator.ElevatorPositions;
import frc.robot.Deck.Deck;
import frc.robot.Deck.PositionList;
import frc.robot.Deck.Commands.AdoptSetAngle;;

public class AdoptTargetDistance extends Command{
    
    Elevator s_Elevator;
    private double target_dist;
    public static boolean finished = false;

    
    public AdoptTargetDistance(Elevator s_Elevator, Deck s_Deck, DistanceList distance)
    {

        

        this.s_Elevator = s_Elevator;

        //check to see if elevator is extending or not
        if (distance == DistanceList.HOME || distance == DistanceList.ZERO)
        {
            s_Elevator.intakeOut = false;
        }
        else
        {
            s_Elevator.intakeOut = true;
        }//end set elevatorOut

        //assign target distance based on desired state
        switch (distance) {
            case HOME:
                
                if(!s_Deck.onBattery)
                {
                    target_dist = ElevatorPositions.home;
                }
                else 
                {
                    Command angleUp = new AdoptSetAngle(s_Deck, PositionList.PRE_INTAKE);
                    target_dist = ElevatorPositions.home;

                }
                
                break;


            case INTAKE:
                if(!s_Deck.onBattery)
                {
                    target_dist = ElevatorPositions.intake;
                }
                else 
                {
                    Command angleUp = new AdoptSetAngle(s_Deck, PositionList.PRE_INTAKE);
                    target_dist = ElevatorPositions.intake;

                }
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
                if(!s_Deck.onBattery)
                {
                    target_dist = ElevatorPositions.zero;
                }
                else 
                {
                    Command angleUp = new AdoptSetAngle(s_Deck, PositionList.PRE_INTAKE);
                    target_dist = ElevatorPositions.zero;

                }
                break;

        
            default:
                if(!s_Deck.onBattery)
                {
                    target_dist = ElevatorPositions.home;
                }
                else 
                {
                    Command angleUp = new AdoptSetAngle(s_Deck, PositionList.PRE_INTAKE);
                    target_dist = ElevatorPositions.home;

                }
                break;
        }
        finished = false;
        System.out.println("Elevator position initialized");
        addRequirements(s_Elevator);

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

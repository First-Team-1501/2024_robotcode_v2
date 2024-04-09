// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequential;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.deck.SetDeckPosition;
import frc.robot.commands.elevator.SetElevatorAmpLimit;
import frc.robot.commands.elevator.SetElevatorMaxOutput;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.intake.IndexNote;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.swervedrive.drivebase.NormalDrive;
import frc.robot.commands.swervedrive.drivebase.NoteAutoAimRobot;
import frc.robot.subsystems.deck.DeckPositions;
import frc.robot.subsystems.deck.DeckSubsystem;
import frc.robot.subsystems.elevator.ElevatorPositions;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeSequenceAutoAim extends SequentialCommandGroup {
    /** Creates a new IntakeSequenceTeleop. */
    public IntakeSequenceAutoAim(IntakeSubsystem intake, DeckSubsystem deck, ElevatorSubsystem elevator, Leds leds,
            SwerveSubsystem drivebase, CommandJoystick driveStick, CommandJoystick rotStick) {
        addCommands(
                new NoteAutoAimRobot(drivebase, driveStick, rotStick, intake)
                        .alongWith(
                                new SetDeckPosition(deck, DeckPositions.home)
                                        .andThen(
                                                new SetElevatorPosition(elevator, ElevatorPositions.intake))
                                        .andThen(
                                                new SetDeckPosition(deck, DeckPositions.intake)
                                                        .alongWith(
                                                                new RunIntakeCommand(intake, leds))))
        );
                
    }
}

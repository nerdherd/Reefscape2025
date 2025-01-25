package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class SuperSystem {
    public CoralWrist intakeWrist;
    public AlgaeRoller intakeRoller;
    public Elevator elevator;
    public ElevatorPivot elevatorPivot;

    public SuperSystem(CoralWrist intakeWrist, AlgaeRoller intakeRoller,
                       Elevator elevator, ElevatorPivot elevatorPivot) {
        this.intakeWrist = intakeWrist;
        this.intakeRoller = intakeRoller;
        this.elevator = elevator;
        this.elevatorPivot = elevatorPivot;
    }

    public Command intakeCoralStation() {
        Command command = Commands.sequence(
            elevatorPivot.moveToStart(),
            elevator.moveToStation(),
            intakeWrist.moveToStation(),
            intakeRoller.intake()
        );
        command.addRequirements(elevator, elevatorPivot, intakeWrist, intakeRoller);
        return command;
    }

    public Command intakeCoralGround() { // TODO finish this
        Command command = Commands.sequence(
            // elevatorPivot.moveToStart(),
            // elevator.moveToStation(),
            // intakeWrist.moveToStation(),
            // intakeRoller.intake()
        );
        command.addRequirements(elevator, elevatorPivot, intakeWrist, intakeRoller);
        return command;
    }

    public Command intakeAlgaeGround() { // TODO finish this
        Command command = Commands.sequence(
            // elevatorPivot.moveToStart(),
            // elevator.moveToStation(),
            // intakeWrist.moveToStation(),
            // intakeRoller.intake()
        );
        command.addRequirements(elevator, elevatorPivot, intakeWrist, intakeRoller);
        return command;
    }

    public Command placeCoralL1() {
        Command command = Commands.sequence(
            elevatorPivot.moveToStart(),
            elevator.moveToReefL1(),
            intakeWrist.moveToReefL14(),
            intakeRoller.outtake()
        );
        command.addRequirements(elevator, elevatorPivot, intakeWrist, intakeRoller);
        return command;
    }

    public Command placeCoralL2() {
        Command command = Commands.sequence(
            elevatorPivot.moveToStart(),
            elevator.moveToReefL2(),
            intakeWrist.moveToReefL23(),
            intakeRoller.outtake()
        );
        command.addRequirements(elevator, elevatorPivot, intakeWrist, intakeRoller);
        return command;
    }

    public Command placeCoralL3() {
        Command command = Commands.sequence(
            elevatorPivot.moveToStart(),
            elevator.moveToReefL3(),
            intakeWrist.moveToReefL23(),
            intakeRoller.outtake()
        );
        command.addRequirements(elevator, elevatorPivot, intakeWrist, intakeRoller);
        return command;
    }

    public Command placeCoralL4() {
        Command command = Commands.sequence(
            elevatorPivot.moveToStart(),
            elevator.moveToReefL4(),
            intakeWrist.moveToReefL14(),
            intakeRoller.outtake()
        );
        command.addRequirements(elevator, elevatorPivot, intakeWrist, intakeRoller);
        return command;
    }

    public Command placeAlgaeProcessor() { // TODO finish this
        Command command = Commands.sequence(
            // elevatorPivot.moveToStow(),
            // elevator.moveToReefL1(),
            // intakeWrist.moveToStation(),
            // intakeRoller.outtake()
        );
        command.addRequirements(elevator, elevatorPivot, intakeWrist, intakeRoller);
        return command;
    }

    public Command collapse() {
        Command command = Commands.sequence(
            intakeRoller.stop(),
            intakeWrist.moveToStow(),
            elevator.stow(),
            elevatorPivot.moveToStart()
        );
        command.addRequirements(elevator, elevatorPivot, intakeWrist, intakeRoller);
        return command;
    }

    public Command stow() {
        Command command = Commands.sequence(
            intakeRoller.stop(),
            intakeWrist.moveToStow(),
            elevator.stow(),
            elevatorPivot.moveToStow()
        );
        command.addRequirements(elevator, elevatorPivot, intakeWrist, intakeRoller);
        return command;
    }
}

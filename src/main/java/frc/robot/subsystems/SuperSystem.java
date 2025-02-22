package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SuperSystem {
    public IntakeWrist wrist;
    public IntakeRoller roller;
    public Elevator elevator;
    public ElevatorPivot pivot;

    public SuperSystem(IntakeWrist wrist, IntakeRoller roller, Elevator elevator, ElevatorPivot pivot) {
        this.wrist = wrist;
        this.roller = roller;
        this.elevator = elevator;
        this.pivot = pivot;
        this.pivot.elevatorPosition = () -> this.elevator.getPosition();
    }

    // Intake Commands
    public Command intakeCoralStation() {
        Command command = Commands.sequence(
            pivot.moveToStart(),
            elevator.moveToStation(),
            wrist.moveToStation(),
            roller.intake()
        );
        command.addRequirements(elevator, pivot, wrist, roller);
        return command;
    }
    public Command intakeCoralGround() {
        Command command = Commands.sequence(
            pivot.moveToPickup(),
            elevator.stow(),
            wrist.moveToPickup(),
            roller.intake()
        );
        command.addRequirements(elevator, pivot, wrist, roller);
        return command;
    }
    public Command intakeAlgaeGround() { // TODO is this real
        Command command = Commands.sequence(
            pivot.moveToStart(),
            elevator.moveToStation(),
            wrist.moveToStation(),
            roller.intake()
        );
        command.addRequirements(elevator, pivot, wrist, roller);
        return command;
    }

    // OUttake Commands
    public Command placeCoralL1() {
        Command command = Commands.sequence(
            pivot.moveToStart(),
            elevator.moveToReefL1(),
            wrist.moveToReefL13()
        );
        command.addRequirements(elevator, pivot, wrist);
        return command;
    }
    public Command placeCoralL2() {
        Command command = Commands.sequence(
            pivot.moveToStart(),
            elevator.moveToReefL2(),
            wrist.moveToReefL24()
        );
        command.addRequirements(elevator, pivot, wrist);
        return command;
    }
    
    public Command placeCoralL3() {
        Command command = Commands.sequence(
            pivot.moveToStart(),
            elevator.moveToReefL3(),
            wrist.moveToReefL13()
        );
        command.addRequirements(elevator, pivot, wrist);
        return command;
    }

    public Command placeCoralL4() {
        Command command = Commands.sequence(
            pivot.movetoL4(),
            elevator.moveToReefL4(),
            wrist.moveToReefL24()
        );
        command.addRequirements(elevator, pivot, wrist);
        return command;
    }

    // public Command placeAlgaeProcessor() { // TODO finish this
    //     Command command = Commands.sequence(
    //         pivot.moveToStow(),
    //         elevator.moveToReefL1(),
    //         wrist.moveToStation(),
    //         roller.outtake()
    //     );
    //     command.addRequirements(elevator, pivot, wrist, roller);
    //     return command;
    // }

    public Command stow() {
        Command command = Commands.sequence(
            roller.stop(),
            wrist.stow(),
            elevator.stow(),
            pivot.stow()
        );
        command.addRequirements(elevator, pivot, wrist, roller);
        return command;
    }
}
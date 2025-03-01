package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.V1ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.SuperSystemConstants.NamedPositions;
import frc.robot.commands.SuperSystemCommand;
import frc.robot.commands.SuperSystemCommand.ExecutionOrder;

public class SuperSystem {
    public Elevator elevator;
    public ElevatorPivot pivot;
    public IntakeWrist wrist;
    public IntakeV2 claw;

    public SuperSystem(Elevator elevator, ElevatorPivot pivot, IntakeWrist wrist, IntakeV2 claw) {
        this.elevator = elevator;
        this.pivot = pivot;
        this.wrist = wrist;
        this.claw = claw;
    }

    public Command zeroEncoders() {
        return Commands.runOnce(()-> {
            pivot.zeroEncoder();
            elevator.zeroEncoder();
            wrist.zeroEncoder();
            claw.zeroEncoder();
        });
    }

    // public Command moveTo(NamedPositions position) {
    //     SuperSystemCommand superSystemCommand = 
    //         new SuperSystemCommand(pivot, elevator, wrist, 
    //             position.pivotPosition, position.elevatorPosition, position.intermediateWristPosition, 
    //             position.executionOrder, 10.0);
    //     if (position.intermediateWristPosition == position.finalWristPosition) return superSystemCommand;
    //     return Commands.sequence(
    //       superSystemCommand,
    //       wrist.setPositionCommand(position.finalWristPosition),
    //       wrist.setEnabledCommand(true)
    //     );
    // }

    // public Command moveToStow() {
    //     SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
    //     V1ElevatorConstants.kElevatorPivotStowPosition, ElevatorConstants.kElevatorStowPosition, WristConstants.kIntermediatePosition, 
    //     ExecutionOrder.WRT_ELV_PVT, 10.0);

    //     return Commands.sequence(
    //         superSystemCommand,
    //         wrist.setPositionCommand(WristConstants.kStowPosition),
    //         wrist.setEnabledCommand(true)
    //     );
    // }

    // public Command moveToSemiStow() { //TODO: see if this is legit
    //     SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
    //     V1ElevatorConstants.kElevatorPivotStationPosition, ElevatorConstants.kElevatorStowPosition, WristConstants.kIntermediatePosition, 
    //     ExecutionOrder.WRT_ELV_PVT, 10.0);

    //     return superSystemCommand;
    // }

    public Command moveToStation() {
        SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
        V1ElevatorConstants.kElevatorPivotStationPosition, ElevatorConstants.kElevatorStationPosition, WristConstants.kIntermediatePosition, 
        ExecutionOrder.WRT_PVT_ELV, 10.0,
        () -> pivot.atPosition(), () -> elevator.atPosition(), () -> wrist.atPosition()
        );

        return Commands.sequence(
            Commands.runOnce(() -> superSystemCommand.initialize()),
            Commands.run(() -> superSystemCommand.execute()),
                // Commands.run(() -> superSystemCommand.isFinished())
            wrist.setPositionCommand(WristConstants.kStationPosition),
            wrist.setEnabledCommand(true)
        );
    }

    // public Command moveToL1() {
    //     SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
    //     V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL1Position, WristConstants.kWristL1Position, 
    //     ExecutionOrder.ALL_TOGETHER, 10.0);

    //     return superSystemCommand;

        
    // }

    // public Command moveToL2() {
    //     // SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
    //     // V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL2Position, WristConstants.kL23Position, 
    //     // ExecutionOrder.ALL_TOGETHER, 10.0);

    //     // return superSystemCommand;

    //     // TODO Check kElevatorPivotPositionVertical is reasonable
    //     // TODO Wrist intermediate @ -0.1
    //     SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
    //     V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL2Position, WristConstants.kIntermediatePosition, 
    //     ExecutionOrder.WRT_PVT_ELV, 10.0);

    //     return Commands.sequence(
    //         superSystemCommand,
    //         wrist.setPositionCommand(WristConstants.kWristL2Position),
    //         wrist.setEnabledCommand(true)
    //     );
    // }

    // public Command moveToL3() {
    //     SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
    //     V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL3Position, WristConstants.kWristL3Position, 
    //     ExecutionOrder.ALL_TOGETHER, 10.0);

    //     return superSystemCommand;
    // }

    // public Command moveToL4() {
    //     SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
    //     V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL4Position, WristConstants.kWristL4Position, 
    //     ExecutionOrder.ALL_TOGETHER, 10.0);

    //     return superSystemCommand;
    // }

    // public Command moveTogroundIntake() {
    //     SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, V1ElevatorConstants.kElevatorPivotGroundIntake, ElevatorConstants.kElevatorGroundIntake, WristConstants.kWristGroundIntake, ExecutionOrder.ELV_WRT_PVT, 10);
    //     return superSystemCommand;
    // }
}

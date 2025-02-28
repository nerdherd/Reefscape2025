package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.V1ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.SuperSystemCommand;
import frc.robot.commands.SuperSystemCommand.ExecutionOrder;

public class SuperSystem {
    public Elevator elevator;
    public ElevatorPivot pivot;
    public IntakeWrist wrist;

    public SuperSystem(Elevator elevator, ElevatorPivot pivot, IntakeWrist wrist) {
        this.elevator = elevator;
        this.pivot = pivot;
        this.wrist = wrist;
    }

    public Command moveToStow() {
        SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
        V1ElevatorConstants.kElevatorPivotStowPosition, ElevatorConstants.kElevatorStowPosition, WristConstants.kIntermediatePosition, 
        ExecutionOrder.WRT_ELV_PVT, 10.0);

        return Commands.sequence(
            superSystemCommand,
            wrist.setPositionCommand(WristConstants.kStowPosition),
            wrist.setEnabledCommand(true)
        );
    }

    public Command moveToSemiStow() { //TODO: see if this is legit
        SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
        V1ElevatorConstants.kElevatorPivotStationPosition, ElevatorConstants.kElevatorStowPosition, WristConstants.kIntermediatePosition, 
        ExecutionOrder.WRT_ELV_PVT, 10.0);

        return superSystemCommand;
    }

    public Command moveToStation() {
        SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
        V1ElevatorConstants.kElevatorPivotStationPosition, ElevatorConstants.kElevatorStationPosition, WristConstants.kIntermediatePosition, 
        ExecutionOrder.WRT_PVT_ELV, 10.0);

        return Commands.sequence(
            superSystemCommand,
            wrist.setPositionCommand(WristConstants.kStationPosition),
            wrist.setEnabledCommand(true)
        );
    }

    public Command moveToL1() {
        SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
        V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL1Position, WristConstants.kL14Position, 
        ExecutionOrder.ALL_TOGETHER, 10.0);

        return superSystemCommand;
    }

    public Command moveToL2() {
        SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
        V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL2Position, WristConstants.kL23Position, 
        ExecutionOrder.ALL_TOGETHER, 10.0);

        return superSystemCommand;
    }

    public Command moveToL3() {
        SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
        V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL3Position, WristConstants.kL23Position, 
        ExecutionOrder.ALL_TOGETHER, 10.0);

        return superSystemCommand;
    }

    public Command moveToL4() {
        SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
        V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL4Position, WristConstants.kL14Position, 
        ExecutionOrder.ALL_TOGETHER, 10.0);

        return superSystemCommand;
    }
}

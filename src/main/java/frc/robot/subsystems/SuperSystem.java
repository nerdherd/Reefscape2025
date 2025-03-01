package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.V1ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.SuperSystemCommand;
import frc.robot.commands.SuperSystemCommand.ExecutionOrder;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class SuperSystem {
    public Elevator elevator;
    public ElevatorPivot pivot;
    public IntakeWrist wrist;
    public SwerveDrivetrain swerve;

    public SuperSystem(Elevator elevator, ElevatorPivot pivot, IntakeWrist wrist, SwerveDrivetrain swerve) {
        this.elevator = elevator;
        this.pivot = pivot;
        this.wrist = wrist;
        this.swerve = swerve;
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
        // SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
        // V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL2Position, WristConstants.kL23Position, 
        // ExecutionOrder.ALL_TOGETHER, 10.0);

        // return superSystemCommand;

        // TODO Check kElevatorPivotPositionVertical is reasonable
        // TODO Wrist intermediate @ -0.1
        SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
        V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL2Position, WristConstants.kIntermediatePosition, 
        ExecutionOrder.WRT_PVT_ELV, 10.0);

        return Commands.sequence(
            superSystemCommand,
            wrist.setPositionCommand(WristConstants.kL23Position),
            wrist.setEnabledCommand(true)
        );
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

    //Equation used found by Zachary Martinez
    //https://www.desmos.com/calculator/q70q2ekunm

    public Command moveLeftOf(int tagID) {
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        Rotation2d tagRotation = layout.getTagPose(tagID).get().toPose2d().getRotation();
        Rotation2d tagRotationInverse = new Rotation2d(-tagRotation.getRadians());
        Double theta_0 = tagRotationInverse.getRadians();
        Double moveBy = 1.0; //TODO: Change Later
        // D_x = R_x + M cos (theta_0)
        // D_y = R_y + M sin (theta_0)
        Transform2d transformer = new Transform2d((moveBy * Math.cos(theta_0)), (moveBy * Math.sin(theta_0)), tagRotation);
        return swerve.driveToRelativePose(00, 00, transformer);
    }

    public Command moveRightOf(int tagID) {
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        Rotation2d tagRotation = layout.getTagPose(tagID).get().toPose2d().getRotation();
        Rotation2d tagRotationInverse = new Rotation2d(-tagRotation.getRadians());
        Double theta_0 = tagRotationInverse.getRadians();
        Double moveBy = -1.0; //TODO: Change Later
        // D_x = R_x + M cos (theta_0)
        // D_y = R_y + M sin (theta_0)
        Transform2d transformer = new Transform2d((moveBy * Math.cos(theta_0)), (moveBy * Math.sin(theta_0)), tagRotation);
        return swerve.driveToRelativePose(00, 00, transformer);
    }
}

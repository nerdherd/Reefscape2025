package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.util.filters.OldDriverFilter2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

public class AutoCommand extends Command {
    private final Pose2d startPoseDefault = new Pose2d(0,0, new Rotation2d());
    private final Pose2d targetPose1;
    private final Pose2d targetPose2;
    private final SwerveDrivetrain drivetrain;

    private SequentialCommandGroup fullCommand;

    public AutoCommand(
        SwerveDrivetrain drivetrain,
        Pose2d targetPose1,
        Pose2d targetPose2
    ) {
        this.drivetrain = drivetrain;
        this.targetPose1 = targetPose1;
        this.targetPose2 = targetPose2;
    }

    @Override
    public void initialize() {
        // Get the robot's current pose
        Pose2d currentPose = drivetrain.getEstimatedPose();
        
        // To be added: validate the starting pose

        // if (RobotContainer.IsRedSide()) {
        //     (FlippingUtil.flipFieldPose(path1));
        // } else {
        //     (path1);
        // }

        // Generate the first path (current pose to target pose 1)
        Command path1 = AutoBuilder.pathfindToPose(
            targetPose1,            // End pose
            new PathConstraints(2.0, 1.0, 0,0), // Max velocity and acceleration
            0
        );

        // Generate the second path (target pose 1 to target pose 2)
        Command path2 = AutoBuilder.pathfindToPose(
            targetPose2,            // End pose
            new PathConstraints(2.0, 1.0, 0,0), // Max velocity and acceleration
            0
        );

        // Create a sequential command group
        fullCommand = new SequentialCommandGroup(
            // Commands.runOnce(swerve.getImu()::zeroAll),
            // Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            path1, // Follow path 1
            new WaitCommand(1.0), // Wait for 1 second
            path2 // Follow path 2
        );

        // Schedule the full command
        fullCommand.schedule();
    }

    @Override
    public void execute() {
        // Nothing to do here; the command group handles everything
    }

    @Override
    public void end(boolean interrupted) {
        // Cancel the command group if the AutoCommand is interrupted
        if (interrupted && fullCommand != null) {
            fullCommand.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        // The AutoCommand is finished when the command group is finished
        return fullCommand == null || fullCommand.isFinished();
    }
}

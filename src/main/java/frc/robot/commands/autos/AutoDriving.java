package frc.robot.commands.autos;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;

import org.json.simple.parser.ParseException;

import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;

public class AutoDriving extends SequentialCommandGroup {
    public AutoDriving(SwerveDrivetrain swerve, String autoPath) 
    throws IOException, ParseException {
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);
        Pose2d startingPose = pathGroup.get(0).getStartingDifferentialPose();

        addCommands(
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            Commands.runOnce(swerve.getImu()::zeroAll)
        );

        for (PathPlannerPath path : pathGroup) {
            addCommands(AutoBuilder.followPath(path));
        }
    }

    public static Command stopDriving(IntakeRoller roller, IntakeWrist wrist, Elevator elevator, ElevatorPivot pivot) {
        return Commands.sequence(
            roller.stop(),
            wrist.moveToStow(),
            elevator.stow(),
            pivot.moveToStow()
        );
    }
}

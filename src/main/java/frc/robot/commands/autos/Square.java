package frc.robot.commands.autos;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import frc.robot.subsystems.AlgaeRoller;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;

public class Square extends SequentialCommandGroup {
    public Square(SwerveDrivetrain swerve, AlgaeRoller intakeRoller, String autoPath)
    throws IOException, ParseException {
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);

        Pose2d startingPose = pathGroup.get(0).getStartingDifferentialPose();

        addCommands(
            // Commands.runOnce(swerve.getImu()::zeroAll),
            // Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            // Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            Commands.sequence(
                intakeRoller.intake(),
                AutoBuilder.followPath(pathGroup.get(0)),
                Commands.waitSeconds(0.5),
                intakeRoller.stop(),
                AutoBuilder.followPath(pathGroup.get(1)),
                Commands.waitSeconds(0.5),
                intakeRoller.outtake(),
                AutoBuilder.followPath(pathGroup.get(2)),
                Commands.waitSeconds(0.5),
                intakeRoller.stop(),
                AutoBuilder.followPath(pathGroup.get(3)),
                Commands.waitSeconds(0.5)
            )
        );
    }
}

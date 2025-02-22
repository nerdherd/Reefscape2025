package frc.robot.commands.autos;

import java.util.List;

import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;

public class PreloadTaxi extends SequentialCommandGroup{
    public PreloadTaxi(SwerveDrivetrain swerve, List<PathPlannerPath> pathGroup){
        Pose2d startingPose = pathGroup.get(0).getStartingDifferentialPose();

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            Commands.deadline(
                AutoBuilder.followPath(pathGroup.get(0)),
                Commands.waitSeconds(4.5)
            )
        );
    }
}

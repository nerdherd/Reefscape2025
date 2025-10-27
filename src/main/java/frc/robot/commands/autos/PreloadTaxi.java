package frc.robot.commands.autos;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;
import frc.robot.Constants.SuperSystemConstants.PositionEquivalents;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PreloadTaxi extends SequentialCommandGroup{
    public PreloadTaxi(SwerveDrivetrain swerve, String autoname, SuperSystem superSystem) throws IOException, ParseException{

        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoname);

        Pose2d startingPose = pathGroup.get(0).getStartingDifferentialPose();
        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.waitSeconds(0.1),
            // Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            // Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),

            // Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)), //10/25/25
            // Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            // Commands.runOnce(() -> swerve.resetOdometry(new Pose2d(startingPose.getTranslation(), Rotation2d.fromDegrees(gyro.))),
            // Commands.runOnce(() -> swerve.resetGyroFromPose(startingPose)),
            Commands.sequence(
                superSystem.moveToAuto(PositionEquivalents.Stow),
                AutoBuilder.followPath(pathGroup.get(0)),
                // swerve.driveToTagCommand(VisionConstants.kLimelightBackRightName), 
                Commands.waitSeconds(0.5),
                superSystem.moveTo(PositionEquivalents.L4),
                // superSystem.moveToAuto(PositionEquivalents.L1),
                Commands.waitSeconds(1),
                superSystem.outtake(),
                Commands.waitSeconds(1),
                superSystem.stopRoller(),
                superSystem.moveTo(PositionEquivalents.L1),
                AutoBuilder.followPath(pathGroup.get(1))

            )
        );
    }
}

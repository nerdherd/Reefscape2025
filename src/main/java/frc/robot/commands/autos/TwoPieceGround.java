package frc.robot.commands.autos;

import java.io.IOException;
import java.nio.file.attribute.PosixFilePermission;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.SuperSystemConstants.PositionEquivalents;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class TwoPieceGround extends SequentialCommandGroup {
    public TwoPieceGround(SwerveDrivetrain swerve, String autoname, SuperSystem superSystem) throws IOException, ParseException {
        
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoname);
        Pose2d startingPose = pathGroup.get(0).getStartingDifferentialPose();

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            
            Commands.sequence(
                // Move to preload
                // superSystem.holdPiece(),
                AutoBuilder.followPath(pathGroup.get(0)),
                Commands.parallel(
                    swerve.driveToTagCommand(VisionConstants.kLimelightBackRightName).withTimeout(2),
                    superSystem.moveToAuto(PositionEquivalents.L1)
                ),

                // Place preload L4
                // superSystem.moveToAuto(PositionEquivalents.L4),
                superSystem.moveToAuto(PositionEquivalents.L1),
                Commands.waitSeconds(0.5),
                // superSystem.outtake(),
                Commands.waitSeconds(1.5),
                // superSystem.stopRoller(),

                // Move to ground coral
                superSystem.moveToAuto(PositionEquivalents.L1),
                
                AutoBuilder.followPath(pathGroup.get(1)),

                // Intake ground coral
                superSystem.moveTo(PositionEquivalents.GroundIntake),
                superSystem.moveTo(PositionEquivalents.Stow),
                Commands.race(
                    // swerve.driveToCoralCommand("limelight-coral", 8),
                    AutoBuilder.followPath(pathGroup.get(2))
                    // superSystem.intake(),
                    // superSystem.intakeUntilSensed(2)
                ),
                // superSystem.holdPiece()

                // Move to A3OO
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(3)),
                    superSystem.moveToAuto(PositionEquivalents.L1)
                ),
                
                // Move to L4
                swerve.driveToTagCommand(VisionConstants.kLimelightBackRightName).withTimeout(2),
                superSystem.moveToAuto(PositionEquivalents.L1),
                // superSystem.moveToAuto(PositionEquivalents.L4),
                // Commands.runOnce(() -> swerve.setAutoPathRun(1, () -> true)).withTimeout(2),

                // Outtake
                Commands.sequence(
                    // superSystem.outtake(),
                    Commands.waitSeconds(2)
                    // superSystem.stopRoller()
                )
                )
            );
    }
}

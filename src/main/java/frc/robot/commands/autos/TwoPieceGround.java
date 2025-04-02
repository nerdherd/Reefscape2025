package frc.robot.commands.autos;

import java.io.IOException;
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
                // Move to L4
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(0)), // swerve.driveToTagCommand(VisionConstants.kLimelightBackLeftName).withTimeout(2),
                    superSystem.moveToAuto(PositionEquivalents.L1)
                ),
                superSystem.moveToAuto(PositionEquivalents.L4),

                // Outtake
                Commands.waitSeconds(0.25),
                superSystem.outtake(),
                Commands.waitSeconds(1),
                superSystem.stopRoller(),
                superSystem.moveToAuto(PositionEquivalents.L1),

                // Move to A3O
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(1)),
                    superSystem.moveToAuto(PositionEquivalents.SemiStow)
                ),
                superSystem.moveTo(PositionEquivalents.GroundIntake),

                // Move to and intake ground coral
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(2)),
                    superSystem.intakeUntilSensed(2)
                ),

                // Move to Reef
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(3)),
                    Commands.sequence(
                        superSystem.moveToAuto(PositionEquivalents.SemiStow),
                        Commands.waitSeconds(0.5),
                        superSystem.moveToAuto(PositionEquivalents.L1)
                    )
                ),
                
                // Move to L4
                // Commands.parallel(
                //     // swerve.driveToTagCommand(VisionConstants.kLimelightBackRightName).withTimeout(2),
                //     superSystem.moveToAuto(PositionEquivalents.L1)
                // ),
                superSystem.moveToAuto(PositionEquivalents.L4),

                // Outtake
                Commands.waitSeconds(0.25),
                // superSystem.outtake(),
                Commands.waitSeconds(1)
                // superSystem.stopRoller()
            )
        );
    }
}

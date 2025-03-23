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
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.SuperSystemConstants.PositionEquivalents;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class TwoPiece extends SequentialCommandGroup {
    public TwoPiece(SwerveDrivetrain swerve, String autoname, SuperSystem superSystem) throws IOException, ParseException {
        
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoname);
        Pose2d startingPose = pathGroup.get(0).getStartingDifferentialPose();

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            // Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            // Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            
            Commands.sequence(
                Commands.sequence(
                    superSystem.holdPiece(),
                    superSystem.moveTo(PositionEquivalents.Stow),
                    AutoBuilder.followPath(pathGroup.get(0)),
                    Commands.sequence(
                        // superSystem.moveToAuto(PositionEquivalents.L4)
                        superSystem.moveToAuto(PositionEquivalents.L1),
                        Commands.runOnce(() ->swerve.setAutoPathRun(1, -1)).withTimeout(2)
                    )
                ),
                Commands.sequence(
                    superSystem.outtake(),
                    Commands.waitSeconds(2.0),
                    superSystem.stopRoller()
                ),
                Commands.sequence(
                    // superSystem.moveTo(PositionEquivalents.L5),
                    // superSystem.moveTo(PositionEquivalents.L1),
                    superSystem.moveTo(PositionEquivalents.SemiStow),
                    Commands.parallel(
                        AutoBuilder.followPath(pathGroup.get(1)),
                        superSystem.moveToAuto(PositionEquivalents.GroundIntake)
                        // superSystem.moveToAuto(PositionEquivalents.Stow),
                        // Commands.waitSeconds(2.0)
                    )
                ),
                Commands.race(
                    // superSystem.intake(),
                    superSystem.intakeUntilSensed(2),
                    swerve.driveToCoralCommand("limelight-coral", 8)

                    // Commands.waitSeconds(2)
                    // superSystem.holdPiece()
                ),
                swerve.driveToPose(pathGroup.get(3).getStartingDifferentialPose(), PathPlannerConstants.kPPMaxVelocity, PathPlannerConstants.kPPMaxAngularAcceleration),
                Commands.sequence(
                    Commands.parallel(
                        Commands.sequence(
                            Commands.waitSeconds(0.3)
                        ),
                        AutoBuilder.followPath(pathGroup.get(3))
                    ),
                    Commands.sequence(
                        superSystem.moveToAuto(PositionEquivalents.L1),
                        Commands.runOnce(() ->swerve.setAutoPathRun(1, 1)).withTimeout(2)

                        // superSystem.moveToAuto(PositionEquivalents.L1)
                    )
                ),
                Commands.sequence(
                    superSystem.outtake(),
                    Commands.waitSeconds(2),
                    superSystem.stopRoller()
                ),

                
                Commands.sequence(
                    // superSystem.moveTo(PositionEquivalents.L5),
                    superSystem.moveTo(PositionEquivalents.L1),
                    superSystem.moveTo(PositionEquivalents.Stow)
                )
                )
            );
    }
}

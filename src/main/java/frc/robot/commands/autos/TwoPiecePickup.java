package frc.robot.commands.autos;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SuperSystemConstants.PositionEquivalents;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class TwoPiecePickup extends SequentialCommandGroup {
    public TwoPiecePickup(String autoname, SuperSystem superSystem, SwerveDrivetrain swerve) throws IOException, ParseException {
        
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoname);
        // Pose2d startingPose = pathGroup.get(0).getStartingDifferentialPose();

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.waitSeconds(0.1),
            
            Commands.sequence(
                // Move to Reef
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(0)),
                    Commands.sequence(
                        superSystem.moveToAuto(PositionEquivalents.SemiStow)
                    )
                ),
                superSystem.moveToAuto(PositionEquivalents.L4),
                Commands.waitSeconds(1),

                // Outtake
                superSystem.outtake(),
                Commands.waitSeconds(1),
                superSystem.stopRoller(),
                superSystem.moveToAuto(PositionEquivalents.L2),

                // Move to A3O
                AutoBuilder.followPath(pathGroup.get(1)),
                Commands.sequence(
                    superSystem.moveToAuto(PositionEquivalents.SemiStow),
                    superSystem.moveToAuto(PositionEquivalents.GroundIntake)
                ),
                Commands.waitSeconds(1),

                // Move to and intake ground coral
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(2)),
                    superSystem.intakeUntilSensed(3)
                ),

                // Move to Reef
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(3)),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        superSystem.moveToAuto(PositionEquivalents.SemiStow)
                    )
                ),
                superSystem.moveToAuto(PositionEquivalents.L4),
                Commands.waitSeconds(1),

                // Outtake
                superSystem.outtake(),
                Commands.waitSeconds(1),
                superSystem.stopRoller(),
                superSystem.moveToAuto(PositionEquivalents.L2),

                // Move to A2O
                AutoBuilder.followPath(pathGroup.get(4)),
                Commands.sequence(
                    superSystem.moveToAuto(PositionEquivalents.SemiStow),
                    superSystem.moveToAuto(PositionEquivalents.GroundIntake)
                ),
                Commands.waitSeconds(1),

                // Move to and intake ground coral
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(5)),
                    superSystem.intakeUntilSensed(3)
                ),

                // Prepare for teleop
                Commands.parallel(
                    superSystem.moveToAuto(PositionEquivalents.SemiStow)
                )
            )
        );
    }
}

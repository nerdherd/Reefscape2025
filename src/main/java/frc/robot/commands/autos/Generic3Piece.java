package frc.robot.commands.autos;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import frc.robot.Constants.SuperSystemConstants.NamedPositions;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;

public class Generic3Piece extends SequentialCommandGroup {
    private List<PathPlannerPath> pathGroup;
    private Pose2d startingPose;
    private SuperSystem superSystem;

    public Generic3Piece(SwerveDrivetrain swerve, SuperSystem superSystem, String autoPath, int pos1, int pos2, int pos3) 
    throws IOException, ParseException {
    
        this.superSystem = superSystem;

        this.pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);
        this.startingPose = pathGroup.get(0).getStartingDifferentialPose();
        
        addCommands(
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            runAuto(pos1, pos2, pos3)
        );
    }

    public Command runAuto(int pos1, int pos2, int pos3) {
        return Commands.sequence(
            // Place preload
            AutoBuilder.followPath(pathGroup.get(0)),
            Commands.runOnce(() -> {
                switch (pos1) {
                case 1: superSystem.moveTo(NamedPositions.L1);
                case 2: superSystem.moveTo(NamedPositions.L2);
                case 3: superSystem.moveTo(NamedPositions.L3);
                case 4: superSystem.moveTo(NamedPositions.L4);
                }
            }),
            superSystem.outtake(),
            Commands.waitSeconds(1.5),
            superSystem.stopRoller(),

            // Drive to Coral Station and intake coral 2
            Commands.parallel(
                // elevator.stow(),
                AutoBuilder.followPath(pathGroup.get(1))
            ),
            // elevator.moveToStation(),
            // intakeRoller.intake(),
            Commands.waitSeconds(2.5),
            superSystem.stopRoller(),

            // Drive to Reef and place coral 2
            Commands.parallel(
                // elevator.stow(),
                AutoBuilder.followPath(pathGroup.get(2))
            ),
            superSystem.moveTo(NamedPositions.L2),
            Commands.runOnce(() -> {
                switch (pos2) {
                case 1: superSystem.moveTo(NamedPositions.L1); break;
                case 2: superSystem.moveTo(NamedPositions.L2); break;
                case 3: superSystem.moveTo(NamedPositions.L3); break;
                case 4: superSystem.moveTo(NamedPositions.L4); break;
                }
            }),
            superSystem.outtake(),
            Commands.waitSeconds(1.5),
            superSystem.stopRoller(),
            
            // Drive to Coral Station and intake coral 3
            Commands.parallel(
                // elevator.stow(),
                AutoBuilder.followPath(pathGroup.get(3))
            ),
            // elevator.moveToStation(),
            // intakeRoller.intake(),
            Commands.waitSeconds(2.5),
            superSystem.stopRoller(),

            // Drive to Reef and place coral 3
            Commands.parallel(
                // elevator.stow(),
                AutoBuilder.followPath(pathGroup.get(4))
            ),
            superSystem.moveTo(NamedPositions.L2),
            Commands.runOnce(() -> {
                switch (pos3) {
                case 1: superSystem.moveTo(NamedPositions.L1); break;
                case 2: superSystem.moveTo(NamedPositions.L2); break;
                case 3: superSystem.moveTo(NamedPositions.L3); break;
                case 4: superSystem.moveTo(NamedPositions.L4); break;
                }
            }),
            superSystem.outtake(),
            Commands.waitSeconds(1.5),
            stopAuto()
        );
    }

    public Command stopAuto() {
        return Commands.sequence(
            superSystem.stopRoller()//,
            // elevator.stow()
        );
    }
}

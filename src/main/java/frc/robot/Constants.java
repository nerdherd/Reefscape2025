// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.preferences.PrefDouble;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

 // COMMENT ROBOT IDS INSTEAD OF DELETING

public final class Constants {

  public static class ControllerConstants {

    public static final double kDeadband = 0.05;
    public static final double kRotationDeadband = 0.1;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

  }
  
  public static final class ModuleConstants {

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kTurningMotorGearRatio = 1 / 21.428; // 150 : 7 : 1 MK4i
    // public static final double kDriveDistanceLoss = 0.95; // from measuring IRL
    public static final double kDriveDistanceLoss = 1; // from measuring IRL
    public static final double kMetersPerRevolution = kWheelDiameterMeters * Math.PI * kDriveDistanceLoss;
    public static final double kDriveTicksToMeters = (1 / 2048.0) * kMetersPerRevolution; 
    public static final double kAbsoluteTurningTicksToRad = (1.0 / 4096.0) * 2 * Math.PI;
    public static final double kIntegratedTurningTicksToRad = (1.0 / 2048.0) * 2 * Math.PI;
    public static final double kDriveTicksPer100MsToMetersPerSec = kDriveTicksToMeters * 10;
    public static final double kAbsoluteTurningTicksPer100MsToRadPerSec = kAbsoluteTurningTicksToRad * 10;
    public static final double kIntegratedTurningTicksPer100MsToRadPerSec = kIntegratedTurningTicksToRad * 10;

    public static final double kDriveMotorDeadband = 0.02;
    public static final double kTurnMotorDeadband = 0.001;

    public static final PrefDouble kPTurning = new PrefDouble("kPTurning",0.26); // 0.55
    public static final PrefDouble kITurning = new PrefDouble("kITurning",0.01);
    public static final PrefDouble kDTurning = new PrefDouble("kDTurning",0.0015);//0.02 
    public static final PrefDouble kFTurning = new PrefDouble("kFTurning",0); //0.015

    public static final PrefDouble kPDrive = new PrefDouble("kPDrive",0.13); // 0.6
    public static final PrefDouble kIDrive = new PrefDouble("kIDrive",0);
    public static final PrefDouble kDDrive = new PrefDouble("kDDrive",0); 
    public static final PrefDouble kVDrive = new PrefDouble("kVDrive",0.0469); 

    public static final String kCANivoreName = "rio";

  } 

  public static final class SwerveDriveConstants {

    public static final double kVisionSTDx = 0.7; //0.9
    public static final double kVisionSTDy = 0.7; //0.9s
    public static final double kVisionSTDtheta = 1000; //Old: 69696969
    public static final Matrix<N3, N1> kBaseVisionPoseSTD = MatBuilder.fill(
                                                              Nat.N3(), Nat.N1(), 
                                                              kVisionSTDx,
                                                              kVisionSTDy,
                                                              kVisionSTDtheta);
    // VecBuilder.fill(kVisionSTDx, kVisionSTDy, kVisionSTDtheta);
    public static final PrefDouble kPThetaTeleop = new PrefDouble("kP Theta Teleop", 0);
    public static final PrefDouble kIThetaTeleop = new PrefDouble("kI Theta Teleop", 0);
    public static final PrefDouble kDThetaTeleop = new PrefDouble("kD Theta Teleop", 0);

    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(21);
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(21);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    
    public static final int kFRDriveID = 11;
    public static final int kFLDriveID = 21;
    public static final int kBLDriveID = 31;
    public static final int kBRDriveID = 41;

    public static final int kFRTurningID = 12;
    public static final int kFLTurningID = 22;
    public static final int kBLTurningID = 32;
    public static final int kBRTurningID = 42;

    public static final boolean kFRTurningReversed = true;
    public static final boolean kFLTurningReversed = true; 
    public static final boolean kBLTurningReversed = true; 
    public static final boolean kBRTurningReversed = true; 

    public static final boolean kFRDriveReversed = false;
    public static final boolean kFLDriveReversed = false;     
    public static final boolean kBLDriveReversed = false;      
    public static final boolean kBRDriveReversed = false;

    public static final class CANCoderConstants {
      public static final int kFRCANCoderID = 14;
      public static final int kFLCANCoderID = 24;
      public static final int kBLCANCoderID = 34;
      public static final int kBRCANCoderID = 44;

      public static final boolean kFRCANCoderReversed = false;    
      public static final boolean kFLCANCoderReversed = false;      
      public static final boolean kBLCANCoderReversed = false;       
      public static final boolean kBRCANCoderReversed = false; 
    }

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;    
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleMaxAcceleration = 5;
    // THIS CONSTANT HAS TO BE NEGATIVE OTHERWISE THE ROBOT WILL CRASH
    // TODO: Change deceleration with driver feedback, only in small increments (<= -2 is dangerous)
    public static final double kTeleMaxDeceleration = -5; // Russell says he likes 2.5 from sims, but keep at 3 until tested on real robot 

    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
      kPhysicalMaxAngularSpeedRadiansPerSecond * 0.75;
    public static final double kTurnToAngleMaxAngularSpeedRadiansPerSecond 
      = kPhysicalMaxAngularSpeedRadiansPerSecond;
    public static final double kTurnToBigAngleMaxAngularSpeedRadiansPerSecond = 1.5 * Math.PI;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double kMinimumMotorOutput = 0.05; // Minimum percent output on the falcons
    
    public static final double kDriveAlpha = 0.11765;
    public static final double kDriveOneMinusAlpha = 0.88235;

    public static final SwerveModuleState[] towModuleStates = 
    new SwerveModuleState[] {
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(135))
    };

    public static final double kGravityMPS = 9.80665; 

    public static final double kTurnToAnglePositionToleranceAngle = 5;
    public static final double kTurnToAngleVelocityToleranceAnglesPerSec = 2;

    public static enum FieldPositions {
      Reef1(5.008,5.279,-120.0),
      Reef2(5.345, 5.12, -120.0),
      Reef3(5.84, 4.084, 180.0),
      Reef4(5.84, 3.916, 180.0),
      Reef5(5.345, 2.88, 120.0),
      Reef6(5.008, 2.721, 120.0),
      //--------------------------------------
      // center is (4.49, 4.0)
      Reef7(3.972, 2.721, 60.0),
      Reef8(3.635, 2.88, 60.0),
      Reef9(3.14, 3.916, 0.0),
      Reef10(3.14, 4.084, 0.0),
      Reef11(3.635, 5.12, -60.0),
      Reef12(3.972, 5.279, -60.0),
      //--------------------------------------
      Source1(1.582, 7.275, 126.0),
      Source2(0.767, 6.692, 126.0),
      Source3(0.767, 1.35, -126.0),
      Source4(1.582, 0.78, -126.0),
      ;
      
      public Pose2d pos;
      FieldPositions(double _x, double _y, double _heading) {
        pos = new Pose2d(new Translation2d(_x, _y), new Rotation2d(Units.degreesToRadians(_heading)));
      }
    }

  }

  public static final class PathPlannerConstants {

    public static final double kPPMaxVelocity = 3.0;
    public static final double kPPMaxAcceleration = 3.0;
    public static final double kPPMaxAngularVelocity = Math.PI * 2;
    public static final double kPPMaxAngularAcceleration = Math.PI * 2;

    public static final double kPP_P = new PrefDouble("PP_kP", 5.0).get();
    public static final double kPP_I = new PrefDouble("PP_kI", 0.0).get();
    public static final double kPP_D = new PrefDouble("PP_kD", 0.0).get();

    public static final PIDConstants kPPTranslationPIDConstants = new PIDConstants(kPP_P, kPP_I, kPP_D);

    public static final double kPP_ThetaP = new PrefDouble("PP_kThetaP", 3.0).get();
    public static final double kPP_ThetaI = new PrefDouble("PP_kThetaI", 0).get();
    public static final double kPP_ThetaD = new PrefDouble("PP_kThetaD", 0.1).get();

    public static final PIDConstants kPPRotationPIDConstants = new PIDConstants(kPP_ThetaP, kPP_ThetaI, kPP_ThetaD);

    public static final boolean kUseAllianceColor = true;

  }

  public static final class VisionConstants {

    public static final double kFrontCameraHeightMeters = 0; // TODO change for new bot limelights
    public static final double kNoteHeightMeters = 0;
    public static final double kCameraPitchRadians = 0;
    public static final double kSunflowerP = 0.2;
    public static final double kSunflowerI = 0;
    public static final double kSunflowerD = 0;
    public static final String kLimelightFrontName = "limelight-front";
    public static final String kLimelightFrontIP = "10.6.87.25:5802";
    public static final int kAprilTagPipeline = 4;
    public static final double fieldXOffset = 8.27; // Certified (Half field dimensions)
    public static final double fieldYOffset = 4.01; // Certified (Half as well)
    public static final double kMinimumTA = 0.7;
    public static final Transform3d fieldPoseOffset = new Transform3d(
      new Translation3d(-VisionConstants.fieldXOffset, -VisionConstants.fieldYOffset, 0), 
      new Rotation3d()
    );

    public static final String kPhotonVisionFrontName = "laserbean";
    public static final Transform3d kCameraToRobot = new Transform3d(
      new Translation3d(),
      new Rotation3d()
    ); // distance from camera to center of robot

  }

  public static class LEDConstants {

    public static final int CANdleID = 0; // TODO change later
    public static final int CANdleLength = 8; // TODO change later

    public static class Colors {
      public static final Color BLACK         = new Color(0.0, 0.0, 0.0); // shows up as nothing
      public static final Color WHITE         = new Color(1.0,1.0,1.0); 
      public static final Color RED           = new Color(1.0, 0.0, 0.0); 
      public static final Color GREEN         = new Color(0.0, 1.0, 0.0); 
      public static final Color BLUE          = new Color(0.0, 0.0, 1.0); 
      public static final Color NERDHERD_BLUE = new Color(0.132, 0.415, 1.0); // #071635 as base, brightened fully
    }
    
    public enum LEDStrips {
      ALL(0, CANdleLength),
      CANDLE(0,8),
      ;

      public int index, count;
      LEDStrips(int _index, int _count) {
        this.index = _index;
        this.count = _count;
      }
    }

  }

  public static final class ElevatorConstants {

    // ************************************** ELEVATOR CONSTANTS *************************************** //
    public static final int kElevatorMotorID = 18;            // TODO change later 
    public static final double kElevatorStowPosition    = -1; // TODO change later
    public static final double kElevatorStationPosition = 0 ; // TODO change later
    public static final double kElevatorL1Position =   0;     // TODO change later
    public static final double kElevatorL2Position =  -9;     // TODO change later
    public static final double kElevatorL3Position = -18;     // TODO change later
    public static final double kElevatorL4Position = -28;     // TODO change later
    public static final double kElevatorSpeed = 1.0;          // TODO change later
    public static final PrefDouble kPElevatorMotor = new PrefDouble("P Elevator Motor", 0.1);
    public static final PrefDouble kIElevatorMotor = new PrefDouble("I Elevator Motor", 0);
    public static final PrefDouble kDElevatorMotor = new PrefDouble("D Elevator Motor", 0);
    public static final PrefDouble kVElevatorMotor = new PrefDouble("V Elevator Motor", 0);

    // ********************************* ELEVATOR PIVOT CONSTANTS ********************************** //
    public static final int kPivotMotorID = 59; // TODO IDs cant go over 62
    public static final int kPivotPigeonID = 2; // TODO change later
    
    public static final PrefDouble kPElevatorPivot = new PrefDouble("P Elevator Pivot Motor", 0.1);
    public static final PrefDouble kIElevatorPivot = new PrefDouble("I Elevator Pivot Motor", 0);
    public static final PrefDouble kDElevatorPivot = new PrefDouble("D Elevator Pivot Motor", 0);
    public static final PrefDouble kVElevatorPivot = new PrefDouble("V Elevator Pivot Motor", 0.015);
    public static final PrefDouble kSElevatorPivot = new PrefDouble("S Elevator Pivot Motor", 0.25);
    public static final PrefDouble kAElevatorPivot = new PrefDouble("A Elevator Pivot Motor", 0.01);
    public static final PrefDouble kGElevatorPivot = new PrefDouble("G Elevator Pivot Motor", 0.01);

    public static final PrefDouble kElevatorPivotStowPosition = new PrefDouble("Elevator Pivot Stow", -90.5); //TODO change later
    public static final PrefDouble kElevatorPivotStartPosition = new PrefDouble("Elevator Pivot Start", 0); //TODO change later
    public static final PrefDouble kElevatorPivotPickUpPosition = new PrefDouble("Elevator Pivot PickUp", 90); //TODO change later

    public static final double kElevatorPivotMin = -90; // TODO change later
    public static final double kElevatorPivotMax = 90; // TODO change later

    public static final double kElevatorPivotGearRatio = 0; // TODO change later
    public static final PrefDouble kElevatorPivotDeadBand = new PrefDouble("Elevator Pivot DeadBand", 0);
    public static final PrefDouble kElevatorPivotOffSet = new PrefDouble("Elevator Pivot Offset", 0);

    public static final PrefDouble kEPivotCruiseVelocity = new PrefDouble("Elevator Pivot Cruise Velocity",16_333);
    public static final PrefDouble kElevatorPivotCruiseAcceleration = new PrefDouble("Elevator Pivot Acceleration",100_000);

  }

  public static final class IntakeConstants {

    // ************************************** ROLLER CONSTANTS *************************************** //
    public static final int kRollerMotorID = 61;
    public static final PrefDouble kPRollerMotor = new PrefDouble("kP Roller Motor", 0.05);
    public static final PrefDouble kIRollerMotor = new PrefDouble("kI Roller Motor", 0);
    public static final PrefDouble kDRollerMotor = new PrefDouble("kD Roller Motor", 0);
    public static final PrefDouble kVRollerMotor = new PrefDouble("kV Roller Motor", 0);

    public static final double kRollerMaxVelocityRPS =  100;
    public static final double kRollerMinVelocityRPS = -100;

    public static final double kRollerNeutralDeadband = 0.01; // In revolutions!

    public static final PrefDouble kIntakePower  = new PrefDouble("Roller Intake Power", 30);
    public static final PrefDouble kOuttakePower = new PrefDouble("Roller Outtake Power",     -30);

    // ************************************** WRIST CONSTANTS *************************************** //
    public static final int kWristMotorID = 54;
  
    public static final PrefDouble kPWristMotor = new PrefDouble("kP Wrist", 0.1);
    public static final PrefDouble kIWristMotor = new PrefDouble("kI Wrist", 0);
    public static final PrefDouble kDWristMotor = new PrefDouble("kD Wrist", 0);
    public static final PrefDouble kVWristMotor = new PrefDouble("kV Wrist", 0.12);
    public static final PrefDouble kSWristMotor = new PrefDouble("kS Wrist", 0); //Static Friction
    public static final PrefDouble kGWristMotor = new PrefDouble("kG Wrist", 0.015); //Gravity
    public static final PrefDouble kWristAcceleration = new PrefDouble("Wrist Acceleration", 100); //Rotations per second squared
    public static final PrefDouble kWristJerk = new PrefDouble("Wrist Jerk", 700); //Rotations per second squared

    public static final PrefDouble kWristStowPosition = new PrefDouble("Wrist Stow Position", -0.1);
    public static final PrefDouble kWristStationPosition = new PrefDouble("Wrist Station Position", 0);
    public static final PrefDouble kWristL23Position = new PrefDouble("Wrist L23 Position", 0);
    public static final PrefDouble kWristL14Position = new PrefDouble("Wrist L4 Position", -0.9);

    public static final double kWristSpeed = 0.5;

 }

}

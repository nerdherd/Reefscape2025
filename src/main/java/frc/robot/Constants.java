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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.SuperSystem.ExecutionOrder;

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
  public enum ROBOT_ID{
    ISME,
    V1,
    V2,
  }

  public final static ROBOT_ID ROBOT_NAME = ROBOT_ID.ISME;

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

    public static final double kPTurning = 0.26; // 0.55
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0.0015;//0.02 
    public static final double kFTurning = 0.015;//0.015

    public static final double kPDrive = 0.13; // 0.6
    public static final double kIDrive = 0;
    public static final double kDDrive = 0; 
    public static final double kVDrive = 0.0469; 

    public static final String kCANivoreName = "CANivore";//"CANivore";

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
    public static final double kPThetaTeleop = 0;
    public static final double kIThetaTeleop = 0;
    public static final double kDThetaTeleop = 0;

    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(24.125);
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(24.125);

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

    public static final class MapPoses {
      public static final double CCReefOffset = 0.0; // meters
      
      public static Pose2d offsetPosWithRotation(Pose2d pos) { return offsetPosWithRotation(pos, CCReefOffset); }
      public static Pose2d offsetPosWithRotation(Pose2d pos, double offset) {
        return new Pose2d(pos.getX() + offset * (Math.cos(pos.getRotation().getRadians() - Math.toRadians(90.0))), 
                          pos.getY() + offset * (Math.sin(pos.getRotation().getRadians() - Math.toRadians(90.0))), 
                          pos.getRotation());
      }
      // Blue Side
      public static final Pose2d tag12Left = new Pose2d(1.768, 0.783, new Rotation2d(Math.toRadians(-125))); // Bot Station
      public static final Pose2d tag12Right = new Pose2d(0.797, 1.489, new Rotation2d(Math.toRadians(-125))); // Bot Station
      public static final Pose2d tag12Mid = new Pose2d(1.678, 0.83, new Rotation2d(Math.toRadians(-125))); // Bot Station
 
      public static final Pose2d tag13Left = new Pose2d(0.797, 6.563, new Rotation2d(Math.toRadians(125))); // Top Station
      public static final Pose2d tag13Right = new Pose2d(1.768,7.269, new Rotation2d(Math.toRadians(125))); // Top Station
      public static final Pose2d tag13Mid = new Pose2d(1.367, 6.896, new Rotation2d(Math.toRadians(125))); // Top Station
 
      public static final Pose2d tag16Mid = new Pose2d(6.0, 0.75, new Rotation2d(Math.toRadians(-90))); // Processor
 
      // they are for blue; but tag is on the red field
      public static final Pose2d tag4LeftBlueOnRed = new Pose2d(9.496, 7.233, new Rotation2d(Math.toRadians(0))); // cage top
      public static final Pose2d tag4MidBlueOnRed = new Pose2d(9.511, 6.151, new Rotation2d(Math.toRadians(0))); // cage mid
      public static final Pose2d tag4RightBlueOnRed = new Pose2d(9.51, 5.054, new Rotation2d(Math.toRadians(0))); // cage btm
     
      // Reef
      public static final Pose2d tag17Left  = offsetPosWithRotation(new Pose2d(3.513, 2.959, new Rotation2d(Math.toRadians(-120)))); 
      public static final Pose2d tag17Right = offsetPosWithRotation(new Pose2d(3.88, 2.745, new Rotation2d(Math.toRadians(-120)))); 
      public static final Pose2d tag17Mid   = offsetPosWithRotation(new Pose2d(3.70, 2.6, new Rotation2d(Math.toRadians(-120)))); 
  
      public static final Pose2d tag18Left  = offsetPosWithRotation(new Pose2d(3.067, 4.220, new Rotation2d(Math.toRadians(180)))); 
      public static final Pose2d tag18Right = offsetPosWithRotation(new Pose2d(3.067, 3.832, new Rotation2d(Math.toRadians(180)))); 
      public static final Pose2d tag18Mid   = offsetPosWithRotation(new Pose2d(3.067, 4.026, new Rotation2d(Math.toRadians(180)))); 
  
      public static final Pose2d tag19Left  = offsetPosWithRotation(new Pose2d(3.946, 5.354, new Rotation2d(Math.toRadians(120)))); 
      public static final Pose2d tag19Right = offsetPosWithRotation(new Pose2d(3.610, 5.160, new Rotation2d(Math.toRadians(120)))); 
      public static final Pose2d tag19Mid   = offsetPosWithRotation(new Pose2d(3.778, 5.257, new Rotation2d(Math.toRadians(120)))); 
  
      public static final Pose2d tag20Left  = offsetPosWithRotation(new Pose2d(5.368, 5.160, new Rotation2d(Math.toRadians(60))));
      public static final Pose2d tag20Right = offsetPosWithRotation(new Pose2d(5.032, 5.354, new Rotation2d(Math.toRadians(60))));
      public static final Pose2d tag20Mid   = offsetPosWithRotation(new Pose2d(5.200, 5.257, new Rotation2d(Math.toRadians(60))));
  
      public static final Pose2d tag21Left  = offsetPosWithRotation(new Pose2d(5.912, 3.832, new Rotation2d(Math.toRadians(0))));
      public static final Pose2d tag21Right = offsetPosWithRotation(new Pose2d(5.912, 4.220, new Rotation2d(Math.toRadians(0))));
      public static final Pose2d tag21Mid   = offsetPosWithRotation(new Pose2d(5.912, 4.026, new Rotation2d(Math.toRadians(0))));
  
      public static final Pose2d tag22Left  = offsetPosWithRotation(new Pose2d(5.1, 2.725, new Rotation2d(Math.toRadians(-60))));
      public static final Pose2d tag22Right = offsetPosWithRotation(new Pose2d(5.391, 2.921, new Rotation2d(Math.toRadians(-60))));
      public static final Pose2d tag22Mid   = offsetPosWithRotation(new Pose2d(5.26, 2.77, new Rotation2d(Math.toRadians(-60))));
     
      // Red Side
      public static final Pose2d tag1Left = new Pose2d(16.751, 1.489, new Rotation2d(Math.toRadians(-55))); // Bot Station
      public static final Pose2d tag1Right = new Pose2d(15.78, 0.783, new Rotation2d(Math.toRadians(-55)));
      public static final Pose2d tag1Mid = new Pose2d(16.255, 1.238, new Rotation2d(Math.toRadians(-55)));
 
      public static final Pose2d tag2Left = new Pose2d(15.78, 7.269, new Rotation2d(Math.toRadians(55))); // Top Station
      public static final Pose2d tag2Right = new Pose2d(16.751, 6.563, new Rotation2d(Math.toRadians(55)));
      public static final Pose2d tag2Mid = new Pose2d(16.0, 6.860, new Rotation2d(Math.toRadians(55)));
     
      public static final Pose2d tag3Mid = new Pose2d(11.560, 7.2, new Rotation2d(Math.toRadians(90))); // Processor
 
      // they are for red; but tag is on the blue field
      public static final Pose2d tag15RightRedOnBlue = new Pose2d(8.024, 2.996, new Rotation2d(Math.toRadians(180))); // cage top
      public static final Pose2d tag15MidRedOnBlue = new Pose2d(8.024, 1.944, new Rotation2d(Math.toRadians(180))); // cage mid
      public static final Pose2d tag15LeftRedOnBlue = new Pose2d(8.03, 0.802, new Rotation2d(Math.toRadians(180))); // cage btm
 
      // Reef
      public static final Pose2d tag6Left  = offsetPosWithRotation(new Pose2d(13.602, 2.697, new Rotation2d(Math.toRadians(-60))));   
      public static final Pose2d tag6Right = offsetPosWithRotation(new Pose2d(13.938, 2.892, new Rotation2d(Math.toRadians(-60))));  
      public static final Pose2d tag6Mid   = offsetPosWithRotation(new Pose2d(13.770, 2.795, new Rotation2d(Math.toRadians(-60)))); 
  
      public static final Pose2d tag7Left  = offsetPosWithRotation(new Pose2d(14.481, 3.832, new Rotation2d(Math.toRadians(0)))); 
      public static final Pose2d tag7Right = offsetPosWithRotation(new Pose2d(14.481, 4.220, new Rotation2d(Math.toRadians(0)))); 
      public static final Pose2d tag7Mid   = offsetPosWithRotation(new Pose2d(14.481, 4.026, new Rotation2d(Math.toRadians(0)))); 
  
      public static final Pose2d tag8Left  = offsetPosWithRotation(new Pose2d(13.938, 5.160, new Rotation2d(Math.toRadians(60)))); 
      public static final Pose2d tag8Right = offsetPosWithRotation(new Pose2d(13.602, 5.354, new Rotation2d(Math.toRadians(60)))); 
      public static final Pose2d tag8Mid   = offsetPosWithRotation(new Pose2d(13.770, 5.257, new Rotation2d(Math.toRadians(60)))); 
  
      public static final Pose2d tag9Left  = offsetPosWithRotation(new Pose2d(12.516, 5.354, new Rotation2d(Math.toRadians(120))));
      public static final Pose2d tag9Right = offsetPosWithRotation(new Pose2d(12.180, 5.160, new Rotation2d(Math.toRadians(120))));
      public static final Pose2d tag9Mid   = offsetPosWithRotation(new Pose2d(12.348, 5.257, new Rotation2d(Math.toRadians(120))));
  
      public static final Pose2d tag10Left  = offsetPosWithRotation(new Pose2d(11.636, 4.220, new Rotation2d(Math.toRadians(180))));
      public static final Pose2d tag10Right = offsetPosWithRotation(new Pose2d(11.636, 3.832, new Rotation2d(Math.toRadians(180))));
      public static final Pose2d tag10Mid   = offsetPosWithRotation(new Pose2d(11.636, 4.026, new Rotation2d(Math.toRadians(180))));
  
      public static final Pose2d tag11Left  = offsetPosWithRotation(new Pose2d(12.180, 2.892, new Rotation2d(Math.toRadians(-120))));
      public static final Pose2d tag11Right = offsetPosWithRotation(new Pose2d(12.516, 2.697, new Rotation2d(Math.toRadians(-120))));
      public static final Pose2d tag11Mid   = offsetPosWithRotation(new Pose2d(12.348, 2.795, new Rotation2d(Math.toRadians(-120))));
    }
  }

  public static final class PathPlannerConstants {

    public static final double kPPMaxVelocity = 2.0;
    public static final double kPPMaxAcceleration = 2.0;
    public static final double kPPMaxAngularVelocity = Math.PI * 2;
    public static final double kPPMaxAngularAcceleration = Math.PI * 2;

    public static final double kPP_P = 6.0; //5
    public static final double kPP_I = 0.0;
    public static final double kPP_D = 0.0;

    public static final PIDConstants kPPTranslationPIDConstants = new PIDConstants(kPP_P, kPP_I, kPP_D);

    public static final double kPP_ThetaP = 3.0; //3
    public static final double kPP_ThetaI = 0;
    public static final double kPP_ThetaD = 0.1;

    public static final PIDConstants kPPRotationPIDConstants = new PIDConstants(kPP_ThetaP, kPP_ThetaI, kPP_ThetaD);

    public static final boolean kUseAllianceColor = true;

  }

  public static final class VisionConstants {

    public static final String kLimelightFrontLeftName = "limelight-fl";
    public static final String kLimelightFrontLeftIP = "10.6.87.15:5802";
    public static final String kLimelightFrontRightName = "limelight-fr";
    public static final String kLimelightFrontRightIP = "10.6.87.17:5802";
    public static final String kLimelightBackLeftName = "limelight-bl";
    public static final String kLimelightBackLeftIP = "10.6.87.5:5802";
    public static final String kLimelightBackRightName = "limelight-br";
    public static final String kLimelightBackRightIP = "10.6.87.7:5802";

    public static final double kFrontCameraHeightMeters = 0; // TODO change for new bot limelights
    public static final double kNoteHeightMeters = 0;
    public static final double kCameraPitchRadians = 0;
    public static final double kSunflowerP = 0.2;
    public static final double kSunflowerI = 0;
    public static final double kSunflowerD = 0;
    
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
    public static final int kElevatorMotorID = 8;
    public static final int kElevatorMotorID2 = 9; 

    public static final double kPElevatorMotor = 4.0; // 3.0 as of 3/1/25
    public static final double kIElevatorMotor = 0;
    public static final double kDElevatorMotor = 0;
    public static final double kVElevatorMotor = 0;
    public static final double kGElevatorMotor = 0.2;
    public static final double kSElevatorMotor = 0.11;

    public static final double kElevatorCruiseVelocity = 100; //16.333 //20
    public static final double kElevatorCruiseAcceleration = kElevatorCruiseVelocity * 10;
    public static final double kElevatorJerk = kElevatorCruiseAcceleration * 10;
 }
 

 public static final class PivotConstants {
    public static final int kLeftPivotMotorID = 17;  // TODO: Switch back motor IDs. This is TEMPORARY 2/24
    public static final int kRightPivotMotorID = 18;
    public static final int kPivotPigeonID = 2; // TODO change later
    
    public static final double kPElevatorPivot = 50; // TODO: NEED TO CALCULATE AND INPUT A kP
    // 0.22V = kP * 0.01         max kP = 100 .01 error is pretty high
    public static final double kIElevatorPivot = 0;
    public static final double kDElevatorPivot = 0;
    public static final double kVElevatorPivot = 0; 
    public static final double kSElevatorPivot = 0; 
    public static final double kAElevatorPivot = 0.0; 
    public static final double kGElevatorPivot = 0;

    public static final double kFElevatorPivot = 0.22; 

    public static final double kElevatorPivotStowPosition = 0.01; 
    public static final double kElevatorPivotSemiStowPosition = 0.11; 
    public static final double kElevatorPivotStationPosition = 0.1;//0.15 // .0833 //TODO change later
    public static final double kElevatorPivotPositionVertical = 0.24;//0.1 // 0.25

    public static final double kElevatorPivotMin = 0; // This is Stow with Foam underneath // TODO change later   
    public static final double kElevatorPivotMax = 0.23; // Vertical    // TODO change later   

    public static final double kElevatorPivotGearRatio = 125.0 / 1.0; // 16:1 for Gearbox, 5:1 for Chain
    public static final double kElevatorPivotDeadBand = 0;
    public static final double kElevatorPivotOffSet = 0;

    public static final double kEPivotCruiseVelocity = 0.8;//0.25 // 0.4S
    public static final double kElevatorPivotCruiseAcceleration = kEPivotCruiseVelocity * 5; // 0.5
    public static final double kElevatorPivotJerk = kElevatorPivotCruiseAcceleration * 10; 
    
  }

  public static final class RollerConstants {

    public static final int kLeftMotorID = 61; 
    public static final int kRightMotorID = 62; 
    public static final double kPMotor = 0.05;
    public static final double kIMotor = 0;
    public static final double kDMotor = 0;
    public static final double kVMotor = 0;

    public static final double kMaxVelocityRPS =  100;
    public static final double kMinVelocityRPS = -100;

    public static final double kNeutralDeadband = 0.01; // In revolutions!

    public static final double kIntakePower  = -2.8;
    public static final double kOuttakePower = 1.5;
    public static final double kL1OuttakePower = 0.7;
  }
  public static final class WristConstants{
    public static final int kMotorID = 54;
  
    public static final double kPMotor =  40; 
    // kP * err_rotations = Max_Volt_Needed (A little higher than kG)
    //      err = 0.28 (10/360 degrees)
    public static final double kItMotor = 0;
    public static final double kDMotor = 0;
    public static final double kVMotor = 0;
    public static final double kSMotor = 0; //Static Friction
    public static final double kGMotor = 0.6; //Gravity
    public static final double kCruiseVelocity = 2;
    public static final double kAcceleration = kCruiseVelocity * 1.5; // Double velocity - Rotations per second squared
    public static final double kJerk = kAcceleration * 10; // 10 times accel - Rotations per second cubed

    public static final double kMaxPosition = 0.14; //156
    public static final double kMinPosition = 0;

    public static final double kSpeed = 0.5;
    public static final int kPigeonID = 2;
  }

  public static final class ClimbConstants { // TODO change

    public static final int kMotorID = 49;

    public static final double kPMotor = 0;
    public static final double kIMotor = 0;
    public static final double kDMotor = 0;
    public static final double kVMotor = 0;
    public static final double kSMotor = 0;
    public static final double kGMotor = 0;

    public static final double kCruiseVelocity = 0;
    public static final double kAcceleration = 0;
    public static final double kJerk = 0;

    public static final double kOpenPosition = 0;
    public static final double kClosedPosition = 0;

  }
  

  public static final class SuperSystemConstants {
    public enum NamedPositions { 
      Stow(                ExecutionOrder.WRTELV_PVT  , 0.01,  0,    -0.010, -0.01),
      SemiStow(            ExecutionOrder.WRTELV_PVT  , 0.11,  0.05, -0.164, -0.164      ),
      // GroundIntake(        ExecutionOrder.ALL_TOGETHER, 0.018, 0.55, -0.760, -0.760      ),
      GroundIntake(        ExecutionOrder.ALL_TOGETHER, 0.02, 0.55, -0.780, -0.780      ),
      Station(             ExecutionOrder.ALL_TOGETHER, 0.18,  1.12, -0.850, -0.4),
      Processor(           ExecutionOrder.WRTELV_PVT  , 0.047, 0.53, -0.790, -0.4),
      Net(                 ExecutionOrder.WRTELV_PVT  , 0.24,  0.05, -0.100, -0.100      ),
      Cage(                ExecutionOrder.WRTELV_PVT  , 0.11,  0.05, -0.100, -0.100      ),
      L1(                  ExecutionOrder.WRTELV_PVT  , 0.25,  0,    -0.180, -0.570      ),
      L2(                  ExecutionOrder.WRTELV_PVT  , 0.25,  0.0,  -0.210, -0.570      ),
      L3(                  ExecutionOrder.WRTPVT_ELV  , 0.25,  1.14, -0.240, -0.570      ),
      L4(                  ExecutionOrder.WRTPVT_ELV  , 0.255,  3.18, -0.270, -0.570      ),
      L4Auto(              ExecutionOrder.WRTPVT_ELV  , 0.255,  3.18, -0.270, -0.570      ),
      L4AutoPre(           ExecutionOrder.ALL_TOGETHER, 0.255,  0.0, -0.570, -0.570      ),
      L5(                  ExecutionOrder.WRTELV_PVT  , 0.255,  1.12, -0.570, -0.570      ),
      AlgaeL2(             ExecutionOrder.WRTPVT_ELV  , 0.247, 0,  -0.266, -0.570      ), 
      AlgaeL3(             ExecutionOrder.WRTPVT_ELV  , 0.24,  1,    -0.266, -0.570      ), 
      ClimbDown(           ExecutionOrder.WRT_ELV_PVT ,   -0.06, 1.38, 0, 0      ),
      ClimbUp(             ExecutionOrder.WRTELV_PVT  , 0.14,  0.05, -0.164, -0.164      ),
      intermediateGround(  ExecutionOrder.PVT_ELV_WRT , 0.1,   0.18, -0.760, -0.760      ),
      ;

      public double intermediateWristPosition, finalWristPosition, elevatorPosition, pivotPosition; // rotations not degrees
      public ExecutionOrder executionOrder;
      NamedPositions(ExecutionOrder eo, double pp, double ep, double fwp, double iwp) {
        intermediateWristPosition = iwp;
        finalWristPosition = fwp;
        elevatorPosition = ep;
        pivotPosition = pp;
        executionOrder = eo;
      }
      NamedPositions(ExecutionOrder eo, double pp, double ep, double fwp) {
        this(eo, pp, ep, fwp, fwp);
      }
    }
  }
}

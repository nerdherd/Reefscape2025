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

  public final static int ROBOT_NAME = 1;// 1= is me; 2 = demo

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

  }

  public static final class PathPlannerConstants {

    public static final double kPPMaxVelocity = 3.0;
    public static final double kPPMaxAcceleration = 3.0;
    public static final double kPPMaxAngularVelocity = Math.PI * 2;
    public static final double kPPMaxAngularAcceleration = Math.PI * 2;

    public static final double kPP_P = 5.0;
    public static final double kPP_I = 0.0;
    public static final double kPP_D = 0.0;

    public static final PIDConstants kPPTranslationPIDConstants = new PIDConstants(kPP_P, kPP_I, kPP_D);

    public static final double kPP_ThetaP = 3.0;
    public static final double kPP_ThetaI = 0;
    public static final double kPP_ThetaD = 0.1;

    public static final PIDConstants kPPRotationPIDConstants = new PIDConstants(kPP_ThetaP, kPP_ThetaI, kPP_ThetaD);

    public static final boolean kUseAllianceColor = true;

  }

  public static final class VisionConstants {

    public static final String kLimelightLowLeftName = "limelight-lowleft";
    public static final String kLimelightLowLeftIP = "10.6.87.12:5802";
    public static final String kLimelightLowRightName = "limelight-lowright";
    public static final String kLimelightLowRightIP = "10.6.87.24:5802";
    public static final String kLimelightHighLeftName = "limelight-highleft";
    public static final String kLimelightHighLeftIP = "10.6.87.55:5802";
    public static final String kLimelightHighRightName = "limelight-highright";
    public static final String kLimelightHighRightIP = "10.6.87.75:5802";

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
    public static final double kElevatorStowPosition    =  0.05; //0.5   // TODO change later
    public static final double kElevatorStationPosition = 0;//-2 // TODO change later
    public static final double kElevatorL1Position =   0;      // TODO change later
    public static final double kElevatorL2Position =  0.1;      // TODO change later
    public static final double kElevatorL2L3Position =  0.5;      // TODO change later
    public static final double kElevatorL3Position = 1.36;      // TODO change later
    public static final double kElevatorL3L4Position = 1.36;      // TODO change later
    public static final double kElevatorL4Position = 3.15;      // TODO change later
    public static final double kElevatorSpeed = 1.0;           // TODO change later
    public static final double kElevatorGroundIntake = 0.562;           // TODO change later


    public static final double kPElevatorMotor = 4.0; // 3.0 as of 3/1/25
    public static final double kIElevatorMotor = 0;
    public static final double kDElevatorMotor = 0;
    public static final double kVElevatorMotor = 0;
    public static final double kGElevatorMotor = 0.33;
    public static final double kSElevatorMotor = 0.11;

    public static final double kElevatorCruiseVelocity = 20; //16.333 //30
    public static final double kElevatorCruiseAcceleration = 40.000;
    public static final double kElevatorJerk = 400.0;

    // ********************************* ELEVATOR PIVOT CONSTANTS ********************************** //
    public static final int kLeftPivotMotorID = 17; 
    public static final int kPivotPigeonID = 2; // TODO change later
    
    public static final double kPElevatorPivot = 40; //1.2
    public static final double kIElevatorPivot = 0;
    public static final double kDElevatorPivot = 0;
    public static final double kVElevatorPivot = 0; //0.4
    public static final double kSElevatorPivot = 0; //0.34
    public static final double kAElevatorPivot = 0.0; //0.01
    public static final double kGElevatorPivot = 0; //0.08

    public static final double kElevatorPivotStowPosition = -1.6; // Was -26 //TODO change later -22
    public static final double kElevatorPivotStationPosition = 0.15;
    public static final double kElevatorPivotStartPosition = 0; //TODO change later
    public static final double kElevatorPivotPickUpPosition = -40; //TODO change later   -60

    public static final double kElevatorPivotMin = -56.3; // TODO change later    -61.3
    public static final double kElevatorPivotMax = -25.4; // TODO change later   -20.4

    public static final double kElevatorPivotGearRatio = 27.0; // TODO change later
    public static final double kElevatorPivotDeadBand = 0;
    public static final double kElevatorPivotOffSet = 0;

    public static final double kEPivotCruiseVelocity = 80.0; //16.333
    public static final double kElevatorPivotCruiseAcceleration = 500.000;
    public static final double kElevatorPivotJerk = 900.0;

    public static final double kElevatorPivotStowedFF = 0.5;
    public static final double kElevatorPivotExtendedFF = 0.54117686;
    public static final double kElevatorPivotDiffFF = kElevatorPivotExtendedFF-kElevatorPivotStowedFF;
    public static final double kElevatorPivotExtendedFFPosition = 0.231934;
    
 }

 public static final class V1ElevatorConstants {
    // ************************************** ELEVATOR CONSTANTS *************************************** //
    public static final int kElevatorMotorID = 0;            // TODO change later 
    public static final int kElevatorMotorRightID = 0;            // TODO change later 
    public static final double kElevatorStowPosition    = 0; // TODO change later
    public static final double kElevatorStationPosition = 0 ; // TODO change later
    public static final double kElevatorL1Position =   0;     // TODO change later
    public static final double kElevatorL2Position =  0;     // TODO change later
    public static final double kElevatorL2L3Position =  0;     // TODO change later
    public static final double kElevatorL3Position = 0;     // TODO change later
    public static final double kElevatorL3L4Position = 0;     // TODO change later
    public static final double kElevatorL4Position = 0;     // TODO change later
    public static final double kElevatorSpeed = 1.0;          // TODO change later
    public static final double kPElevatorMotor = 0.1;
    public static final double kIElevatorMotor = 0;
    public static final double kDElevatorMotor = 0;
    public static final double kVElevatorMotor = 0;

    // ********************************* ELEVATOR PIVOT CONSTANTS ********************************** //
    public static final int kLeftPivotMotorID = 17;  // TODO: Switch back motor IDs. This is TEMPORARY 2/24
    public static final int kRightPivotMotorID = 18;
    public static final int kPivotPigeonID = 2; // TODO change later
    
    public static final double kPElevatorPivot = 60.0; // TODO: NEED TO CALCULATE AND INPUT A kP
    // 1V = kP * 0.01         max kP = 100 .01 error is pretty high
    public static final double kIElevatorPivot = 0;
    public static final double kDElevatorPivot = 0;
    public static final double kVElevatorPivot = 0; 
    public static final double kSElevatorPivot = 0; 
    public static final double kAElevatorPivot = 0.0; 
    public static final double kGElevatorPivot = 0; 

    public static final double kElevatorPivotStowPosition = 0.01; 
    public static final double kElevatorPivotSemiStowPosition = 0.11; 
    public static final double kElevatorPivotStationPosition = 0.1;//0.15 // .0833 //TODO change later
    public static final double kElevatorPivotPositionVertical = 0.24;//0.1 // 0.25

    public static final double kElevatorPivotMin = 0; // This is Stow with Foam underneath // TODO change later   
    public static final double kElevatorPivotMax = 0.23; // Vertical    // TODO change later   

    public static final double kElevatorPivotGearRatio = 80.0 / 1.0; // 16:1 for Gearbox, 5:1 for Chain
    public static final double kElevatorPivotDeadBand = 0;
    public static final double kElevatorPivotOffSet = 0;

    public static final double kEPivotCruiseVelocity = 0.4;//0.25 // 0.5
    public static final double kElevatorPivotCruiseAcceleration = 0.6; // 0.5
    // public static final double kElevatorPivotJerk = 200.0; // TODO
    public static final double kElevatorPivotGroundIntake = -0.001;
  }

  public static final class RollerConstants {

    public static final int kMotorID = 62;  
    public static final double kPMotor = 0.05;
    public static final double kIMotor = 0;
    public static final double kDMotor = 0;
    public static final double kVMotor = 0;

    public static final double kMaxVelocityRPS =  100;
    public static final double kMinVelocityRPS = -100;

    public static final double kNeutralDeadband = 0.01; // In revolutions!

    public static final double kIntakePower  = -5;
    public static final double kOuttakePower = 5;

  }

  public static final class ClawConstants {

    public static final int kMotorID = 61;

    public static final double kPMotor = 40;//60 //MAX 4000
    // 3 degrees error / 360 degrees * kP = 0.4V
    public static final double kIMotor = 0;
    public static final double kDMotor = 0;
    public static final double kVMotor = 0;
    public static final double kSMotor = 0; //Static Friction
    public static final double kGMotor = 0; //Gravity

    public static final double kCruiseVelocity = 50; // TODO change later
    public static final double kAcceleration = kCruiseVelocity * 2; //Rotations per second squared
    public static final double kJerk = kAcceleration * 10; //Rotations per second cubed

    public static final double kAlgaeHoldPosition = 0.169; 
    public static final double kCoralHoldPosition = -0.01;//-0.004
    public static final double kAlgaeReleasePosition = 0.169; 
    public static final double kCoralReleasePosition = 0.023438;//-0.008789;  
    public static final double kAlgaeOpenPosition = 0.169; 
    public static final double kCoralOpenPosition = -0.008789; 
    public static final double kStowPosition = 0; 
    public static final double kClosedPosition = -0.0488;
    public static final double kStationPosition = 0.08;
    public static final double kStationHoldPosition = -0.04;


    
 }

  public static final class WristConstants{
    public static final int kMotorID = 54;
  
    public static final double kPMotor =  4.147 * 6; // 4.147  max
    public static final double kItMotor = 0;
    public static final double kDMotor = 0;
    public static final double kVMotor = 0; //0.12;
    public static final double kSMotor = 0; //0.015; //Static Friction
    public static final double kGMotor = 0.6; //1.928; //0.015; //Gravity
    // kGMotor = -2.310 + (0.002420 * theta)
    public static final double kCruiseVelocity = 2;
    public static final double kAcceleration = kCruiseVelocity * 1.5; // Double velocity - Rotations per second squared
    public static final double kJerk = kAcceleration * 10; // 10 times accel - Rotations per second cubed

    public static final double kStowPosition = -0.1;//-0.15//0.092//0.0
    public static final double kIntermediatePosition = -0.123; //-0.06//-0.4
    public static final double kStationPosition = -0.6043;//-0.89//-0.3//0.54
    public static final double kL23Position = -0.3; // was -0.6
    public static final double kL14Position = -0.8;
    public static final double kL234AlagePosition = -0.8;
    public static final double kWristStowPosition = 0.0;
    public static final double kWristL3Position = -0.28;
    public static final double kWristL2Position = -0.28;
    public static final double kWristL1Position = -0.21;
    public static final double kWristL4Position = -0.318;
    public static final double kWristGroundIntake = -0.808;


    public static final double kMaxPosition = 0.14; //156
    public static final double kMinPosition = 0;

    public static final double kSpeed = 0.5;
    public static final int kPigeonID = 2;
  }

  public static final class SuperSystemConstants {
    public enum NamedPositions { // positions are all in human-readable, converted internally
      Stow(         ExecutionOrder.WRT_ELV_PVT, 0.01, 0.05, -0.4, -0.4),
      SemiStow(     ExecutionOrder.WRT_ELV_PVT, 0.11, 0.05, -0.4, -0.4),
      GroundIntake( ExecutionOrder.PVT_WRT_ELV, 0.01, 0.562, -0.808, -0.4),
      Station(      ExecutionOrder.WRT_PVT_ELV, 0.1, 0.0, -0.6043, -0.4),
      Processor(    ExecutionOrder.WRT_ELV_PVT, 0.04, 0.05, -0.1, -0.4),
      Net(          ExecutionOrder.WRT_ELV_PVT, 0.24, 0.05, -0.1, -0.4),
      Cage(         ExecutionOrder.WRT_ELV_PVT, 0.11, 0.05, -0.1, -0.4),
      L1(           ExecutionOrder.WRT_PVT_ELV, 0.24, 0.0, -0.21, -0.123),
      L2(           ExecutionOrder.WRT_PVT_ELV, 0.24, 0.1, -0.28, -0.123),
      L3(           ExecutionOrder.WRT_PVT_ELV, 0.24, 1.36, -0.28, -0.123),
      L4(           ExecutionOrder.WRT_PVT_ELV, 0.24, 3.15, -0.318, -0.123),
      L2L3(         ExecutionOrder.WRT_PVT_ELV, 0.24, 0.5, -0.318, -0.4),
      L3L4(         ExecutionOrder.WRT_PVT_ELV, 0.24, 1.5, -0.318, -0.4),
      ;
      // todo: we might need pre position for wrist
      public double intermediateWristPosition, finalWristPosition, elevatorPosition, pivotPosition; // rotations not degrees
      public ExecutionOrder executionOrder;
      NamedPositions(ExecutionOrder eo, double pp, double ep, double fwp, double iwp) {
        intermediateWristPosition = iwp; // TODO: add conversions
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

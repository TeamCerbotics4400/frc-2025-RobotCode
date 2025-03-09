// Copyright (c) 2025 FRC 4400//
package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static final String rioCanbus = "rio";
  public static final String canivoreCanbus = "Swerve_Canivore";
  public static final boolean tuningMode = true;

  public static Mode currentMode = Mode.REAL;
  public static final boolean needToLog = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class DriveConstants {

    public static final double kDriveGearRatio = 4.59;
    public static final double kTurnGearRatio = 13.3714;

    // Distance between left and right wheels
    public static final double kTrackWidth = 0.6096;
    // Distance between front and back wheels
    public static final double kWheelBase = 0.635; // 20.25

    public static final double MaxAngularRate = 2 * Math.PI;
    public static final double MaxLinearSpeed = 9.8; // 22.8

    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = Units.feetToMeters(22.8);
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond =
        kPhysicalMaxAngularSpeedRadiansPerSecond / 3;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double traslationP = 0.0,
        traslationD = 0.2,
        rotationP = 0.4,
        rotationD = 0.0;

    public static final int PIGEON_ID = 15;

  }

  public class FieldConstants {

    /* PID OUTPUT 1 WITH OBJECT AVOIDING ------------------------------------------------------------- */
    public static final Pose2d[] alignBluePose = {
      new Pose2d(2.950, 4.1, new Rotation2d(Units.degreesToRadians(180))),//A
      new Pose2d(2.950, 3.860, new Rotation2d(Units.degreesToRadians(180))),//B

      new Pose2d(3.549, 2.709, new Rotation2d(Units.degreesToRadians(-120))),//C
      new Pose2d(3.871, 2.553, new Rotation2d(Units.degreesToRadians(-120))),//D   
      
      new Pose2d(5.109, 2.650, new Rotation2d(Units.degreesToRadians(-60))),//E
      new Pose2d(5.382, 2.787, new Rotation2d(Units.degreesToRadians(-60))),//F  

      new Pose2d(6.320, 3.860, new Rotation2d(Units.degreesToRadians(0))),//G
      new Pose2d(6.320, 4.150, new Rotation2d(Units.degreesToRadians(0))),//H

      new Pose2d(5.490, 5.450, new Rotation2d(Units.degreesToRadians(60))),//I
      new Pose2d(5.128, 5.478, new Rotation2d(Units.degreesToRadians(60))),//J
      
      new Pose2d(3.920, 5.430, new Rotation2d(Units.degreesToRadians(120))),//K
      new Pose2d(3.520, 5.270, new Rotation2d(Units.degreesToRadians(120))) //L                           
          };

    public static final Pose2d[] alignRedPose = {
      FlippingUtil.flipFieldPose(new Pose2d(2.950, 4.1, new Rotation2d(Units.degreesToRadians(180)))), // A
      FlippingUtil.flipFieldPose(new Pose2d(2.950, 3.860, new Rotation2d(Units.degreesToRadians(180)))), // B
        
      FlippingUtil.flipFieldPose(new Pose2d(3.549, 2.709, new Rotation2d(Units.degreesToRadians(-120)))), // C
      FlippingUtil.flipFieldPose(new Pose2d(3.871, 2.553, new Rotation2d(Units.degreesToRadians(-120)))), // D   
        
      FlippingUtil.flipFieldPose(new Pose2d(5.109, 2.650, new Rotation2d(Units.degreesToRadians(-60)))), // E
      FlippingUtil.flipFieldPose(new Pose2d(5.382, 2.787, new Rotation2d(Units.degreesToRadians(-60)))), // F  
        
      FlippingUtil.flipFieldPose(new Pose2d(6.320, 3.860, new Rotation2d(Units.degreesToRadians(0)))), // G
      FlippingUtil.flipFieldPose(new Pose2d(6.320, 4.150, new Rotation2d(Units.degreesToRadians(0)))), // H
        
      FlippingUtil.flipFieldPose(new Pose2d(5.490, 5.450, new Rotation2d(Units.degreesToRadians(60)))), // I
      FlippingUtil.flipFieldPose(new Pose2d(5.128, 5.478, new Rotation2d(Units.degreesToRadians(60)))), // J
        
      FlippingUtil.flipFieldPose(new Pose2d(3.920, 5.430, new Rotation2d(Units.degreesToRadians(120)))), // K
      FlippingUtil.flipFieldPose(new Pose2d(3.520, 5.270, new Rotation2d(Units.degreesToRadians(120))))  // L                            
        };
        


    /* SECOND PID OUTPUT ------------------------------------------------------------------------------------  */
    public static final Pose2d[] blueSidePositions = {
        new Pose2d(3.300, 4.150, new Rotation2d(Units.degreesToRadians(180))), // A                    
        new Pose2d(3.300, 3.860, new Rotation2d(Units.degreesToRadians(180))), // B                    

        new Pose2d(3.700, 3.010, new Rotation2d(Units.degreesToRadians(-120))), // C                            
        new Pose2d(3.990, 2.870, new Rotation2d(Units.degreesToRadians(-120))), // D                            

        new Pose2d(4.963, 2.835, new Rotation2d(Units.degreesToRadians(-60))), // E
        new Pose2d(5.250, 3.030, new Rotation2d(Units.degreesToRadians(-60))), // F   

        new Pose2d(5.750, 3.860, new Rotation2d(Units.degreesToRadians(0))), // G
        new Pose2d(5.750, 4.150, new Rotation2d(Units.degreesToRadians(0))), // H

        new Pose2d(5.255, 5.039, new Rotation2d(Units.degreesToRadians(60))), // I  
        new Pose2d(4.963, 5.205, new Rotation2d(Units.degreesToRadians(60))), // J
        
        new Pose2d(4.017, 5.205, new Rotation2d(Units.degreesToRadians(120))), // K            
        new Pose2d(3.750, 5.100, new Rotation2d(Units.degreesToRadians(120)))  // L         
    };

    public static final Pose2d[] redSidePositions = {
      FlippingUtil.flipFieldPose(new Pose2d(3.300, 4.150, new Rotation2d(Units.degreesToRadians(180)))), // A                    
      FlippingUtil.flipFieldPose(new Pose2d(3.300, 3.860, new Rotation2d(Units.degreesToRadians(180)))), // B                    
  
      FlippingUtil.flipFieldPose(new Pose2d(3.700, 3.010, new Rotation2d(Units.degreesToRadians(-120)))), // C                            
      FlippingUtil.flipFieldPose(new Pose2d(3.990, 2.870, new Rotation2d(Units.degreesToRadians(-120)))), // D                            
  
      FlippingUtil.flipFieldPose(new Pose2d(4.963, 2.835, new Rotation2d(Units.degreesToRadians(-60)))), // E
      FlippingUtil.flipFieldPose(new Pose2d(5.250, 3.030, new Rotation2d(Units.degreesToRadians(-60)))), // F   
  
      FlippingUtil.flipFieldPose(new Pose2d(5.750, 3.860, new Rotation2d(Units.degreesToRadians(0)))), // G
      FlippingUtil.flipFieldPose(new Pose2d(5.750, 4.150, new Rotation2d(Units.degreesToRadians(0)))), // H
  
      FlippingUtil.flipFieldPose(new Pose2d(5.255, 5.039, new Rotation2d(Units.degreesToRadians(60)))), // I  
      FlippingUtil.flipFieldPose(new Pose2d(4.963, 5.205, new Rotation2d(Units.degreesToRadians(60)))), // J
  
      FlippingUtil.flipFieldPose(new Pose2d(4.017, 5.205, new Rotation2d(Units.degreesToRadians(120)))), // K            
      FlippingUtil.flipFieldPose(new Pose2d(3.750, 5.100, new Rotation2d(Units.degreesToRadians(120))))  // L         
  };
  

  public static final double fieldBorderMargin = 0.01;
  public static final double fieldLength = 17.29;  //Meters
  public static final double fieldWidth = 7.78;
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line
  }
      /*
       *                   F
       *   ┌───────┬─────────────────┬───────┐
       *   │       │ elevator/intake │       │
       *   │ Mod 0 │                 │ Mod 1 │
       *   │       │                 │       │
       *   ├───────┘                 └───────┤
       *   │                                 │
       *   │            Modules              │
       * L │            Diagram              │ R
       *   │                                 │
       *   │                                 │
       *   │                                 │
       *   ├───────┐                 ┌───────┤
       *   │       │                 │       │
       *   │ Mod 3 │                 │ Mod 2 │
       *   │       │enter coral      │       │
       *   └───────┴─────────────────┴───────┘
       *                  B
       */
      public static final class IntakeConstants{
        public static final int leftIntakeMotorId = 51;
        public static final int rightIntakeMotorId = 41;
        public static final int laserCanId = 35;

        
    }

    public static final class ElevatorConstants{
        public static final int leftElevatorMotorId = 55;
        public static final int rightElevatorMotorId = 52;
        public static final int encoderPortId1 = 9;  //A
        public static final int encoderPortId2 = 8;  //B

        public static  double kP = 3.4,
                                   kD = 0.01,
                                   kI = 0,
                                   kS = 0.015,//10206
                                   kV = 0.35,//11818
                                   kG = 0.021,
                                   kA = 0.2; //0018792

        public static  double maxVelElevator = 2.6,
                                   maxAccElevator = 2.6;

        public static final double [] elevatorPosition = {
          0.15, //L1
          0.4,  //L2
          0.91, //L3
          1.67  //L4
        };
    }

    public static final class ClimberConstants{
        public static final int CLIMBERKRAKENID = 53;
        public static final int CLIMBERSPARKMAXID = 40;

    }

    public static final class IntakeAlgaeConstants {
      public static final int pivotAlgaeMotorId = 17;
      public static final int rollerAlgaeMotorId = 50;
    }


    public static final class VisionConstants {
      public static final int aprilTagCount = 22;
      public static final double targetLogTimeSecs = 0.1;

      public static final String neuralLimelight = "limelight-neural";
      public static final String tagLimelightName = "limelight-lowtag";

    public static double xyStdDevCoefficient = 0.2;
    public static double thetaStdDevCoefficient = 0.4;

      public static final int main_Pipeline = 0,
          upper_Pipeline = 1,
          medium_Pipeline = 2,
          lower_Pipeline = 3;

    // Cam mounted facing forward, half a meter forward of center, half a meter up from center,
    // values are in meters.
    public static final Pose3d kRobotToCam1 =   //LL that is below the elevator
        new Pose3d(
            Units.inchesToMeters(-7.5),0, Units.inchesToMeters(11),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(0),
                Units.degreesToRadians(180)));    

      /*   _________
       /   _____/__  __ ____   ____
       \_____  \\  \/ // __ \ /    \
      /_______  /\   /\  ___/|   |  \
              \/  \_/  \___  >___|  /
                           \/     \/
              */
  }


  public static AprilTagFields tagLayout = AprilTagFields.k2025ReefscapeWelded;
  public static AprilTagFields andy = AprilTagFields.k2025ReefscapeAndyMark;

  public static  AprilTagFieldLayout aprilTaglayout  =  AprilTagFieldLayout.loadField(tagLayout);
  public static  AprilTagFieldLayout andymarkTag  =  AprilTagFieldLayout.loadField(andy);



}
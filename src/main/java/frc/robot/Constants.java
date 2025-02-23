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

    public static final double fieldBorderMargin = 0.01;

    /* Reef pipes aligntment location
    * (Might need revision later to accomodate robot dimensions)
   
    Middle AB = 3.10, 4.01
    Middle CD = 3.64, 2.56
    */
    
    public static final Pose2d[] alignBluePose = {
      new Pose2d(2.95, 4.28, new Rotation2d(Units.degreesToRadians(180))),//A
      new Pose2d(2.95, 3.79, new Rotation2d(Units.degreesToRadians(180))),//B
      new Pose2d(3.64, 2.56, new Rotation2d(Units.degreesToRadians(180))),//C
      new Pose2d(3.64, 2.56, new Rotation2d(Units.degreesToRadians(180))),//D   
      new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),//E
      new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),  
      new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),
      new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),
      new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),
      new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),
      new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),
      new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),
      new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),
      new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),                            
          };

          public static final Pose2d[] alignRedPose = {//Needs correction
            new Pose2d(2.95, 4.28, new Rotation2d(Units.degreesToRadians(180))),//A
            new Pose2d(2.95, 3.79, new Rotation2d(Units.degreesToRadians(180))),//B
            new Pose2d(3.64, 2.56, new Rotation2d(Units.degreesToRadians(180))),//C
            new Pose2d(3.64, 2.56, new Rotation2d(Units.degreesToRadians(180))),//D   
            new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),//E
            new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),  
            new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),
            new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),
            new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),
            new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),
            new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),
            new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),
            new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),
            new Pose2d(3.10, 4.01, new Rotation2d(Units.degreesToRadians(180))),                            
                };

    public static final Pose2d[] blueSidePositions = {
        new Pose2d(3.30, 4.15, new Rotation2d(Units.degreesToRadians(180))), // A                    Correct real
        new Pose2d(3.30, 3.79, new Rotation2d(Units.degreesToRadians(180))), // B                    Correct real
        new Pose2d(3.69, 2.89, new Rotation2d(Units.degreesToRadians(-120))), // C                           Correct Real
        new Pose2d(4.00, 2.75, new Rotation2d(Units.degreesToRadians(-120))), // D                           Correct real
        new Pose2d(5.039, 2.725, new Rotation2d(Units.degreesToRadians(-60))), // E
        new Pose2d(5.27, 2.99, new Rotation2d(Units.degreesToRadians(-60))), // F   5
        new Pose2d(5.900, 3.859, new Rotation2d(Units.degreesToRadians(0.1))), // G
        new Pose2d(5.900, 4.195, new Rotation2d(Units.degreesToRadians(-0.1))), // H
        new Pose2d(5.31, 5.181, new Rotation2d(Units.degreesToRadians(60))), // I  8
        new Pose2d(5.060, 5.356, new Rotation2d(Units.degreesToRadians(60))), // J
        new Pose2d(3.94, 5.28, new Rotation2d(Units.degreesToRadians(120))), // K                    Correct 
        new Pose2d(3.68, 5.09, new Rotation2d(Units.degreesToRadians(120)))  // L         Last one / Correct
    };

    public static final Pose2d[] redSidePositions = {   //Needs correction
        FlippingUtil.flipFieldPose(new Pose2d(3.078, 4.195, new Rotation2d())), // A
        FlippingUtil.flipFieldPose(new Pose2d(3.078, 3.859, new Rotation2d())), // B
        FlippingUtil.flipFieldPose(new Pose2d(3.625, 2.860, new Rotation2d())), // C
        FlippingUtil.flipFieldPose(new Pose2d(3.925, 2.694, new Rotation2d())), // D
        FlippingUtil.flipFieldPose(new Pose2d(5.039, 2.725, new Rotation2d())), // E
        FlippingUtil.flipFieldPose(new Pose2d(5.348, 2.869, new Rotation2d())), // F
        FlippingUtil.flipFieldPose(new Pose2d(5.900, 3.859, new Rotation2d())), // G
        FlippingUtil.flipFieldPose(new Pose2d(5.900, 4.195, new Rotation2d())), // H
        FlippingUtil.flipFieldPose(new Pose2d(5.338, 5.181, new Rotation2d())), // I
        FlippingUtil.flipFieldPose(new Pose2d(5.060, 5.356, new Rotation2d())), // J
        FlippingUtil.flipFieldPose(new Pose2d(3.914, 5.377, new Rotation2d())), // K
        FlippingUtil.flipFieldPose(new Pose2d(3.615, 5.202, new Rotation2d()))  // L
    };

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
                                   kV = 0.68,//11818
                                   kG = 0.021,
                                   kA = 0.2; //0018792

        public static  double maxVelElevator = 1.75,
                                   maxAccElevator = 1.0;

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
  
  public static  AprilTagFieldLayout aprilTaglayout  =  AprilTagFieldLayout.loadField(tagLayout);



}
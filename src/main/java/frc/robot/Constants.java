// Copyright (c) 2025 FRC 4400//
package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public class Constants {

  public static final String rioCanbus = "rio";
  public static final String canivoreCanbus = "Swerve_Canivore";

  public static Mode currentMode = Mode.SIM;
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
    * (Blue side)
    * A: (X = 3.078, Y = 4.195, θ = 0.0°)
    * B: (X = 3.078, Y = 3.859, θ = 0.0°)
    * C: (X = 3.625, Y = 2.860, θ = 60.0°)
    * D: (X = 3.925, Y = 2.694, θ = 60.0°)
    * E: (X = 5.039, Y = 2.725, θ = 120.0°)
    * F: (X = 5.348, Y = 2.869, θ = 120.0°)
    * G: (X = 5.900, Y = 3.859, θ = 180.0°)
    * H: (X = 5.900, Y = 4.195, θ = 180.0°)
    * I: (X = 5.338, Y = 5.181, θ = 240.0°)
    * J: (X = 5.060, Y = 5.356, θ = 240.0°)
    * K: (X = 3.914, Y = 5.377, θ = 300.0°)
    * L: (X = 3.615, Y = 5.202, θ = 300.0°)
    */

    public static final Pose2d[] blueSidePositions = {
        new Pose2d(3.23, 4.18, new Rotation2d(Units.degreesToRadians(180))), // A                    Correct
        new Pose2d(3.23, 3.82, new Rotation2d(Units.degreesToRadians(180))), // B                    Correct
        new Pose2d(3.625, 2.860, new Rotation2d(Units.degreesToRadians(-120))), // C
        new Pose2d(3.925, 2.694, new Rotation2d(Units.degreesToRadians(-120))), // D
        new Pose2d(5.039, 2.725, new Rotation2d(Units.degreesToRadians(-60))), // E
        new Pose2d(5.27, 2.99, new Rotation2d(Units.degreesToRadians(-60))), // F   5
        new Pose2d(5.900, 3.859, new Rotation2d(Units.degreesToRadians(0.1))), // G
        new Pose2d(5.900, 4.195, new Rotation2d(Units.degreesToRadians(-0.1))), // H
        new Pose2d(5.31, 5.181, new Rotation2d(Units.degreesToRadians(60))), // I  8
        new Pose2d(5.060, 5.356, new Rotation2d(Units.degreesToRadians(60))), // J
        new Pose2d(4.09, 5.24, new Rotation2d(Units.degreesToRadians(120))), // K                    Correct 
        new Pose2d(3.78, 5.01, new Rotation2d(Units.degreesToRadians(120)))  // L         Last one / Correct
    };

    public static final Pose2d[] redSidePositions = {
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
       *   │       │       intake    │       │
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
       *   │       │      arm        │       │
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

        public static final double kP = 2.3,
                                   kD = 0,
                                   kI = 0,
                                   kS = 0,
                                   kV = 0,
                                   kG = 0,
                                   kA = 0; 

        public static final double maxVelElevator = 1.75,
                                   maxAccElevator = 0.75;

        public static final double [] elevatorPosition = {
          0.15, //L1
          0.4,  //L2
          0.91, //L3
          1.67  //L4
        };
    }

    public static final class ClimberConstants{
        public static final int climberMotorId = 17;
    }

    public static final class IntakeAlgaeConstants {
      public static final int pivotAlgaeMotorId = 57;
      public static final int rollerAlgaeMotorId = 59;
    }


    public static final class VisionConstants {
      public static final int aprilTagCount = 22;

      public static final String neuralLimelight = "limelight-neural";
      public static final String tagLimelightName = "limelight-tags";

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
  /*public static  AprilTagFieldLayout aprilTaglayout;
{
  try {
    aprilTaglayout = new AprilTagFieldLayout(
        Path.of(Filesystem.getDeployDirectory().getPath(), "apriltags", "2025-official" + ".json")
    );
} catch (IOException e) {
    throw new RuntimeException("Failed to load AprilTagFieldLayout", e);
}
}
*/
}
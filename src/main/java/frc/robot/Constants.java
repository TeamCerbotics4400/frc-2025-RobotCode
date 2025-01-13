// Copyright (c) 2025 FRC 4400//
package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {

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

    public static final String CANBUS_STRING = "Swerve_Canivore";
    public static final int PIGEON_ID = 15;

    public static final double driveOdometryRatio = 7 / 6.3;
  }

  public class FieldConstants {

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

    public static class ModuleConstants {
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

      /** Front Left module 0 * */
      public static final byte kFrontLeftDriveMotorId = 1;

      public static final byte kFrontLeftSteerMotorId = 2;
      public static final byte kFrontLeftEncoderId = 3;
      public static final double kFrontLeftEncoderOffset = -0.143798828125;

      /** Front Right module 1 * */
      public static final byte kFrontRightDriveMotorId = 4;

      public static final byte kFrontRightSteerMotorId = 5;
      public static final byte kFrontRightEncoderId = 6;
      public static final double kFrontRightEncoderOffset = 0.4609375;

      /** Back Left module 2 * */
      public static final byte kBackLeftDriveMotorId = 10;

      public static final byte kBackLeftSteerMotorId = 11;
      public static final byte kBackLeftEncoderId = 12;
      public static final double kBackLeftEncoderOffset = -0.06396484375;

      /** Back Right module 3 * */
      public static final byte kBackRightDriveMotorId = 7;

      public static final byte kBackRightSteerMotorId = 8;
      public static final byte kBackRightEncoderId = 9;
      public static final double kBackRightEncoderOffset = 0.40673828125;

      /** To change offsets easily * */
      // Minus  = Counterclockwise
      // Plus = Clockwise
    }

    public static final class VisionConstants {

      public static final String neuralLimelight = "limelight-neural";
      public static final String tagLimelightName = "limelight-tags";

      public static final int main_Pipeline = 0,
          upper_Pipeline = 1,
          medium_Pipeline = 2,
          lower_Pipeline = 3;

      /*   _________
       /   _____/__  __ ____   ____
       \_____  \\  \/ // __ \ /    \
      /_______  /\   /\  ___/|   |  \
              \/  \_/  \___  >___|  /
                           \/     \/
              */
    }
  }
}

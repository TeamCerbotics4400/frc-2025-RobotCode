package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

/** Gripper subsystem hardware interface. */
public interface IntakeIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class IntakeIOInputs {
    public double leftMotorappliedVolts = 0.0;
    public double leftMotortempCelcius = 0.0;

    public double rightMotorappliedVolts = 0.0;
    public double rightMotortempCelcius = 0.0;

    public boolean sensor = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the intake open loop at the specified voltage. */
  public default void setVoltage(double armVolt) {}

}
package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

/** Gripper subsystem hardware interface. */
public interface ClimberIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ClimberIOInputs {
    public double appliedVolts = 0.0;
    public double tempCelcius = 0.0;
    public double position = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setPosition(double position) {}

  /** Run the intake open loop at the specified voltage. */
  public default void setVoltage(double voltage) {}
}
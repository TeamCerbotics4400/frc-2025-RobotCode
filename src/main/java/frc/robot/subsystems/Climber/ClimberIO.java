package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

/** Gripper subsystem hardware interface. */
public interface ClimberIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ClimberIOInputs {
    public double sparkAppliedVolts = 0.0;
    public double sparkTempCelcius = 0.0;
    public double sparkPosition = 0.0;

    public double talonAppliedVolts = 0.0;
    public double talonTempCelcius = 0.0;
    public double talonPosition = 0.0;

  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Run the intake open loop at the specified voltage. */
  public default void setSparkMaxVoltage(double voltage) {}

  public default void setTalonFXVoltage(double voltage) {}

  public default void setSparkPosition(double position) {}

  public default void setTalonFXPosition(double position) {}

}
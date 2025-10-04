package frc.robot.Subsystems.Climber;
import org.littletonrobotics.junction.AutoLog;



/** Gripper subsystem hardware interface. */
public interface ClimberIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ClimberIOInputs {
    public double pidOutput = 0.0;
    public double climberFxAppliedVolts = 0.0;
    public double climberFxPosition = 0.0;
    public double sparkAppliedVolts = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Run the intake open loop at the specified voltage. */
  public default void setSparkMaxVoltage(double voltage) {}

  public default void setCageMotorVolatge(double voltage){}
  

  public default void setTalonFXVoltage(double voltage) {}

  public default void setTalonFXPosition(double position) {}

}
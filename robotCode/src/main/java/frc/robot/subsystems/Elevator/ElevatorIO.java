
package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorInputs {
    public double elevatorPosition = 0.0;

    public double rightElevatorTemp = 0.0;
    public double rightElevatorSupplyCurrent = 0.0;
    public double rightElevatorVoltage = 0.0;

    public double leftElevatorTemp = 0.0;
    public double leftElevatorSupplyCurrent = 0.0;
    public double leftElevatorVoltage = 0.0;
  }

  /**
   * Update the inputs for the arm extension
   *
   * @param inputs The inputs to update
   */
  public default void updateInputs(ElevatorInputs inputs){}

  /**
   * Output a set voltage to the arm extension motor
   *
   * @param volts Voltage to output
   */
  public default void setVoltage(double volts, double feedforward){}

  public default void enableBreak(boolean enable){}

  /** Optimize status signals for running sysID */
  default void optimizeForSysID() {}
}
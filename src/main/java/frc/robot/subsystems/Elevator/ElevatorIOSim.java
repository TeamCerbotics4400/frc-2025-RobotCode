package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

  private final ElevatorSim profile1 =
      new ElevatorSim(0.11515, 0.0017514, DCMotor.getFalcon500Foc(1), 0.0, 1.74, false, 0.0);

  public double appliedVolts = 0.0;

  public ElevatorIOSim() {
    profile1.setState(0.0, 0.0);
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    profile1.update(0.02);

    inputs.elevatorPosition = profile1.getPositionMeters();

    inputs.leftElevatorCurrent = profile1.getCurrentDrawAmps();

    profile1.setInputVoltage(0.0);
  }

  @Override
  public void setVoltage(double v1, double v2) {
    runVolts(
        v1
            + v2);
  }

  public void runVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    profile1.setInputVoltage(volts);
  }
}
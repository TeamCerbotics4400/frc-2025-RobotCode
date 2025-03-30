package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

  private final ElevatorSim profile1 =
      new ElevatorSim(0.11515, 0.0017514, DCMotor.getFalcon500Foc(1), 0.0, 34, false, 0.0);

  public double appliedVolts = 0.0;

  private double offsetVal = 0.0;
  private boolean resetEncoder = false;
  public ElevatorIOSim() {
    profile1.setState(0.0, 0.0);
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    profile1.update(0.02);

    inputs.elevatorPosition = getPosition(0);

    inputs.leftElevatorCurrent = profile1.getCurrentDrawAmps();

    profile1.setInputVoltage(0.0);
  }

  @Override
  public void setVoltage(double v1, double v2) {
    runVolts(
        v1
            + v2);
  }

  @Override
  public void resetEncoder(){
    offsetVal = profile1.getPositionMeters(); 
    Logger.recordOutput("Elevator/Maybe", resetEncoder);
    resetEncoder = true;// Store the current position as the offset

  }

  public double getPosition(double reset){
    return profile1.getPositionMeters() - offsetVal; // Return the relative position
  }

  public void runVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    profile1.setInputVoltage(volts);
  }
}
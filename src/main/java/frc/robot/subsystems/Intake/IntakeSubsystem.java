package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  /*Io and inputs */
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void setVoltageVoid(double voltage, double leftVolt) {
    io.setVoltage(voltage,leftVolt);
  }

  public void setVelocity(double rightVoltage, double leftVoltage) {
    double valRight = rightVoltage / 60;
    double valLeft = leftVoltage / 60;
    io.setVelocity(valRight,valLeft);
  }

  public void stopMotors(){
    io.stopMotors();
  }

  public Command setVoltageCommand(double voltage, double leftVolt) {
    return run(() -> io.setVoltage(voltage, leftVolt));
  }

  public boolean hasGamePieceInside() {
    return inputs.sensor;
  }
}
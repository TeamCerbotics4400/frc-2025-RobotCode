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

  public void setVoltageVoid(double voltage) {
    io.setVoltage(voltage);
  }

  public Command setVoltageCommand(double voltage) {
    return run(() -> io.setVoltage(voltage));
  }

  public boolean hasGamePieceInside() {
    return inputs.sensor;
  }
}
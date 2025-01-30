package frc.robot.Subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

  /*Io and inputs */
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public Command setClimberVoltage(double voltage) {
    return run(() -> io.setVoltage(voltage));
  }

  public Command setPosition(double position) {
    return run(() -> io.setPosition(position));
  }
}
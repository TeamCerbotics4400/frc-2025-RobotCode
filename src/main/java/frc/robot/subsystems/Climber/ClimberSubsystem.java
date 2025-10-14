package frc.robot.Subsystems.Climber;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

  /* Io and inputs */
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private PIDController m_controller = new PIDController(1, 0, 0.0);

  private boolean enablePID = false;
  private double setpoint = 0.0;

  public static enum ClimbingState {
    PREPARING_CLIMB,
    CLIMBING
  }

  public ClimbingState climbState = ClimbingState.PREPARING_CLIMB;

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    if (enablePID) {
      double output = m_controller.calculate(inputs.climberFxPosition, setpoint);
      io.setTalonFXVoltage(output);

      Logger.recordOutput("Climber/Setpoint", setpoint);
      Logger.recordOutput("Climber/Position", inputs.climberFxPosition);
  }
}

  public PIDController getController() {
    return m_controller;
  }

  public Command stopMotor() {
    return run(() -> io.stopMotor());
  }

  public Command setKrakenVoltage(double voltage) {
    return run(() -> io.setTalonFXVoltage(voltage));
  }

 /*  public Command setSparkMaxVolatge(double voltage){
return run (() -> io.CageMotorVoltage(voltage)); 

  }*/ 

  public Command goToPosition(double position) {
    return Commands.runOnce(() -> {
      m_controller.reset();
      setpoint = position;
      enablePID = true;
    }, this);
  }

}
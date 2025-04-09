package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.IntakeAlgae.IntakeAlgaeSubsystem.AlgaeState;

import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  public boolean finishedIntakeSequence = true;

  /*Io and inputs */
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public static enum IntakeState{
    INTAKING,
    FINISHED
  }
    private IntakeState intakeState = IntakeState.FINISHED;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/Finished sequence", finishedIntakeSequence);
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

  public IntakeState getState() {
    return intakeState;
  }

  public IntakeState changeState(IntakeState state) {
    intakeState = state;
    return intakeState;
  }

}
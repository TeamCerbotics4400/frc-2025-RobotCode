package frc.robot.Subsystems.IntakeAlgae;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeAlgaeConstants;

public class IntakeAlgaeSubsystem extends SubsystemBase {

  private final IntakeAlgaeIO io;
  private final IntakeAlgaeIOInputsAutoLogged inputs = new IntakeAlgaeIOInputsAutoLogged();
 // private PIDController m_controller = new PIDController(0.0325, 0, 0.0025);
  private boolean enablePID = false;
  private Debouncer currentFilter = new Debouncer(0.5, DebounceType.kBoth);

  private TrapezoidProfile.Constraints m_profile = new TrapezoidProfile.Constraints(IntakeAlgaeConstants.maxVel, IntakeAlgaeConstants.maxXLr8tion);
  private ProfiledPIDController m_controller = new ProfiledPIDController(0.055, 0, 0.005 , m_profile);

  public static enum AlgaeState {
    FLOORPOSITION,
    REEFPOSITION,
    BACKPOSITION

  }

  private AlgaeState systemStates = AlgaeState.BACKPOSITION;

  public IntakeAlgaeSubsystem(IntakeAlgaeIO io) {
    this.io = io;

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeAlgae", inputs);
    if (enablePID) {
      io.setVoltagePiv(
          m_controller.calculate(inputs.positionPiv));
    }

    Logger.recordOutput("IntakeAlgae/PID output", m_controller.calculate(inputs.positionPiv));
    Logger.recordOutput("IntakeAlgae/PID setpoint", m_controller.getSetpoint().position);
    Logger.recordOutput("IntakeAlgae/PID enables", enablePID);
    Logger.recordOutput("IntakeAlgae/Algae Detected", currentFilter.calculate(inputs.rollerMotorCurrent > 54));

    if (DriverStation.isDisabled()) {
      enablePID = false;
    }
  }

  public ProfiledPIDController getController() {
    return m_controller;
  }
  public double getAmperage() {
    return inputs.rollerMotorCurrent;
  }

  public double getPivotPosition() {
    return inputs.positionPiv;
  }

  public void resetController() {
    getController().reset(inputs.positionPiv);
  }

  public void setVoltagePivVoid(double pivotVolt) {
    io.setVoltagePiv(pivotVolt);
  }

  public void stopMotors() {
    io.stopMotors();
  }

  public Command setVoltageCommandPiv(double pivotVolt) {
    return run(() -> io.setVoltagePiv(pivotVolt));
  }

  public Command setVoltageCommandRoll(double rollerVolt) {
    return run(() -> io.setVoltageRoll(rollerVolt));
  }

  public AlgaeState getState() {
    return systemStates;
  }

  public AlgaeState changeState(AlgaeState state) {
    systemStates = state;
    return systemStates;
  }

  public double getRollerCurrent() {
    return inputs.rollerMotorCurrent;
  }

  public Command goToPosition(double position, AlgaeState state) {
    Command ejecutable = Commands.runOnce(
        () -> {
          getController().reset(inputs.positionPiv);
          getController().setGoal(position);
          enablePID = true;
          systemStates = state;
        },
        this);
    return ejecutable;
  }

  public void goToPositionVoid(double position) {
    m_controller.setGoal(position);
    enablePID = true;
  }

  public Command goToPositionVoltage(double position) {
    Command ejecutable = Commands.runOnce(
        () -> {
          getController().reset(inputs.positionPiv);
          getController().setGoal(position);
          enablePID = true;
        },
        this);
    return ejecutable;
  }

  public void setVoltageRollerVoid(double voltage) {
    io.setVoltageRoll(voltage);
  }

  public boolean isGamePieceInside() {
    return inputs.rollerMotorCurrent > 40;
  }
}

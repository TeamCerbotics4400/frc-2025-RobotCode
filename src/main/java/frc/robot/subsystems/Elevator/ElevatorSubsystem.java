// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.Util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorSubsystem extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();
  private boolean enablePID = false;

    /*Set the Maximun velocity and acceleration, needs to be tuned according to your robot*/
    private TrapezoidProfile.Constraints m_profile = new TrapezoidProfile.Constraints(maxVelElevator, maxAccElevator);

    /*Main PID Controller using the constrains as reference */
    private ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, m_profile);

    /*FeedForward Model for extra presicion */
    private ElevatorFeedforward m_ElevatorFeedforward = new ElevatorFeedforward(kS, kG, kV,kA);

    /* Tools to visualize the elevator position and setpoint on Advantage scope */
    private final ElevatorVisualizer m_visualizerPosition = new ElevatorVisualizer("Elevator Position",Color.kBlack);
    private final ElevatorVisualizer m_visualizerSetpoint = new ElevatorVisualizer("Elevator Setpoint",Color.kRed);

    /* Selector to change elevator to break/coast mode */
    private SendableChooser<String> elevatorModeChooser = new SendableChooser<>();

    /* Tunable numbers */
    LoggedTunableNumber logkS = new LoggedTunableNumber("ElevatorTunable/kS",kS);
    LoggedTunableNumber logkG = new LoggedTunableNumber("ElevatorTunable/kG",kG);
    LoggedTunableNumber logkA = new LoggedTunableNumber("ElevatorTunable/kA",kA);
    LoggedTunableNumber logkV = new LoggedTunableNumber("ElevatorTunable/kV",kV);
    LoggedTunableNumber logkP = new LoggedTunableNumber("ElevatorTunable/kP",kP);
    LoggedTunableNumber logkI = new LoggedTunableNumber("ElevatorTunable/kI",kI);
    LoggedTunableNumber logkD = new LoggedTunableNumber("ElevatorTunable/kD",kD);
    LoggedTunableNumber logMaxVel = new LoggedTunableNumber("ElevatorTunable/MAXVEL",maxVelElevator);
    LoggedTunableNumber logMaxAcc = new LoggedTunableNumber("ElevatorTunable/MAXACC",maxAccElevator);

    
  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;

    elevatorModeChooser.setDefaultOption("Break", "Break");
    elevatorModeChooser.addOption("Coast", "Coast");
    elevatorModeChooser.addOption("Break", "Break");

    SmartDashboard.putData("Elevator mode",elevatorModeChooser);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    updatePID();
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/Value Error", m_controller.getPositionError());
    Logger.recordOutput("Elevator/Setpoint", m_controller.getSetpoint().position);
    Logger.recordOutput("Elevator/PID output", m_controller.calculate(inputs.elevatorPosition)
            + m_ElevatorFeedforward.calculate(m_controller.getSetpoint().velocity));
    Logger.recordOutput("Elevator/Is within Threshold", isInPosition());

    if(enablePID){
        io.setVoltage(
          m_controller.calculate(inputs.elevatorPosition),
           m_ElevatorFeedforward.calculate(m_controller.getSetpoint().velocity));
    }

    if(DriverStation.isDisabled()){
      switch(elevatorModeChooser.getSelected()){
        case "Break":
        io.enableBreak(true);
        break;

        case "Coast":
        io.enableBreak(false);
        break;

        default:
        io.enableBreak(true);
        break;
      }
    }else{
      io.enableBreak(true);
    }

    m_visualizerPosition.update(inputs.elevatorPosition);
    m_visualizerSetpoint.update(m_controller.getSetpoint().position);
  }

  public ProfiledPIDController getController(){
    return m_controller;
  }

  public double getPosition(){
    return inputs.elevatorPosition;
  }

  public Command goToPosition(Double position) {
    Command ejecutable =
        Commands.runOnce(
            () -> {
              getController().reset(inputs.elevatorPosition);
              m_controller.setGoal(position);
              enablePID = true;
            },
            this);
    return ejecutable;
  }

  public void goToPositionVoid(double position) {
    m_controller.setGoal(position);
    enablePID = true;
  }

  public void resetController(){
    getController().reset(inputs.elevatorPosition);
  }

  public Command setVoltage(double volts){
    return run (()-> io.setVoltage(volts, 0));
  }

  public void setVoltageVoid(double volts){
    io.setVoltage(volts, 0);
  }

  public boolean isWithinThreshold(double value, double target, double threshold){
    return Math.abs(value - target) < threshold;
  }

  public boolean isInPosition(){
    return isWithinThreshold(inputs.elevatorPosition, getController().getGoal().position, 0.27);
  }

  private final SysIdRoutine m_sysIdRoutinrElevator =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdElevator_State", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> setVoltageVoid(voltage.in(Volts)), null, this));
  
  public Command sysIdQuasistaticElevator(SysIdRoutine.Direction direction) {
    return m_sysIdRoutinrElevator.quasistatic(direction);
  }

  public Command sysIdDynamicElevator(SysIdRoutine.Direction direction) {
    return m_sysIdRoutinrElevator.dynamic(direction);
  }

  public void updatePID(){
    if(logMaxAcc.hasChanged(0)
    || logMaxVel.hasChanged(0)
    || logkA.hasChanged(0)
    || logkD.hasChanged(0)
    || logkG.hasChanged(0)
    || logkI.hasChanged(0)
    || logkP.hasChanged(0)
    || logkS.hasChanged(0)
    || logkV.hasChanged(0)
    ){
      m_profile = new TrapezoidProfile.Constraints(logMaxVel.get(), logMaxAcc.get());
      m_controller = new ProfiledPIDController(logkP.get(), logkI.get(), logkD.get(), m_profile);
      m_ElevatorFeedforward = new ElevatorFeedforward(logkS.get(), logkG.get(), logkV.get() ,logkA.get());
    }
  }
}

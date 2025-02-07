// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

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

import static frc.robot.Constants.ElevatorConstants.*;

import java.util.function.Supplier;

public class ElevatorSubsystem extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();
  private boolean enablePID = false;

    /*Set the Maximun velocity and acceleration, needs to be tuned according to your robot*/
    private final TrapezoidProfile.Constraints m_profile = new TrapezoidProfile.Constraints(maxVelElevator, maxAccElevator);

    /*Main PID Controller using the constrains as reference */
    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP, kI, kD, m_profile);

    /*FeedForward Model for extra presicion */
    private final ElevatorFeedforward m_ElevatorFeedforward = new ElevatorFeedforward(kS, kG, kV,kA);

    /* Tools to visualize the elevator position and setpoint on Advantage scope */
    private final ElevatorVisualizer m_visualizerPosition = new ElevatorVisualizer("Elevator Position",Color.kBlack);
    private final ElevatorVisualizer m_visualizerSetpoint = new ElevatorVisualizer("Elevator Setpoint",Color.kRed);

    /* Selector to change elevator to break/coast mode */
    private SendableChooser<String> elevatorModeChooser = new SendableChooser<>();
    
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
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/Value Error", m_controller.getPositionError());
    Logger.recordOutput("Elevator/Setpoint", m_controller.getSetpoint().position);
    Logger.recordOutput("Elevator/PID output", m_controller.calculate(inputs.elevatorPosition));
    Logger.recordOutput("Elevator/Is within Threshold", isInPosition());

    if(enablePID){
        io.setVoltage(
          m_controller.calculate(inputs.elevatorPosition),
           m_ElevatorFeedforward.calculate(m_controller.getSetpoint().position, m_controller.getSetpoint().velocity));
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

  public boolean isWithinThreshold(double value, double target, double threshold){
    return Math.abs(value - target) < threshold;
  }

  public boolean isInPosition(){
    return isWithinThreshold(inputs.elevatorPosition, getController().getGoal().position, 0.3);
  }
}
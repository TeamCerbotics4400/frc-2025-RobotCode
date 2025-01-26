package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorIOKraken implements ElevatorIO {
  private final TalonFX rightMotor;
  private final TalonFX leftMotor;

  private final TalonFXConfiguration leftConfig;
  private final TalonFXConfiguration rightConfig;

  private final DutyCycleEncoder m_encoder;

  public ElevatorIOKraken() {

    leftMotor = new TalonFX(leftElevatorMotorId, Constants.rioCanbus);
    rightMotor = new TalonFX(rightElevatorMotorId, Constants.rioCanbus);
    m_encoder = new DutyCycleEncoder(encoderPortId);

    leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftConfig.CurrentLimits.SupplyCurrentLimit = 40;
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfig.CurrentLimits.SupplyCurrentLimit = 40;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rightMotor.getConfigurator().apply(rightConfig);
  }

  public double getCurrentPosition() {
    return m_encoder.get();
  }

  /**
   * Update the inputs for the elevator
   *
   * @param inputs The inputs to update
   */
  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.elevatorPosition = getCurrentPosition();
    inputs.leftElevatorTemp = leftMotor.getDeviceTemp().getValueAsDouble();
    inputs.leftElevatorSupplyCurrent = leftMotor.getSupplyCurrent().getValueAsDouble();
    inputs.leftElevatorVoltage = leftMotor.getMotorVoltage().getValueAsDouble();

    inputs.rightElevatorTemp = rightMotor.getDeviceTemp().getValueAsDouble();
    inputs.rightElevatorSupplyCurrent = rightMotor.getSupplyCurrent().getValueAsDouble();
    inputs.rightElevatorVoltage = rightMotor.getMotorVoltage().getValueAsDouble();    
  }

  /**
   * Set the voltage output to both elevator motors
   *
   * @param volts Voltage to output
   */
  @Override
  public void setVoltage(double volts, double feedforward) {
    leftMotor.set(volts + feedforward);
    rightMotor.set(volts + feedforward);
  }

    /**
   * Set if the motor should be in brake or coast
   *
   * @param enable to enable brake
   */

  @Override
  public void enableBreak(boolean enable) {
    rightConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    if (rightConfig.MotorOutput.NeutralMode == leftConfig.MotorOutput.NeutralMode) {

    } else {
      leftConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

      leftMotor.getConfigurator().apply(leftConfig);
      rightMotor.getConfigurator().apply(rightConfig);
    }
  }
}
package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorIOKraken implements ElevatorIO {
  private final TalonFX rightMotor;
  private final TalonFX leftMotor;

  private final TalonFXConfiguration leftConfig;
  private final TalonFXConfiguration rightConfig;

  private final Encoder m_encoder;

  public ElevatorIOKraken() {

    leftMotor = new TalonFX(leftElevatorMotorId, Constants.rioCanbus);
    rightMotor = new TalonFX(rightElevatorMotorId, Constants.rioCanbus);

    m_encoder = new Encoder(8,9);
    m_encoder.reset();
    m_encoder.setDistancePerPulse(1);

    leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftConfig.CurrentLimits.SupplyCurrentLimit = 40;
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      
    rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightConfig.CurrentLimits.SupplyCurrentLimit = 40;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rightMotor.getConfigurator().apply(rightConfig);
    leftMotor.getConfigurator().apply(leftConfig);
  }

  public double getCurrentPosition() {
    return Units.inchesToMeters(m_encoder.getDistance() * 0.01066) / 2;
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
    inputs.leftElevatorCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
    inputs.leftElevatorVoltage = leftMotor.getMotorVoltage().getValueAsDouble();

    inputs.rightElevatorTemp = rightMotor.getDeviceTemp().getValueAsDouble();
    inputs.rightElevatorCurrent = rightMotor.getStatorCurrent().getValueAsDouble();
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
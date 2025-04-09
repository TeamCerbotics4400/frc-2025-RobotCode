package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

import static frc.robot.Constants.ElevatorConstants.*;

import org.littletonrobotics.junction.Logger;

public class ElevatorIOKraken implements ElevatorIO {
  private final TalonFX rightMotor;
  private final TalonFX leftMotor;

  private final TalonFXConfiguration leftConfig;
  private final TalonFXConfiguration rightConfig;

  private final DutyCycleOut m_setterControl = new DutyCycleOut(0);

  public ElevatorIOKraken() {

    leftMotor = new TalonFX(leftElevatorMotorId, Constants.rioCanbus);
    rightMotor = new TalonFX(rightElevatorMotorId, Constants.rioCanbus);

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

    rightMotor.setPosition(0);
    leftMotor.setPosition(0);

    rightMotor.getConfigurator().apply(rightConfig);
    leftMotor.getConfigurator().apply(leftConfig);
  }

  public double getCurrentPosition() {
    double val = (rightMotor.getPosition().getValueAsDouble() + leftMotor.getPosition().getValueAsDouble())/2;
    return val * 0.02988;  //56.18
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
    
    Logger.recordOutput("Elevator/Relative encoder pos",rightMotor.getPosition().getValueAsDouble());
  }


  @Override
  public void resetEncoder(){
    rightMotor.setPosition(0);
    leftMotor.setPosition(0);
    }
  /**
   * Set the voltage output to both elevator motors
   *
   * @param volts Voltage to output
   */
  @Override
  public void setVoltage(double volts, double feedforward) {
    leftMotor.setControl(m_setterControl.withOutput(volts + feedforward).withEnableFOC(true));
    rightMotor.setControl(m_setterControl.withOutput(volts + feedforward).withEnableFOC(true));
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
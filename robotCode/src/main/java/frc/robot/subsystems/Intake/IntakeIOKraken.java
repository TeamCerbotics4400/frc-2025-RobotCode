package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeIOKraken implements IntakeIO {

  /* Hardware */
  private final TalonFX leftMotor = new TalonFX(leftIntakeMotorId, Constants.rioCanbus);
  private final TalonFX rightMotor = new TalonFX(rightIntakeMotorId, Constants.rioCanbus);
  private final DigitalInput intakeSensor = new DigitalInput(digitalInputId);

  /* Configurators */
  private TalonFXConfiguration leftConfig;
  private TalonFXConfiguration rightConfig;

  public IntakeIOKraken() {

    leftConfig = new TalonFXConfiguration();
    rightConfig = new TalonFXConfiguration();

    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leftConfig.CurrentLimits.StatorCurrentLimit = 60;
    leftConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    leftConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.CurrentLimits.StatorCurrentLimit = 60;
    rightConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    rightConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    /* Apply Configurations*/
    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);

  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rightMotortempCelcius = rightMotor.getDeviceTemp().getValueAsDouble();
    inputs.rightMotorappliedVolts = rightMotor.getMotorVoltage().getValueAsDouble();

    inputs.leftMotorappliedVolts = leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftMotortempCelcius = leftMotor.getDeviceTemp().getValueAsDouble();
    inputs.sensor = !intakeSensor.get();
  }

  @Override
  public void setVoltage(double armVolt) {
    rightMotor.set(-armVolt);
    leftMotor.set(armVolt);
  }
}
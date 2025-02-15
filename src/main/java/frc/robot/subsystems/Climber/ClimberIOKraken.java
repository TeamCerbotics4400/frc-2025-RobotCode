package frc.robot.Subsystems.Climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

import static frc.robot.Constants.ClimberConstants.*;

public class ClimberIOKraken implements ClimberIO {

  private final TalonFX climberMotor = new TalonFX(climberMotorId, Constants.rioCanbus);
  private final TalonFXConfiguration climberConfig = new TalonFXConfiguration();

  private final PositionVoltage m_positionVoltage = new PositionVoltage(0);

  public ClimberIOKraken() {
    climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climberConfig.CurrentLimits.StatorCurrentLimit = 60;
    climberConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    climberConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    climberConfig.Slot0.kP = 0;

    climberMotor.setPosition(0);

    climberMotor.getConfigurator().apply(climberConfig);
  }

  @Override
  public void setVoltage(double voltage) {
    climberMotor.set(voltage);
  }

  @Override
  public void setPosition(double position) {
    climberMotor.setControl(m_positionVoltage.withPosition(position));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVolts = climberMotor.getMotorVoltage().getValueAsDouble();
    inputs.tempCelcius = climberMotor.getDeviceTemp().getValueAsDouble();
    inputs.position = climberMotor.getPosition().getValueAsDouble();
  }
    
}
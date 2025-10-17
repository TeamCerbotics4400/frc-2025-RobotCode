package frc.robot.Subsystems.IntakeAlgae;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import static frc.robot.Constants.IntakeAlgaeConstants.*;

public class IntakeAlgaeIOKraken implements IntakeAlgaeIO {

  private final TalonFX pivotMotor = new TalonFX(pivotAlgaeMotorId, Constants.rioCanbus);

  private final TalonFX rollerMotor = new TalonFX(rollerAlgaeMotorId, Constants.rioCanbus);

  private TalonFXConfiguration pivotConfig;
  private TalonFXConfiguration rollerConfig;

  private final DutyCycleOut m_setterControl = new DutyCycleOut(0);

  public IntakeAlgaeIOKraken() {

    pivotConfig = new TalonFXConfiguration();
    rollerConfig = new TalonFXConfiguration();

    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 40;
    pivotConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    pivotConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    pivotMotor.setPosition(0);

    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = 60;
    rollerConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    rollerConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    rollerConfig.Slot0.kP = 0.0;
    rollerConfig.Slot0.kD = 0.0;

    /* Apply Configurations */
    pivotMotor.getConfigurator().apply(pivotConfig);
    rollerMotor.getConfigurator().apply(rollerConfig);
  }

  public double getCurrentPosition() {
    double val = pivotMotor.getPosition().getValueAsDouble();
    return val;

  }

  @Override
  public void updateInputs(IntakeAlgaeIOInputs inputs) {
    inputs.pivotMotortempCelcius = pivotMotor.getDeviceTemp().getValueAsDouble();
    inputs.pivotMotorappliedVolts = pivotMotor.getMotorVoltage().getValueAsDouble();
    inputs.pivotMotorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();
    inputs.pivotCurrentRpms = pivotMotor.getVelocity().getValueAsDouble() * 60;
    inputs.positionPiv = getCurrentPosition();

    inputs.rollerMotorappliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerMotortempCelcius = rollerMotor.getDeviceTemp().getValueAsDouble();
    inputs.rollerMotorCurrent = rollerMotor.getStatorCurrent().getValueAsDouble();
    inputs.rollerCurrentRpms = rollerMotor.getVelocity().getValueAsDouble() * 60;

  }

  @Override
  public void setVoltagePiv(double pivotVolt) {
    pivotMotor.setControl(m_setterControl.withOutput(pivotVolt).withEnableFOC(true));
  }

  @Override
  public void setVoltageRoll(double rollerVolt) {
    rollerMotor.setControl(m_setterControl.withOutput(rollerVolt).withEnableFOC(true));
  }

  @Override
  public void stopMotors() {
    pivotMotor.stopMotor();
    rollerMotor.stopMotor();
  }

  @Override
  public void enableBreak(boolean enable) {
    pivotConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Brake;
    if (pivotConfig.MotorOutput.NeutralMode == pivotConfig.MotorOutput.NeutralMode) {

    } else {
      pivotConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Brake;

      pivotMotor.getConfigurator().apply(pivotConfig);
    }
  }
    
}

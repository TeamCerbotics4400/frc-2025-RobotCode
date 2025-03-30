package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import frc.robot.Constants;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeIOKraken implements IntakeIO {

  /* Hardware */
  private final TalonFX leftMotor = new TalonFX(leftIntakeMotorId, Constants.rioCanbus);
  private final TalonFX rightMotor = new TalonFX(rightIntakeMotorId, Constants.rioCanbus);
  private final LaserCan intakeSensor = new LaserCan(laserCanId);

  /* Configurators */
  private TalonFXConfiguration leftConfig;
  private TalonFXConfiguration rightConfig;

  private final VelocityVoltage leftVelocity = new VelocityVoltage(0);
  private final VelocityVoltage rightVelocity = new VelocityVoltage(0);

  public IntakeIOKraken() {

    leftConfig = new TalonFXConfiguration();
    rightConfig = new TalonFXConfiguration();

    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leftConfig.CurrentLimits.StatorCurrentLimit = 60;
    leftConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    leftConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    leftConfig.Slot0.kP = 0.4;
    leftConfig.Slot0.kD = 0.011;

    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.CurrentLimits.StatorCurrentLimit = 60;
    rightConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    rightConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    rightConfig.Slot0.kP = 0.4;
    rightConfig.Slot0.kD = 0.00501;

    /* Apply Configurations*/
    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);

  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rightMotortempCelcius = rightMotor.getDeviceTemp().getValueAsDouble();
    inputs.rightMotorappliedVolts = rightMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightMotorCurrent = rightMotor.getStatorCurrent().getValueAsDouble();
    inputs.rightCurrentRpms = rightMotor.getVelocity().getValueAsDouble() * 60;

    inputs.leftMotorappliedVolts = leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftMotortempCelcius = leftMotor.getDeviceTemp().getValueAsDouble();
    inputs.leftMotorCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
    inputs.leftCurrentRpms = leftMotor.getVelocity().getValueAsDouble() * 60;

    inputs.sensor = isIntakeFull();
  }

  @Override
  public void setVoltage(double armVolt,double leftVolt) {
    rightMotor.set(armVolt);
    leftMotor.set(leftVolt);
  }

  @Override
  public void setVelocity(double velocityLeft, double velocityRight) {
    leftMotor.setControl(leftVelocity.withVelocity(velocityLeft));
    rightMotor.setControl(rightVelocity.withVelocity(velocityRight));
  }

  @Override
  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public boolean isIntakeFull(){
    LaserCan.Measurement measurement = intakeSensor.getMeasurement();
    if(measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm < 50){
      return true;
    } else {
      return false;
    }
  }
}
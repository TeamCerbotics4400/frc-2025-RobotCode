package frc.robot.Subsystems.Climber;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

import static frc.robot.Constants.ClimberConstants.*;

public class ClimberIOSparkMax implements ClimberIO {

  private final SparkMax climberSparkMax = new SparkMax(CLIMBERSPARKMAXID, MotorType.kBrushless);
  private final SparkMaxConfig climberSparkMaxConfig = new SparkMaxConfig();
  private SparkClosedLoopController climberSparkClosedLoopController;
  private final RelativeEncoder sparkMaxEncoder = climberSparkMax.getEncoder();

  private final TalonFX climberTalonFX = new TalonFX(CLIMBERKRAKENID, Constants.rioCanbus);
  private final TalonFXConfiguration climberSupportConfig = new TalonFXConfiguration();
  private final PositionVoltage climberTalonFXPositionVoltage = new PositionVoltage(0);

  public ClimberIOSparkMax() {

    climberSparkClosedLoopController = climberSparkMax.getClosedLoopController();

    climberSparkMaxConfig.smartCurrentLimit(40);
    climberSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    climberSparkMaxConfig.closedLoop.maxMotion.maxVelocity(6000);
    climberSparkMaxConfig.closedLoop.maxMotion.maxAcceleration(8000);
    climberSparkMaxConfig.closedLoop.pid(0.1, 0, 0);

    climberSparkMax.configure(climberSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    sparkMaxEncoder.setPosition(0);

    /* TALON DOWN HERE -------------------- */
    climberSupportConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climberSupportConfig.CurrentLimits.StatorCurrentLimit = 60;
    climberSupportConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    climberSupportConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    climberSupportConfig.Slot0.kP = 2.5;
    climberTalonFX.setPosition(0);
    climberTalonFX.getConfigurator().apply(climberSupportConfig);
  }

  @Override
  public void setSparkMaxVoltage(double voltage) {
    climberSparkMax.set(voltage);
  }

  @Override
  public void setTalonFXVoltage(double voltage) {
    climberTalonFX.set(voltage);
  }

  @Override
  public void setSparkPosition(double position){
    climberSparkClosedLoopController.setReference(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.1);
  }

  @Override
  public void setTalonFXPosition(double position) {
    climberTalonFX.setControl(climberTalonFXPositionVoltage.withPosition(position));
  }


  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.sparkAppliedVolts = climberSparkMax.getAppliedOutput();
    inputs.sparkTempCelcius = climberSparkMax.getMotorTemperature();
    inputs.sparkPosition = sparkMaxEncoder.getPosition();

    inputs.talonAppliedVolts = climberTalonFX.getMotorVoltage().getValueAsDouble();
    inputs.talonTempCelcius = climberTalonFX.getDeviceTemp().getValueAsDouble();
    inputs.talonPosition = climberTalonFX.getPosition().getValueAsDouble();
  }
    
}
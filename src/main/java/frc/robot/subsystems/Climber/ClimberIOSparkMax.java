package frc.robot.Subsystems.Climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.Constants.ClimberConstants.*;

public class ClimberIOSparkMax implements ClimberIO {

  private final SparkMax climberSparkMax = new SparkMax(CLIMBERSPARKMAXID, MotorType.kBrushless);
  private final SparkMaxConfig climberSparkMaxConfig = new SparkMaxConfig();
  private final SparkClosedLoopController m_pidController = climberSparkMax.getClosedLoopController();
  private final RelativeEncoder sparkMaxEncoder = climberSparkMax.getEncoder();

  public ClimberIOSparkMax() {

    climberSparkMaxConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    climberSparkMaxConfig.smartCurrentLimit(40);

    climberSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).
    p(3).
    i(0).
    d(0);

    climberSparkMaxConfig.idleMode(IdleMode.kBrake);

    
    climberSparkMaxConfig.softLimit.forwardSoftLimitEnabled(true).forwardSoftLimit(-2);

    sparkMaxEncoder.setPosition(0);

    climberSparkMax.configure(climberSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setSparkMaxVoltage(double voltage) {
    climberSparkMax.set(voltage);
  }

  @Override
  public void setSparkPosition(double position){
    m_pidController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.sparkAppliedVolts = climberSparkMax.getAppliedOutput();
    inputs.sparkTempCelcius = climberSparkMax.getMotorTemperature();
    inputs.sparkPosition = sparkMaxEncoder.getPosition();
  }
    
}
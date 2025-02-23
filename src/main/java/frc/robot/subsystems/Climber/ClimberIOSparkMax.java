package frc.robot.Subsystems.Climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.Constants.ClimberConstants.*;

public class ClimberIOSparkMax implements ClimberIO {

  private final SparkMax climberMotor = new SparkMax(CLIMBERSPARKMAXID, MotorType.kBrushless);
  private final SparkMaxConfig climberConfig = new SparkMaxConfig();
  private RelativeEncoder encoder = climberMotor.getEncoder();


  public ClimberIOSparkMax() {
    climberConfig.smartCurrentLimit(40);

    climberConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0)
        .i(0)
        .d(0);

    climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    encoder.setPosition(0);

  }

  @Override
  public void setVoltage(double voltage) {
    climberMotor.set(voltage);
  }

  @Override
  public void setPosition(double position) {
    //climberMotor.setControl(m_positionVoltage.withPosition(position));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVolts = climberMotor.getAppliedOutput();
    inputs.tempCelcius = climberMotor.getMotorTemperature();
    inputs.position = encoder.getPosition();
  }
    
}
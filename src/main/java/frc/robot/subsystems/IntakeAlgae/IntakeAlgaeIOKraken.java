package frc.robot.Subsystems.IntakeAlgae;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import static frc.robot.Constants.IntakeAlgaeConstants.*;

public class IntakeAlgaeIOKraken implements IntakeAlgaeIO {

    private final TalonFX pivotMotor = new TalonFX(pivotAlgaeMotorId, Constants.rioCanbus);
    private final TalonFX rollerMotor = new TalonFX(rollerAlgaeMotorId, Constants.rioCanbus);
    
    private TalonFXConfiguration pivotConfig;
    private TalonFXConfiguration rollerConfig;

      private final Encoder m_encoder;

    public IntakeAlgaeIOKraken(){ 

      m_encoder = new Encoder(7,6);
      m_encoder.reset();
      m_encoder.setDistancePerPulse(1);

    pivotConfig = new TalonFXConfiguration();
    rollerConfig = new TalonFXConfiguration();

    pivotConfig.Feedback.FeedbackRemoteSensorID = 12;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 60;
    pivotConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    pivotConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    pivotConfig.Slot0.kP = 0.0;
    pivotConfig.Slot0.kD = 0.0;

    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = 60;
    rollerConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    rollerConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    rollerConfig.Slot0.kP = 0.0;
    rollerConfig.Slot0.kD = 0.0;

    /* Apply Configurations*/
    pivotMotor.getConfigurator().apply(pivotConfig);
    rollerMotor.getConfigurator().apply(rollerConfig);
    }
    @Override
    public void updateInputs(IntakeAlgaeIOInputs inputs){
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
    public void setVoltagePiv(double pivotVolt){
        pivotMotor.set(pivotVolt);
    }
    @Override
    public void setVoltageRoll(double rollerVolt){
      rollerMotor.set(rollerVolt);
  }

  @Override
  public void stopMotors() {
    pivotMotor.stopMotor();
    rollerMotor.stopMotor();
  }

  @Override
  public void enableBreak(boolean enable) {
    pivotConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Coast : NeutralModeValue.Coast;
    if (pivotConfig.MotorOutput.NeutralMode == pivotConfig.MotorOutput.NeutralMode) {

    } else {
      pivotConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Coast : NeutralModeValue.Coast;

      pivotMotor.getConfigurator().apply(pivotConfig);
    }
  }
 
  public double getCurrentPosition() {
    return -m_encoder.getDistance()*0.2737;
  }
    
}

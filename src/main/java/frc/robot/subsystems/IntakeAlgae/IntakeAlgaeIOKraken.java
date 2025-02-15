package frc.robot.Subsystems.IntakeAlgae;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Velocity;
import frc.robot.Constants;
import frc.robot.Subsystems.Climber.ClimberIO.ClimberIOInputs;

import static frc.robot.Constants.IntakeAlgaeConstants.*;

public class IntakeAlgaeIOKraken implements IntakeAlgaeIO {

    private final TalonFX pivotMotor = new TalonFX(pivotAlgaeMotorId, Constants.rioCanbus);
    private final TalonFX rollerMotor = new TalonFX(rollerAlgaeMotorId, Constants.rioCanbus);
    
    private TalonFXConfiguration pivotConfig;
    private TalonFXConfiguration rollerConfig;


    private final PositionVoltage m_positionVoltage = new PositionVoltage(0);

    public IntakeAlgaeIOKraken(){ 

    pivotConfig = new TalonFXConfiguration();
    rollerConfig = new TalonFXConfiguration();

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
    inputs.positionPiv = pivotMotor.getPosition().getValueAsDouble();

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
  public void setVelocityPiv(double pivotVell) {
    pivotMotor.setControl(m_positionVoltage.withVelocity(pivotVell));
  }
  /*Positions
   *@Override
  public void setPosition(double position) {
    pivotMotor.setControl(m_positionVoltage.withPosition(position));
  }
   * }
   */
  @Override
  public void stopMotors() {
    pivotMotor.stopMotor();
    rollerMotor.stopMotor();
  }
 

    
}

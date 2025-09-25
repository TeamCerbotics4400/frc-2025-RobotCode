package frc.robot.Subsystems.Climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class ClimberIOkraken {

    private final TalonFX climberFx;
    private final TalonFXConfiguration climberTalonFXConfiguration;

    private final SparkMax motorClimber = new SparkMax(Constants.ClimberConstants.MotorClimberID, MotorType.kBrushless);
    private final SparkMaxConfig motorClimberConfig = new SparkMaxConfig();

    public ClimberIOkraken() {

        motorClimberConfig.smartCurrentLimit(40);

        climberFx = new TalonFX(Constants.ClimberConstants.CLIMBERKRAKENID, Constants.rioCanbus);

        climberTalonFXConfiguration = new TalonFXConfiguration();
        climberTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        climberTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;

        climberFx.setPosition(0);
        climberFx.getConfigurator().apply(climberTalonFXConfiguration);

        motorClimber.configure(motorClimberConfig, null, null);

    }

    public void setTalonFXVoltage(double voltage) {
        climberFx.set(voltage);
    }

    public void setMotorClimberVoltage(double voltage) {
        motorClimber.set(voltage);
    }

}

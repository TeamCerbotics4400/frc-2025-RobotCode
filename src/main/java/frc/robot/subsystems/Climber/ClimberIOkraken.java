package frc.robot.Subsystems.Climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class ClimberIOkraken implements ClimberIO {

    private final TalonFX climberFx;
    private final TalonFXConfiguration climberTalonFXConfiguration;

    private final SparkMax cageMotor = new SparkMax(Constants.ClimberConstants.cageMotorID, MotorType.kBrushless);
    private final SparkMaxConfig cageMotorConfig = new SparkMaxConfig();

    public ClimberIOkraken() {

        cageMotorConfig.smartCurrentLimit(40);

        climberFx = new TalonFX(Constants.ClimberConstants.CLIMBERKRAKENID, Constants.rioCanbus);

        climberTalonFXConfiguration = new TalonFXConfiguration();
        climberTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        climberTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        climberTalonFXConfiguration.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        climberFx.setPosition(0);

        // climberFx.setPosition(0);
        climberFx.getConfigurator().apply(climberTalonFXConfiguration);

        cageMotor.configure(cageMotorConfig, null, null);

    }

    public void setTalonFXVoltage(double voltage) {
        climberFx.set(voltage);
    }

    public void setCageMotorVolatge(double voltage) {
        cageMotor.set(voltage);
    }

    public double getClimberPosition(){
        double  pos = (climberFx.getPosition().getValueAsDouble());
         return pos;

    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.sparkAppliedVolts = cageMotor.getAppliedOutput();
        inputs.climberFxPosition = getClimberPosition();
    

    }

}

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import edu.wpi.first.units.measure.Angle;
import frc.robot.HardwareID;

public class IntakeConstants implements HardwareID.IntakeHardwareID {

    public static final double INTAKE_SPIN_DUTY_CYCLE = 0.75;

    public static final TalonFXConfiguration DEPLOY_MOTOR_CONFIG = new TalonFXConfiguration()
            .withCurrentLimits(
                    new CurrentLimitsConfigs()
                            .withStatorCurrentLimit(40)
                            .withStatorCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(40)
                            .withSupplyCurrentLimitEnable(true))
            .withSoftwareLimitSwitch(
                    new SoftwareLimitSwitchConfigs()
                            .withForwardSoftLimitEnable(true)
                            .withReverseSoftLimitEnable(true)
                            .withForwardSoftLimitThreshold(-140.0)
                            .withReverseSoftLimitThreshold(20.0));

    public static final double kP_DEPLOY = 1.5;
    public static final Slot0Configs DEPLOY_MOTOR_GAINS = new Slot0Configs().withKP(kP_DEPLOY);

    public static final TalonFXConfiguration SPIN_MOTOR_CONFIG = new TalonFXConfiguration().withCurrentLimits(
            new CurrentLimitsConfigs().withStatorCurrentLimit(50.0).withStatorCurrentLimitEnable(true));

    public static final Angle FULLY_RETRACTED_ANGLE = Degrees.of(0.0);
    public static final Angle FULLY_DEPLOYED_ANGLE = Degrees.of(-120);

    public static final PositionDutyCycle DEPLOY_POSITION_REQUEST = new PositionDutyCycle(Degrees.of(0.0));
    public static final DutyCycleOut DEPLOY_DUTY_CYCLE_REQUEST = new DutyCycleOut(0.0);
}

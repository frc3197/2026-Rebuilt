package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.HardwareID;

public class IntakeConstants implements HardwareID.IntakeHardwareID {
    public static final class intakeNumericalConstants {
        public static final double forwardLimit = -1.0; // Please find a reasonable value for me (Or switch me to the angle version, figure it out on friday)
        public static final double reverseLimit = -1.0; // Please find a reasonable value for me (Or switch me to the angle version, figure it out on friday)

    }
    public static final class motorConfigurationConstants {
        public static final TalonFXConfiguration spinMotorConfig = new TalonFXConfiguration()
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(intakeNumericalConstants.forwardLimit)
            .withReverseSoftLimitThreshold(intakeNumericalConstants.reverseLimit)
        );
    }
}

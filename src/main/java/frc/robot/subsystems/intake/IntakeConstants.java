package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.HardwareID;

public class IntakeConstants implements HardwareID.IntakeHardwareID {

  public static final double INTAKE_SPIN_DUTY_CYCLE = 0.75;

  // Motion magic configs
  private static final double max_intake_rps = DegreesPerSecond.of(135).in(RotationsPerSecond);
  private static final double max_intake_acceleration =
      DegreesPerSecondPerSecond.of(400).in(RotationsPerSecondPerSecond);
  private static final MotionMagicConfigs INTAKE_MM_CONFIGS =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(max_intake_rps)
          .withMotionMagicAcceleration(max_intake_acceleration);
  public static final MotionMagicTorqueCurrentFOC INTAKE_MOTION_MAGIC_REQUEST =
      new MotionMagicTorqueCurrentFOC(Degrees.of(0.0));

  private static final double kP_DEPLOY = 100.5;
  private static final Slot0Configs DEPLOY_MOTOR_GAINS = new Slot0Configs().withKP(kP_DEPLOY);

  public static final TalonFXConfiguration DEPLOY_MOTOR_CONFIG =
      new TalonFXConfiguration()
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
                  .withForwardSoftLimitThreshold(150.0)
                  .withReverseSoftLimitThreshold(-20.0))
          .withMotionMagic(INTAKE_MM_CONFIGS)
          .withSlot0(DEPLOY_MOTOR_GAINS)
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

  public static final TalonFXConfiguration SPIN_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(90.0)
                  .withStatorCurrentLimitEnable(true))
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

  public static final Angle FULLY_RETRACTED_ANGLE = Degrees.of(0.0);
  public static final Angle FULLY_DEPLOYED_ANGLE = Degrees.of(103);

  public static final PositionDutyCycle DEPLOY_POSITION_REQUEST =
      new PositionDutyCycle(Degrees.of(0.0));
  public static final DutyCycleOut DEPLOY_DUTY_CYCLE_REQUEST = new DutyCycleOut(0.0);
}

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.HardwareID;

public class ShooterConstants implements HardwareID.ShooterHardwareID {

  public static final double FX_TO_TURRET_RATIO = (1 * 12 * (100 / 20));

  public static final PIDController FLYWHEEL_PID_CONTROLLER = new PIDController(0.0, 0.0, 0.0);

  public static final Voltage MAX_TURRET_ROTATION_MOTOR_VOLTS = Volts.of(12.0);

  // Describes the turret's bottom opening relative to robot position
  public static final Transform3d ROBOT_TO_TURRET_CENTER =
      new Transform3d(0.2, 0.2, 0.5, Rotation3d.kZero);

  // Turret rotation PID controller
  public static final PIDController TURRET_ANGLE_PID_CONTROLLER = new PIDController(0.5, 0.0, 0.0);

  public static final TalonFXConfiguration FLYWHEEL_TALON_FX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(100)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(100.0)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.CounterClockwise_Positive));

  private static final double kSFlywheel = 0.25; // Add 0.25 V output to overcome static friction
  private static final double kVFlywheel =
      0.12; // A velocity target of 1 rps results in 0.12 V output
  private static final double kPFlywheel =
      4.8; // A position error of 2.5 rotations results in 12 V output
  private static final double kIFlywheel = 0; // no output for integrated error
  private static final double kDFlywheel = 0.1; // A velocity error of 1 rps results in 0.1 V output
  public static Slot0Configs FLYWHEEL_SLOT0_CONFIGS =
      new Slot0Configs()
          .withKS(kSFlywheel)
          .withKV(kVFlywheel)
          .withKP(kPFlywheel)
          .withKI(kIFlywheel)
          .withKD(kDFlywheel);

  public static FeedbackConfigs TURRET_FEEDBACK_CONFIGS =
      new FeedbackConfigs().withSensorToMechanismRatio(FX_TO_TURRET_RATIO);

  public static TalonFXConfiguration TURRET_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withFeedback(TURRET_FEEDBACK_CONFIGS);

  public static final Angle TURRET_ROTATION_LIMIT_FORWARD = Degrees.of(180);
  public static final Angle TURRET_ROTATION_LIMIT_REVERSE = Degrees.of(-180);
}

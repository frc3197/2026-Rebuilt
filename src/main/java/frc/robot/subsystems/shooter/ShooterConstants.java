package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.HardwareID;

public class ShooterConstants implements HardwareID.ShooterHardwareID {

  // Physical
  public static final Distance FUEL_RELEASE_HEIGHT = Inches.of(20);

  public static final double MAX_SPEED_WHILE_SHOOTING_MPS = 1.0;
  public static final double MAX_OMEGA_WHILE_SHOOTING_RADPS = 2.0;

  // Thresholds
  public static final AngularVelocity FRENZY_FEED_THRESHOLD = RotationsPerSecond.of(6.0);
  public static final AngularVelocity NORMAL_FEED_THRESHOLD = RotationsPerSecond.of(4.25);

  public static final LinearVelocity TRANSLATIONAL_SPEED_THRESHOLD = MetersPerSecond.of(2.25);
  public static final AngularVelocity ANGULAR_SPEED_THRESHOLD = DegreesPerSecond.of(360);
  public static final Distance HOOD_EXTENSION_THRESHOLD = Millimeters.of(15.0);
  public static final Angle TURRET_ANGLE_ERROR_THRESHOLD = Degrees.of(25.0);

  // Lookahead & compensation
  public static final double TURRET_ROTATION_LOOKAHEAD_CONSTANT = 0.14;
  public static final double TURRET_OMEGA_COMPENSATION_CONSTANT = 1.7;
  public static final double VELOCITY_COMPENSATION_CONSTANT = 1.24;
  public static final double ACCELERATION_COMPENSATION_CONSTANT = 0.0;

  private static final Distance ROBOT_TO_TURRET_X = Inches.of(18.25 - 13);
  private static final Distance ROBOT_TO_TURRET_Y = Inches.of(13 - 7.5);
  public static Vector<N3> ROBOT_TO_TURRET_VECTOR =
      VecBuilder.fill(-ROBOT_TO_TURRET_Y.in(Meters), -ROBOT_TO_TURRET_X.in(Meters), 0);
  // Describes the turret's bottom opening relative to robot position
  public static final Transform3d ROBOT_TO_TURRET_CENTER =
      new Transform3d(
          -ROBOT_TO_TURRET_Y.in(Meters),
          -ROBOT_TO_TURRET_X.in(Meters),
          0.5,
          new Rotation3d(0, 0, Degrees.of(180).in(Radians)));

  // TURRET ----------------------------------------------------------------------
  public static final double FX_TO_TURRET_RATIO = (4 * (105 / 25));
  public static final Voltage MAX_TURRET_ROTATION_MOTOR_VOLTS = Volts.of(12.0);

  // Turret rotation PID controller
  private static final double kSTurret = 0.0253;
  private static final double kVTurret = 0.00;
  private static final double kPTurret =
      3800; // A position error of 2.5 rotations results in 12 V output
  private static final double kITurret = 0;
  private static final double kDTurret = 100;
  public static Slot0Configs TURRET_SLOT0_CONFIGS =
      new Slot0Configs()
          .withKS(kSTurret)
          .withKV(kVTurret)
          .withKP(kPTurret)
          .withKI(kITurret)
          .withKD(kDTurret);

  // Motion magic configs
  private static final double max_turret_rps = RadiansPerSecond.of(30).in(RotationsPerSecond);
  private static final double max_turret_acceleration =
      RadiansPerSecondPerSecond.of(30).in(RotationsPerSecondPerSecond);
  private static final MotionMagicConfigs TURRET_MM_CONFIGS =
      new MotionMagicConfigs()
          .withMotionMagicAcceleration(max_turret_acceleration)
          .withMotionMagicCruiseVelocity(max_turret_rps);
  public static final MotionMagicTorqueCurrentFOC TURRET_MOTION_MAGIC_REQUEST =
      new MotionMagicTorqueCurrentFOC(Degrees.of(0.0));

  public static final VoltageOut TURRET_VOLTAGE_REQUEST = new VoltageOut(0.0);
  public static final PositionDutyCycle TURRET_POSITION_REQUEST = new PositionDutyCycle(0.0);

  public static final Angle TURRET_SPRING_ANGLE_ZERO = Radians.of(-0.917321);
  public static final double TURRET_SPRING_FF = -0.05;

  // Turret rotation limits
  public static final Angle TURRET_ROTATION_LIMIT_FORWARD = Degrees.of(180);
  public static final Angle TURRET_ROTATION_LIMIT_REVERSE = Degrees.of(-180);

  // Set the turret ratio
  public static FeedbackConfigs TURRET_FEEDBACK_CONFIGS =
      new FeedbackConfigs().withSensorToMechanismRatio(FX_TO_TURRET_RATIO);

  public static final int TURRET_ZERO_LIMIT = 0;

  // Turret motor configs
  public static TalonFXConfiguration TURRET_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
          .withFeedback(TURRET_FEEDBACK_CONFIGS)
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(65.0)
                  .withStatorCurrentLimitEnable(true))
          .withSoftwareLimitSwitch(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitEnable(true)
                  .withForwardSoftLimitThreshold(TURRET_ROTATION_LIMIT_FORWARD)
                  .withReverseSoftLimitEnable(true)
                  .withReverseSoftLimitThreshold(TURRET_ROTATION_LIMIT_REVERSE))
          .withSlot0(TURRET_SLOT0_CONFIGS)
          .withMotionMagic(TURRET_MM_CONFIGS);

  // FLYWHEEL --------------------------------------------------------------------

  private static final double kSFlywheel = 12.5; // Add 0.25 V output to overcome static friction
  private static final double kVFlywheel =
      0.364; // A velocity target of 1 rps results in 0.12 V output
  private static final double kPFlywheel =
      12.5; // A position error of 2.5 rotations results in 12 V output
  private static final double kIFlywheel = 0; // no output for integrated error
  private static final double kDFlywheel = 0.0;
  private static final double kAFlywheel = 1.5; // A velocity error of 1 rps results in 0.1 V output
  public static Slot0Configs FLYWHEEL_SLOT0_CONFIGS =
      new Slot0Configs()
          .withKS(kSFlywheel)
          .withKV(kVFlywheel)
          .withKP(kPFlywheel)
          .withKI(kIFlywheel)
          .withKA(kAFlywheel)
          .withKD(kDFlywheel);

  public static Slot1Configs FLYWHEEL_SLOT1_CONFIGS =
      new Slot1Configs()
          .withKS(kSFlywheel)
          .withKV(kVFlywheel)
          .withKP(15)
          .withKI(kIFlywheel)
          .withKA(kAFlywheel)
          .withKD(kDFlywheel);

  public static Slot2Configs FLYWHEEL_SLOT2_CONFIGS =
      new Slot2Configs()
          .withKS(kSFlywheel)
          .withKV(kVFlywheel)
          .withKP(75)
          .withKI(kIFlywheel)
          .withKA(kAFlywheel)
          .withKD(kDFlywheel);

  public static final VelocityTorqueCurrentFOC FLYWHEEL_TORQUE_REQUEST =
      new VelocityTorqueCurrentFOC(RotationsPerSecond.of(0.0)).withSlot(0);

  public static final VelocityTorqueCurrentFOC QUICK_RECOVER =
      new VelocityTorqueCurrentFOC(RotationsPerSecond.of(0.0)).withSlot(1);

  public static final VelocityTorqueCurrentFOC FLYWHEEL_BANG_BANG_REQUEST =
      new VelocityTorqueCurrentFOC(RotationsPerSecond.of(0.0)).withSlot(2);

  public static final VoltageOut FLYWHEEL_VOLTAGE_REQUEST = new VoltageOut(Volts.of(0.0));

  // Manual preset flywheel speed
  public static final Voltage FLYWHEEL_VOLTAGE_SHORT_SHOT_POPCORN = Volts.of(3.50);

  public static final TalonFXConfiguration FLYWHEEL_TALON_FX_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(130.0)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(60.0)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withSlot0(FLYWHEEL_SLOT0_CONFIGS)
          .withSlot1(FLYWHEEL_SLOT1_CONFIGS)
          .withSlot2(FLYWHEEL_SLOT2_CONFIGS);

  // HOOD --------------
  public static Angle MAX_HOOD_ANGLE = Degrees.of(45);
  public static Angle MIN_HOOD_ANGLE = Degrees.of(20);
}

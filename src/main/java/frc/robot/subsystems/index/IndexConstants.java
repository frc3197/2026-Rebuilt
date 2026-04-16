package frc.robot.subsystems.index;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.HardwareID.IndexHardwareID;

public class IndexConstants implements IndexHardwareID {

  /*
   * public static final TalonFXSConfiguration SPINDEX_CONFIG =
   * new TalonFXSConfiguration()
   * .withCommutation(
   * new
   * CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST))
   * .withMotorOutput(new
   * MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
   */
  public static final TalonFXConfiguration SPINDEX_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

  public static final VelocityDutyCycle FEED_TORQUE_REQUEST =
      new VelocityDutyCycle(RotationsPerSecond.of(0.0)).withEnableFOC(true);

  private static final double kSFeed = 0.01; // Add 0.25 V output to overcome static friction
  private static final double kVFeed =
      0.0084; // A velocity target of 1 rps results in 0.12 V output
  private static final double kPFeed =
      0.03; // A position error of 2.5 rotations results in 12 V output
  private static final double kIFeed = 0; // no output for integrated error
  private static final double kDFeed = 0.0;
  private static final double kAFeed = 0.0; // A velocity error of 1 rps results in 0.1 V output
  public static Slot0Configs Feed_SLOT0_CONFIGS =
      new Slot0Configs()
          .withKS(kSFeed)
          .withKV(kVFeed)
          .withKP(kPFeed)
          .withKI(kIFeed)
          .withKA(kAFeed)
          .withKA(kAFeed)
          .withKD(kDFeed);

  public static final TalonFXSConfiguration FEED_CONFIG =
      new TalonFXSConfiguration()
          .withCommutation(
              new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST))
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withSlot0(Feed_SLOT0_CONFIGS)
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(100)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(55)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive));

  /*
   * public static final TalonFXConfiguration FEED_CONFIG =
   * new TalonFXConfiguration()
   * .withCurrentLimits(
   * new CurrentLimitsConfigs()
   * .withStatorCurrentLimit(120)
   * .withStatorCurrentLimitEnable(true)
   * .withSupplyCurrentLimit(120)
   * .withSupplyCurrentLimitEnable(true));
   */

  // VOLTAGES
  public static final Voltage SPINDEX_SHOOTING_VOLTAGE = Volts.of(8.0);
  public static final Voltage SPINDEX_BACKFEED_VOLTAGE = Volts.of(-7.0);
  public static final Voltage FEEDER_SHOOTING_VOLTAGE = Volts.of(4.5);
  public static final Voltage FEEDER_BACKFEED_VOLTAGE = Volts.of(-4.5);

  public static final AngularVelocity FEEDER_SHOOTING_RPS = RotationsPerSecond.of(150);
}

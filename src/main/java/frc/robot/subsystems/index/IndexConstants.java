package frc.robot.subsystems.index;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.HardwareID.IndexHardwareID;

public class IndexConstants implements IndexHardwareID {

  public static final TalonFXSConfiguration SPINDEX_CONFIG =
      new TalonFXSConfiguration()
          .withCommutation(
              new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST))
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

  public static final TalonFXSConfiguration FEED_CONFIG =
      new TalonFXSConfiguration()
          .withCommutation(
              new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST))
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

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
  public static final Voltage SPINDEX_SHOOTING_VOLTAGE = Volts.of(1.65);
  public static final Voltage FEEDER_SHOOTING_VOLTAGE = Volts.of(12.0);
}

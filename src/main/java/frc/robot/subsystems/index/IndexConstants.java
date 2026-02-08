package frc.robot.subsystems.index;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import frc.robot.HardwareID.IndexHardwareID;

public class IndexConstants implements IndexHardwareID {

  public static final TalonFXSConfiguration SPINDEX_CONFIG =
      new TalonFXSConfiguration()
          .withCommutation(
              new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST));

  public static final TalonFXConfiguration FEED_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(120)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(120)
                  .withSupplyCurrentLimitEnable(true));
}

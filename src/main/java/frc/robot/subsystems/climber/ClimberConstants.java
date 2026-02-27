// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.HardwareID;

/** Add your docs here. */
public class ClimberConstants implements HardwareID.ClimberHardwareID {

  public static final FeedbackConfigs climberFeedback =
      new FeedbackConfigs().withSensorToMechanismRatio(5 * 5 * 4 * (30 / 10));

  public static final TalonFXConfiguration climberTalonConfig =
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKP(12.0))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(160)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(160.0)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withFeedback(climberFeedback);

  public static final Angle climberStowAngle = Degrees.of(0.0);
  public static final Angle climberUpAngle = Degrees.of(127.5);
  public static final Angle climberClimbAngle = Degrees.of(70.0);
}

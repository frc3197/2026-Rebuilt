// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.HardwareID;

/** Add your docs here. */
public class ClimberConstants implements HardwareID.ClimberHardwareID {

  public static final TalonFXConfiguration climberTalonConfig =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(80)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(0.0)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.CounterClockwise_Positive));

  public static final PIDController controller = new PIDController(5.0, 0.0, 0.0);
}

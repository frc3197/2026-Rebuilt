// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.index;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IndexIO {
  @AutoLog
  public static class IndexInputs {
    public double indexMotorSuppliedVoltage = 0.0;
    public double feedRPS = 0.0;
    public double spindexDrawAmps = 0.0;
    public double feedDrawAmps = 0.0;
  }

  public default void updateInputs(IndexInputs inputs) {}

  public default void setFeedMotorVoltage(Voltage volts) {}

  public default void setSpindexMotorVoltage(Voltage volts) {}

  public default void setGains(Slot0Configs gains) {}

  public default void setFeedMotorRequest(ControlRequest request) {}
}

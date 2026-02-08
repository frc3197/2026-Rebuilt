// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.index;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IndexIO {
  @AutoLog
  public static class IndexInputs {
    public double indexMotorSuppliedVoltage = 0.0;
  }

  public default void updateInputs(IndexInputs inputs) {}

  public default void setFeedMotorVoltage(Voltage volts) {}

  public default void setSpindexMotorVoltage(Voltage volts) {}
}

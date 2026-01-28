// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {

  @AutoLog
  public static class ClimberInputs {
    public double leftMotorCurrent = 0;
    public double rightMotorCurrent = 0;
  }

  public default void updateInputs(ClimberInputs inputs) {}
  ;

  public default void setClimbMotorSpeed(double speed) {}
  ;
}

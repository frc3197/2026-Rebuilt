// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {

  @AutoLog
  public static class ClimberInputs {
    public double motorCurrent = 0;
    public double climberAngleDegrees = 0.0;
  }

  public default void updateInputs(ClimberInputs inputs) {}

  public default void setClimbMotorSpeed(double speed) {}

  public default void setTarget(Distance target) {}

  public default void zeroClimber() {}

  public default void setClimberControl(ControlRequest request) {}

  public default Angle getClimberAngle() {
    return Degrees.of(0.0);
  }
}

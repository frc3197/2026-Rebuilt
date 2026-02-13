// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ClimberIOSim implements ClimberIO {

  private double motorSpeed = 0.0;

  private Distance target = Inches.of(0);

  public ClimberIOSim() {
    // climberSim.setState(0, 8.0);
  }

  @Override
  public void setTarget(Distance target) {
    System.out.println(target.in(Meters));
    this.target = target;
  }

  private void updateVoltageSetpoint() {}

  private void runVolts(Voltage volts) {}

  @Override
  public void updateInputs(ClimberInputs inputs) {

    updateVoltageSetpoint();
  }

  @Override
  public void setClimbMotorSpeed(double speed) {
    motorSpeed = speed;
  }
}

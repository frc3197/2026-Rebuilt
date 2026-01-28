// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.RobotState;

/** Add your docs here. */
public class ClimberIOSim implements ClimberIO {

  private double leftMotorSpeed = 0.0;
  private double rightMotorSpeed = 0.0;

  private final ElevatorSim climberSim =
      new ElevatorSim(
          LinearSystemId.createElevatorSystem(
              DCMotor.getKrakenX60Foc(2),
              Pounds.of(45).in(Kilograms),
              Inches.of(0.5).in(Meters),
              4.0 / 1.0),
          DCMotor.getKrakenX60Foc(2),
          Inches.of(0).in(Meters),
          Inches.of(32).in(Meters),
          true,
          Inches.of(0).in(Meters));

  @Override
  public void updateInputs(ClimberInputs inputs) {
    inputs.leftMotorCurrent = climberSim.getCurrentDrawAmps();
    inputs.rightMotorCurrent = climberSim.getCurrentDrawAmps();

    RobotState.instance().setClimberHeight(Meters.of(climberSim.getPositionMeters()));
  }

  @Override
  public void setClimbMotorSpeed(double speed) {

    leftMotorSpeed = speed;
    rightMotorSpeed = speed;

    climberSim.setInputVoltage(speed);
  }
}

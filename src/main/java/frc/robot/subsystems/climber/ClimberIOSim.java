// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.managersubsystems.RobotState;

/** Add your docs here. */
public class ClimberIOSim implements ClimberIO {

  private final ProfiledPIDController controller =
      new ProfiledPIDController(5.0, 0.0, 0.0, new Constraints(90, 120));

  private double leftMotorSpeed = 0.0;
  private double rightMotorSpeed = 0.0;

  private Distance target = Inches.of(0);
  private MutVoltage appliedVoltage = Volts.mutable(0.0);

  private final ElevatorSim climberSim =
      new ElevatorSim(
          LinearSystemId.createElevatorSystem(
              DCMotor.getKrakenX60Foc(2),
              Pounds.of(45).in(Kilograms),
              Inches.of(0.5).in(Meters),
              4.0 / 1.0),
          DCMotor.getKrakenX60Foc(1),
          Inches.of(0).in(Meters),
          Inches.of(32).in(Meters),
          false,
          Inches.of(0).in(Meters));

  public ClimberIOSim() {
    // climberSim.setState(0, 8.0);
  }

  @Override
  public void setTarget(Distance target) {
    System.out.println(target.in(Meters));
    this.target = target;
    controller.setGoal(target.in(Inches));
  }

  private void updateVoltageSetpoint() {
    Distance currentPosition = Meters.of(climberSim.getPositionMeters());
    LinearVelocity currentVelocity = MetersPerSecond.of(climberSim.getVelocityMetersPerSecond());
    Voltage controllerVoltage =
        Volts.of(controller.calculate(currentPosition.in(Inches), this.target.in(Inches)));

    runVolts(controllerVoltage);
  }

  private void runVolts(Voltage volts) {
    double clampedEffort = MathUtil.clamp(volts.in(Volts), -12, 12);
    appliedVoltage.mut_replace(clampedEffort, Volts);
    climberSim.setInputVoltage(clampedEffort);
  }

  @Override
  public void updateInputs(ClimberInputs inputs) {
    inputs.leftMotorCurrent = climberSim.getCurrentDrawAmps();
    inputs.rightMotorCurrent = climberSim.getCurrentDrawAmps();

    RobotState.instance().setClimberHeight(Meters.of(climberSim.getPositionMeters()));

    SmartDashboard.putNumber("CL TEST", climberSim.getPositionMeters());

    updateVoltageSetpoint();
  }

  @Override
  public void setClimbMotorSpeed(double speed) {

    System.out.println("SETTING CLIMBER: " + climberSim.getVelocityMetersPerSecond());

    leftMotorSpeed = speed;
    rightMotorSpeed = speed;

    climberSim.setInputVoltage(speed);
  }
}

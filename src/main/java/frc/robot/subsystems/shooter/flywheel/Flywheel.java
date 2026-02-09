// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

  private final FlywheelIO flywheelIO;
  private final FlywheelInputsAutoLogged loggedInputs = new FlywheelInputsAutoLogged();

  public Flywheel(FlywheelIO flywheelIO) {
    this.flywheelIO = flywheelIO;
  }

  @Override
  public void periodic() {
    flywheelIO.updateInputs(loggedInputs);

    Logger.processInputs("Shooter/Flywheel", loggedInputs);
  }

  /*
   * public Command setFlywheelVoltage(Voltage volts) {
   * return Commands.runOnce(() -> flywheelIO.setFlywheelMotorVolts(volts), this);
   * }
   */

  public void setFlywheelVoltage(Voltage volts) {
    flywheelIO.setFlywheelMotorVolts(volts);
  }
}

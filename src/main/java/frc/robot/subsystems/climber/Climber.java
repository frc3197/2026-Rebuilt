// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final ClimberIO climberIO;

  /** Creates a new Climber. */
  public Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command setClimbSpeed(double speed) {
    return Commands.runOnce(
        () -> {
          climberIO.setClimbMotorSpeed(speed);
        },
        this);
  }
}

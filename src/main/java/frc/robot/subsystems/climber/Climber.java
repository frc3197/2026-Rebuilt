// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final ClimberIO climberIO;
  private ClimberInputsAutoLogged loggedClimber = new ClimberInputsAutoLogged();

  /** Creates a new Climber. */
  public Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(loggedClimber);
  }

  public Command setClimbSpeed(double speed) {
    return Commands.runOnce(
        () -> {
          this.climberIO.setClimbMotorSpeed(speed);
        },
        this);
  }

  public Command setClimbPostion(Distance position) {
    return Commands.runOnce(
        () -> {
          this.climberIO.setTarget(position);
        },
        this);
  }
  ;
}

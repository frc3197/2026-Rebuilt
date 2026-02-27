// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

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

    Logger.processInputs("Climber", loggedClimber);
  }

  public Command setClimbSpeed(double speed) {
    return Commands.runOnce(
        () -> {
          this.climberIO.setClimbMotorSpeed(speed);
        },
        this);
  }

  public void zeroClimber() {
    climberIO.zeroClimber();
  }

  public Command zeroClimberCommand() {
    return Commands.runOnce(() -> climberIO.zeroClimber(), this);
  }

  public void setClimberControlType(ControlRequest request) {
    climberIO.setClimberControl(request);
  }
}

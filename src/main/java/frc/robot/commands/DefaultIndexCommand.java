// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.managersubsystems.RobotState;
import frc.robot.subsystems.index.Index;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultIndexCommand extends Command {

  private final Index index;

  /**
   * Creates a new DefaultFlywheelCommand.
   *
   * @param intake The intake subsystem.
   */
  public DefaultIndexCommand(Index index) {
    this.index = index;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    switch (RobotState.instance().getFlywheelMode()) {
      case IDLE:
        index.setFeedMotor(Volts.of(0.0));
        index.setSpindexMotor(Volts.of(0.0));
        break;
      default:
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {}
}

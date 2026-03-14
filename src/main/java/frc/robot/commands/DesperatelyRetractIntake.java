// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.Modes.IntakeDeployMode;
import frc.robot.managersubsystems.RobotState;

// Creates a desperate attempt to retract intake
public class DesperatelyRetractIntake extends Command {
  /** Creates a new DesperatelyRetractIntake. */
  Timer timer = new Timer();

  public DesperatelyRetractIntake() {
    timer.start();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() % 2 <= 0.5) {
      RobotState.instance().setIntakeDeployMode(IntakeDeployMode.DEPLOYING);
    } else {
      RobotState.instance().setIntakeDeployMode(IntakeDeployMode.RETRACTING);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotState.instance().setIntakeDeployMode(IntakeDeployMode.RETRACTING);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

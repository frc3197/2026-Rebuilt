// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.enums.Modes.IntakeDeployMode;
import frc.robot.managersubsystems.RobotState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultIntakeCommand extends Command {
  /** Creates a new DefaultIntakeCommand. */
  private final Intake intake;

  private final BooleanSupplier spinManual;
  private final DoubleSupplier deployManual;

  /**
   * Creates a new DefaultFlywheelCommand.
   *
   * @param intake The intake subsystem.
   * @param spinManual Determines if manual intake spin button is pressed.
   * @param deployManual Determines if manual intake deploy button is pressed.
   */
  public DefaultIntakeCommand(
      Intake intake, BooleanSupplier spinManual, DoubleSupplier deployManual) {
    this.spinManual = spinManual;
    this.deployManual = deployManual;

    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // First update spinning logic
    intakeSpinLogic();

    // Then update deploy logic
    intakeDeployLogic();
  }

  private void intakeSpinLogic() {
    switch (RobotState.instance().getIntakeSpinMode()) {
      case IDLE:
        intake.setIntakeSpinSpeed(0.0);
        break;

      case INTAKING:
        intake.setIntakeSpinSpeed(IntakeConstants.INTAKE_SPIN_DUTY_CYCLE);
        break;

      case OUTTAKING:
        intake.setIntakeSpinSpeed(IntakeConstants.INTAKE_SPIN_DUTY_CYCLE);
        break;

      case MANUAL:
        intake.setIntakeSpinSpeed(
            spinManual.getAsBoolean() ? IntakeConstants.INTAKE_SPIN_DUTY_CYCLE : 0.0);
        break;

      default:
        DriverStation.reportError(
            "Invalid intake spin mode: " + RobotState.instance().getIntakeSpinMode(), false);
        break;
    }
  }

  private void intakeDeployLogic() {

    IntakeDeployMode mode = RobotState.instance().getIntakeDeployMode();

    if (mode != IntakeDeployMode.MANUAL
        && MathUtil.isNear(
            IntakeConstants.FULLY_DEPLOYED_ANGLE.in(Degrees),
            intake.getDeployAngle().in(Degrees),
            Degrees.of(5.0).in(Degrees))) {
      RobotContainer.setIntakeDeployMode(IntakeDeployMode.IDLE_DEPLOYED);
    }

    if (mode != IntakeDeployMode.MANUAL
        && MathUtil.isNear(
            IntakeConstants.FULLY_RETRACTED_ANGLE.in(Degrees),
            intake.getDeployAngle().in(Degrees),
            Degrees.of(5.0).in(Degrees))) {
      RobotContainer.setIntakeDeployMode(IntakeDeployMode.IDLE_RETRACTED);
    }

    switch (RobotState.instance().getIntakeDeployMode()) {
      case DEPLOYING:
        intake.setDeployControlRequest(
            IntakeConstants.INTAKE_MOTION_MAGIC_REQUEST.withPosition(
                IntakeConstants.FULLY_DEPLOYED_ANGLE));
        break;

      case RETRACTING:
        intake.setDeployControlRequest(
            IntakeConstants.INTAKE_MOTION_MAGIC_REQUEST.withPosition(
                IntakeConstants.FULLY_RETRACTED_ANGLE));
        break;

      case IDLE_DEPLOYED:
        intake.setDeployControlRequest(IntakeConstants.DEPLOY_DUTY_CYCLE_REQUEST.withOutput(0.0));
        break;

      case IDLE_RETRACTED:
        intake.setDeployControlRequest(IntakeConstants.DEPLOY_DUTY_CYCLE_REQUEST.withOutput(0.0));
        break;

      case MANUAL:
        intake.setDeployControlRequest(
            IntakeConstants.DEPLOY_DUTY_CYCLE_REQUEST.withOutput(deployManual.getAsDouble()));
        break;

      default:
        DriverStation.reportError(
            "Invalid intake deploy mode: " + RobotState.instance().getIntakeDeployMode(), false);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
}

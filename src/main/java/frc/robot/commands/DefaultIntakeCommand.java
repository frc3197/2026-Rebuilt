// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LoggingConstants;
import frc.robot.enums.Modes.FlywheelMode;
import frc.robot.enums.Modes.IntakeDeployMode;
import frc.robot.managersubsystems.RobotState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultIntakeCommand extends Command {
  /** Creates a new DefaultIntakeCommand. */
  private final Intake intake;

  private final BooleanSupplier spinManual;
  private final BooleanSupplier backfeedManual;
  private final DoubleSupplier deployManual;

  private Timer flopTImer = new Timer();

  private final LoggedTunableNumber deployKs =
      new LoggedTunableNumber("Intake deploy ks", IntakeConstants.DEPLOY_MOTOR_GAINS.kS);
  private final LoggedTunableNumber deployKp =
      new LoggedTunableNumber("Intake deploy kp", IntakeConstants.DEPLOY_MOTOR_GAINS.kP);
  private final LoggedTunableNumber deployKv =
      new LoggedTunableNumber("Intake deploy kv", IntakeConstants.DEPLOY_MOTOR_GAINS.kV);
  private final LoggedTunableNumber deployKg =
      new LoggedTunableNumber("Intake deploy kg", IntakeConstants.DEPLOY_MOTOR_GAINS.kG);

  /**
   * Creates a new DefaultFlywheelCommand.
   *
   * @param intake The intake subsystem.
   * @param spinManual Determines if manual intake spin button is pressed.
   * @param deployManual Determines if manual intake deploy button is pressed.
   */
  public DefaultIntakeCommand(
      Intake intake,
      BooleanSupplier spinManual,
      DoubleSupplier deployManual,
      BooleanSupplier backfeedManual) {
    this.spinManual = spinManual;
    this.deployManual = deployManual;
    this.backfeedManual = backfeedManual;

    this.intake = intake;

    flopTImer.start();
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (LoggingConstants.tuningMode
        && (deployKs.hasChanged(hashCode())
            || deployKg.hasChanged(hashCode())
            || deployKp.hasChanged(hashCode())
            || deployKv.hasChanged(hashCode()))) {
      intake.setDeployGains(
          new Slot0Configs()
              .withKV(deployKv.get())
              .withKG(deployKg.get())
              .withKS(deployKs.get())
              .withKP(deployKp.get()));
    }

    // First update spinning logic
    intakeSpinLogic();

    // Then update deploy logic
    intakeDeployLogic();
  }

  private void intakeSpinLogic() {
    switch (RobotState.instance().getIntakeSpinMode()) {
      case IDLE:
        if (RobotState.instance().getFlywheelMode() == FlywheelMode.SHOOTING
            || RobotState.instance().getFlywheelMode() == FlywheelMode.FRENZY) {
          intake.setIntakeSpinSpeed(IntakeConstants.INTAKE_SPIN_DUTY_CYCLE);
        } else {
          intake.setIntakeSpinSpeed(0.0);
        }
        break;

      case INTAKING:
        intake.setIntakeSpinSpeed(IntakeConstants.INTAKE_SPIN_DUTY_CYCLE);
        break;

      case OUTTAKING:
        intake.setIntakeSpinSpeed(-IntakeConstants.INTAKE_SPIN_DUTY_CYCLE);
        break;

      case MANUAL:
        if (spinManual.getAsBoolean()) {

          intake.setIntakeSpinSpeed(IntakeConstants.INTAKE_SPIN_DUTY_CYCLE);
        } else if (backfeedManual.getAsBoolean()) {
          intake.setIntakeSpinSpeed(-IntakeConstants.INTAKE_SPIN_DUTY_CYCLE);
        } else {
          intake.setIntakeSpinSpeed(0.0);
        }
        break;

      default:
        DriverStation.reportError(
            "Invalid intake spin mode: " + RobotState.instance().getIntakeSpinMode(), false);
        break;
    }
  }

  private void intakeDeployLogic() {

    IntakeDeployMode mode = RobotState.instance().getIntakeDeployMode();

    if (mode == IntakeDeployMode.DEPLOYING
        && MathUtil.isNear(
            IntakeConstants.FULLY_DEPLOYED_ANGLE.in(Degrees),
            intake.getDeployAngle().in(Degrees),
            5.0)) {
      // RobotState.instance().setIntakeDeployMode(IntakeDeployMode.IDLE_DEPLOYED);
      // mode = IntakeDeployMode.IDLE_DEPLOYED;
    }

    if (mode == IntakeDeployMode.RETRACTING
        && MathUtil.isNear(
            IntakeConstants.FULLY_RETRACTED_ANGLE.in(Degrees),
            intake.getDeployAngle().in(Degrees),
            5.0)) {
      // RobotState.instance().setIntakeDeployMode(IntakeDeployMode.IDLE_RETRACTED);
      // mode = IntakeDeployMode.IDLE_RETRACTED;
    }

    switch (mode) {
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

      case FLOPPING:
        if (flopTImer.hasElapsed(IntakeConstants.FLOP_PERIOD.in(Seconds))) {
          flopTImer.reset();
        } else if (flopTImer.hasElapsed(IntakeConstants.FLOP_PERIOD.in(Seconds) / 2.0)) {
          intake.setDeployControlRequest(
              IntakeConstants.INTAKE_MOTION_MAGIC_REQUEST.withPosition(
                  IntakeConstants.FULLY_DEPLOYED_ANGLE));
        } else {
          intake.setDeployControlRequest(
              IntakeConstants.INTAKE_MOTION_MAGIC_REQUEST.withPosition(IntakeConstants.FLOP_ANGLE));
        }
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

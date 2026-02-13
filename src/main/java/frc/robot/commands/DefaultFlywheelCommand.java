// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.Modes.FlywheelMode;
import frc.robot.managersubsystems.RobotState;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import java.util.function.BooleanSupplier;

public class DefaultFlywheelCommand extends Command {

  private final Flywheel flywheel;
  private final BooleanSupplier isTriggerPressed;

  /**
   * Creates a new DefaultFlywheelCommand.
   *
   * @param flywheel The flywheel subsystem.
   * @param isTriggerPressed Determines if manual backup button is pressed.
   */
  public DefaultFlywheelCommand(Flywheel flywheel, BooleanSupplier isTriggerPressed) {

    this.isTriggerPressed = isTriggerPressed;
    this.flywheel = flywheel;
    addRequirements(flywheel);
  }

  @Override
  public void initialize() {}

  // Change flywheel behavior based on current robot-flywheel state
  // PID control logic will be in this class, however, calculated targets come
  // from the ShotCalculator class
  @Override
  public void execute() {
    FlywheelMode currentMode = RobotState.instance().getFlywheelMode();

    switch (currentMode) {

        // Flywheel is idle, let it naturally spool down
      case IDLE:
        flywheel.setFlywheelVoltage(Volts.of(0.0));
        break;

        // Flywheel is preparing to shoot (revving), so use simple bang-bang control to
        // reach target angular velocity
      case PREPARE:
        Voltage prepareVolts =
            getBangBangVoltage(ShotCalculator.instance().getTargetFlywheelVelocity());
        flywheel.setFlywheelVoltage(prepareVolts);
        break;

        // Flywheel is shooting in typical fashion with known recovery period intervals
        // TODO this is for testing right now, bang-bang should be replaced later
      case SHOOTING:
        Voltage shootingVolts =
            getBangBangVoltage(ShotCalculator.instance().getTargetFlywheelVelocity());
        flywheel.setFlywheelVoltage(shootingVolts);
        break;

        // Flywheel is frantically shooting, little to no care about recovery interval
        // TODO this is for testing right now, bang-bang should be replaced later
      case FRENZY:
        Voltage frenzyVolts =
            getBangBangVoltage(ShotCalculator.instance().getTargetFlywheelVelocity());
        flywheel.setFlywheelVoltage(frenzyVolts);
        break;

        // Flywheel is operated by secondary controller, basically a true-false boolean
        // supplier
        // Target voltage is tuned for short shots like a popcorn-popper
        // TODO tune this
      case MANUAL:
        if (isTriggerPressed.getAsBoolean())
          flywheel.setFlywheelVoltage(ShooterConstants.FLYWHEEL_VOLTAGE_SHORT_SHOT_POPCORN);
        else flywheel.setFlywheelVoltage(Volts.of(0.0));
        break;

        // Invalid mode or unassigned behavior
      default:
        DriverStation.reportError("Invalid flywheel mode: " + currentMode, false);
        break;
    }
  }

  private Voltage getBangBangVoltage(AngularVelocity target) {
    return Volts.of(MathUtil.clamp(target.magnitude() / 50.0, -12, 12));
  }

  @Override
  public void end(boolean interrupted) {}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.managersubsystems.RobotState;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private IntakeIO intakeIO;
  private IntakeInputsAutoLogged loggedInputs = new IntakeInputsAutoLogged();

  /** Creates a new Intake. */
  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;

    intakeIO.configureMotors();
  }

  public void setIntakeSpinSpeed(double speed) {
    intakeIO.setSpinMotorSpeed(speed);
  }

  public void setIntakeSpinRequest(ControlRequest request) {
    intakeIO.setSpinMotorRequest(request);
  }

  public void setDeployControlRequest(ControlRequest request) {
    intakeIO.setDeployMotorRequest(request);
  }

  public Angle getDeployAngle() {
    return intakeIO.getDeployMotorAbsPos();
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(loggedInputs);

    Logger.processInputs("Intake", loggedInputs);

    RobotState.instance()
        .setIntakeFullyRetracted(
            MathUtil.isNear(
                loggedInputs.deployAngleDegrees,
                IntakeConstants.FULLY_RETRACTED_ANGLE.in(Degrees),
                IntakeConstants.INTAKE_RETRACTED_THRESHOLD.in(Degrees)));

    RobotState.instance()
        .setIntakeCurrentDraw(
            loggedInputs
                .deployMotorSuppliedCurrent
                .plus(loggedInputs.leftSpinMotorSuppliedCurrent)
                .plus(loggedInputs.rightSpinMotorSuppliedCurrent));
  }

  public void setDeployGains(Slot0Configs gains) {
    intakeIO.setDeployGains(gains);
  }
}

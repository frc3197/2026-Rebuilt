// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private IntakeIO intakeIO;
  private IntakeInputsAutoLogged loggedInputs = new IntakeInputsAutoLogged();

  /** Creates a new Intake. */
  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;

    intakeIO.configureMotors();
  }

  /**
   * Starts the intaker at the desired params
   *
   * @param speed (DoubleSupplier) The speed to run the intaker
   * @return none There is no return
   */
  public Command setIntakeSpinSpeedCommand(double speed) {
    return Commands.runOnce(
        () -> {
          intakeIO.setSpinMotorSpeed(speed);
        },
        this);
  }

  public void setIntakeSpinSpeed(double speed) {
    intakeIO.setSpinMotorSpeed(speed);
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
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {

  private IntakeIO intakeIO;

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
  public Command startIntake(DoubleSupplier speed) {
    return Commands.runOnce(
        () -> {
          intakeIO.setSpinMotorSpeed(speed.getAsDouble());
          intakeIO.setDeployMotorSpeed(.2); // Find more reasonable value
        },
        this);
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(null);
  }
}

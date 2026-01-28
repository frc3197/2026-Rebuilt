// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Intake extends SubsystemBase {

  private IntakeIO intakeIO;

  /** Creates a new Intake. */
  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  /**
   * Starts the intaker at the desired params
   *
   * @param angle The desired angle of the intaker
   * @param speed The speed to run the intaker
   * @return None
   */
  public Command startIntake(Supplier<Angle> angle, DoubleSupplier speed) {
    return Commands.runOnce(
        () -> {
          Angle suppliedAngle = angle.get();
          Double suppliedSpeed = speed.getAsDouble();
        },
        this);
  }

  @Override
  public void periodic() {
    //intakeIO.updateInputs(null);
  }
}

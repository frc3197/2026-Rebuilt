// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.managersubsystems.RobotState;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretParameters;

public class Turret extends SubsystemBase {

  private TurretIO turretIO;

  private TurretInputsAutoLogged loggedShooter = new TurretInputsAutoLogged();

  public Turret(TurretIO turretIO) {
    this.turretIO = turretIO;
  }

  @Override
  public void periodic() {
    turretIO.updateInputs(loggedShooter);

    RobotState.instance().setTurretRotationAngle(loggedShooter.turretAngle);
  }

  public TurretParameters getTurretParameters() {
    return turretIO.getTurretParameters();
  }

  public void setTurretRotationMotorVoltage(Voltage volts) {
    double maxVolts = ShooterConstants.MAX_TURRET_ROTATION_MOTOR_VOLTS.magnitude();
    turretIO.setTurretMotorVolts(Volts.of(MathUtil.clamp(volts.magnitude(), -maxVolts, maxVolts)));
  }
}

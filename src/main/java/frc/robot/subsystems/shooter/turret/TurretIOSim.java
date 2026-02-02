package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.ShotCalculator;

public class TurretIOSim implements TurretIO {

  private final DCMotor turretGearbox = DCMotor.getKrakenX60(1);

  // Random MOI guess
  private final DCMotorSim turretRotationMotor =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(turretGearbox, 2, 0.7), turretGearbox);

  public TurretIOSim() {}

  @Override
  public void updateInputs(TurretInputs inputs) {
    if (turretRotationMotor.getInputVoltage() == 0.0)
      turretRotationMotor.setAngularVelocity(
          turretRotationMotor.getAngularVelocity().in(RadiansPerSecond)
              * (turretRotationMotor.getInputVoltage() / 12));
    turretRotationMotor.update(0.02);

    inputs.turretAngleSuppliedVoltage = turretRotationMotor.getInputVoltage();
    inputs.turretAngle = turretRotationMotor.getAngularPosition();
  }

  @Override
  public void setTurretMotorVolts(Voltage volts) {
    turretRotationMotor.setInputVoltage(volts.magnitude());
  }

  @Override
  public TurretParameters getTurretParameters() {
    Angle currentTurretAngle = turretRotationMotor.getAngularPosition();
    Angle targetTurretAngle = ShotCalculator.instance().getTargetTurretAngle();

    TurretParameters params = new TurretParameters();
    Angle error = currentTurretAngle.minus(targetTurretAngle);
    Angle clampedError = Degrees.of(error.in(Degrees) % 360);
    if (clampedError.in(Degrees) > 180) {
      params.turretRotationError = clampedError.minus(Degrees.of(360));
    } else if (clampedError.in(Degrees) < -180) {
      params.turretRotationError = clampedError.plus(Degrees.of(360));

    } else {
      params.turretRotationError = clampedError;
    }
    SmartDashboard.putNumber("ERROR TURRRR", params.turretRotationError.in(Degrees));

    return params;
  }
}

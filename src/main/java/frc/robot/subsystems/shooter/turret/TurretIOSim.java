package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.shooter.ShotCalculator;

public class TurretIOSim implements TurretIO {

  private final DCMotor turretGearbox = DCMotor.getKrakenX60(1);
  private TurretParameters params = new TurretParameters();

  // Random MOI guess
  private final DCMotorSim turretRotationMotor =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(turretGearbox, 0.25, 0.7), turretGearbox);

  public TurretIOSim() {}

  @Override
  public void updateInputs(TurretInputs inputs) {
    turretRotationMotor.setAngularVelocity(
        turretRotationMotor.getAngularVelocity().in(RadiansPerSecond) * (0.85));
    turretRotationMotor.update(0.02);

    inputs.turretMotorAngle.mut_replace(turretRotationMotor.getAngularPosition());
  }

  @Override
  public void setTurretMotorVolts(Voltage volts) {
    turretRotationMotor.setInputVoltage(volts.magnitude());
  }

  @Override
  public TurretParameters getTurretParameters() {
    Angle currentTurretAngle = turretRotationMotor.getAngularPosition();
    Angle targetTurretAngle = ShotCalculator.instance().getTargetTurretAngleWithOmegaLookahead();

    if (targetTurretAngle.gt(Degrees.of(180))) {
      targetTurretAngle = targetTurretAngle.minus(Degrees.of(360));
    }

    if (targetTurretAngle.lt(Degrees.of(-180))) {
      targetTurretAngle = targetTurretAngle.plus(Degrees.of(360));
    }

    params.turretRotationError.mut_replace(currentTurretAngle.minus(targetTurretAngle));
    params.turretRotationTarget.mut_replace(targetTurretAngle);
    params.turretRotation.mut_replace(currentTurretAngle);

    return params;
  }

  @Override
  public void zeroTurretEncoder() {
    System.out.println("Zeroed turret rotation");
    turretRotationMotor.setAngle(0.0);
  }
}

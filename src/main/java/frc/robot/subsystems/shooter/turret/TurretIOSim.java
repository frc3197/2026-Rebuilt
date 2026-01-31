package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
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
    params.turretRotationError = currentTurretAngle.minus(targetTurretAngle);

    return params;
  }
}

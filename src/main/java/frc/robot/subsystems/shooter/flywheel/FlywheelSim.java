package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelSim implements FlywheelIO {

  private final DCMotor flywheelGearbox = DCMotor.getKrakenX60(1);

  private final DCMotorSim flywheelMotorSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(flywheelGearbox, 5, 1.0), flywheelGearbox);

  private FlywheelParameters params = new FlywheelParameters();

  public FlywheelSim() {}

  @Override
  public void setFlywheelMotorVolts(Voltage volts) {
    flywheelMotorSim.setInputVoltage(volts.magnitude());
  }

  @Override
  public void setFlywheelTargetVelocity(AngularVelocity velocity) {}

  @Override
  public FlywheelParameters getFlywheelParameters() {
    return params;
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {

    flywheelMotorSim.setAngularVelocity(
        flywheelMotorSim.getAngularVelocity().in(RadiansPerSecond) * (0.85));
    flywheelMotorSim.update(0.02);

    // inputs.flywheelCurrentDraw.mut_replace(flywheelMotor.getSupplyCurrent().getValue());
    // inputs.flywheelSuppliedVoltage.mut_replace(flywheelMotor.getSupplyVoltage().getValue());
    inputs.flywheelVelocity = getFlywheelVelocity();
    inputs.flywheelCurrentDraw.mut_replace(Amps.of(flywheelMotorSim.getCurrentDrawAmps()));
    inputs.flywheelSuppliedVoltage.mut_replace(Volts.of(flywheelMotorSim.getInputVoltage()));
  }

  // Helper functions
  private AngularVelocity getFlywheelVelocity() {
    return flywheelMotorSim.getAngularVelocity();
  }
}

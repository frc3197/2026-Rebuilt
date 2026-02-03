package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelSim implements FlywheelIO {

    private final DCMotor flywheelGearbox = DCMotor.getKrakenX60(1);

    private final DCMotorSim flywheelMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(flywheelGearbox, 5, 1.0), flywheelGearbox);

    private FlywheelParameters params = new FlywheelParameters();

    public FlywheelSim() {
    }

    @Override
    public void setFlywheelMotorVolts(Voltage volts) {
    }

    @Override
    public void setFlywheelTargetVelocity(AngularVelocity velocity) {
    }

    @Override
    public FlywheelParameters getFlywheelParameters() {
        return params;
    }

    @Override
    public void updateInputs(FlywheelInputs inputs) {
        // inputs.flywheelCurrentDraw.mut_replace(flywheelMotor.getSupplyCurrent().getValue());
        // inputs.flywheelSuppliedVoltage.mut_replace(flywheelMotor.getSupplyVoltage().getValue());
        inputs.flywheelVelocity = getFlywheelVelocity();
    }

    // Helper functions
    private AngularVelocity getFlywheelVelocity() {
        return flywheelMotorSim.getAngularVelocity();
    }
}

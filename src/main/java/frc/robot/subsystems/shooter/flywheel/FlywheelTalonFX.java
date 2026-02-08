package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.HardwareID;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.RealSubsystem;

public class FlywheelTalonFX extends RealSubsystem implements FlywheelIO {

  private final TalonFX flywheelMotor;
  private AngularVelocity targetFlywheelAngularVelocity = RotationsPerSecond.of(0.0);

  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
  private final VoltageOut voltageOutRequest = new VoltageOut(0.0);

  private FlywheelParameters params = new FlywheelParameters();

  public FlywheelTalonFX() {
    this.flywheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR_ID, HardwareID.MAIN_CANBUS);

    configureHardware();
  }

  @Override
  protected void configureHardware() {
    flywheelMotor.getConfigurator().apply(ShooterConstants.FLYWHEEL_TALON_FX_CONFIG);
    flywheelMotor.getConfigurator().apply(ShooterConstants.FLYWHEEL_SLOT0_CONFIGS);
  }

  @Override
  public void setFlywheelMotorVolts(Voltage volts) {
    flywheelMotor.setControl(voltageOutRequest.withOutput(volts));
  }

  @Override
  public void setFlywheelTargetVelocity(AngularVelocity velocity) {
    targetFlywheelAngularVelocity = velocity;
    flywheelMotor.setControl(velocityVoltageRequest.withVelocity(velocity));
  }

  @Override
  public FlywheelParameters getFlywheelParameters() {
    params.rpsError = targetFlywheelAngularVelocity.minus(getFlywheelVelocity());
    return params;
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
    inputs.flywheelCurrentDraw.mut_replace(flywheelMotor.getSupplyCurrent().getValue());
    inputs.flywheelSuppliedVoltage.mut_replace(flywheelMotor.getSupplyVoltage().getValue());
    inputs.flywheelVelocity = getFlywheelVelocity();
  }

  // Helper functions
  private AngularVelocity getFlywheelVelocity() {
    return flywheelMotor.getVelocity().getValue();
  }
}

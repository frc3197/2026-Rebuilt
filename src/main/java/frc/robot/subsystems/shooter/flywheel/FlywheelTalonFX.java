package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.HardwareID;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.RealSubsystem;

public class FlywheelTalonFX extends RealSubsystem implements FlywheelIO {

  private final TalonFX flywheelMotor;
  private AngularVelocity targetFlywheelAngularVelocity = RotationsPerSecond.of(0.0);

  private FlywheelParameters params = new FlywheelParameters();

  public FlywheelTalonFX() {
    this.flywheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR_ID, HardwareID.MAIN_CANBUS);

    configureHardware();
  }

  @Override
  protected void configureHardware() {
    flywheelMotor.getConfigurator().apply(ShooterConstants.FLYWHEEL_TALON_FX_CONFIG);
  }

  @Override
  public void setFlywheelOutput(ControlRequest request) {
    flywheelMotor.setControl(request);
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
    inputs.flywheelCurrentDraw.mut_replace(flywheelMotor.getSupplyCurrent().getValue());
    inputs.flywheelSuppliedVoltage.mut_replace(flywheelMotor.getSupplyVoltage().getValue());
    inputs.flywheelTargetVelocityRPS = (getFlywheelVelocity().in(RotationsPerSecond));
    inputs.flywheelTargetVelocity =
        (ShotCalculator.instance().getTargetFlywheelVelocity().in(RotationsPerSecond));
  }

  // Helper functions
  private AngularVelocity getFlywheelVelocity() {
    return flywheelMotor.getVelocity().getValue();
  }

  @Override
  public void updateFlywheelSlot0Configs(Slot0Configs newConfig) {
    flywheelMotor.getConfigurator().apply(newConfig);
  }
}

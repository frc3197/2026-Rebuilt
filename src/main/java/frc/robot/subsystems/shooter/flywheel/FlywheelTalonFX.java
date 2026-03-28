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

  private final TalonFX flywheelLeftMotor;
  private final TalonFX flywheelRightMotor;

  public FlywheelTalonFX() {
    this.flywheelLeftMotor =
        new TalonFX(ShooterConstants.FLYWHEEL_LEFT_MOTOR_ID, HardwareID.MAIN_CANBUS);
    this.flywheelRightMotor =
        new TalonFX(ShooterConstants.FLYWHEEL_RIGHT_MOTOR_ID, HardwareID.MAIN_CANBUS);

    configureHardware();
  }

  @Override
  protected void configureHardware() {
    flywheelLeftMotor.getConfigurator().apply(ShooterConstants.FLYWHEEL_LEFT_TALON_FX_CONFIG);
    flywheelRightMotor.getConfigurator().apply(ShooterConstants.FLYWHEEL_RIGHT_TALON_FX_CONFIG);
  }

  @Override
  public void setFlywheelOutput(ControlRequest request) {
    flywheelLeftMotor.setControl(request);
    flywheelRightMotor.setControl(request);
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
    inputs.flywheelCurrentDraw.mut_replace(flywheelLeftMotor.getSupplyCurrent().getValue());
    inputs.flywheelSuppliedVoltage.mut_replace(flywheelLeftMotor.getSupplyVoltage().getValue());
    inputs.flywheelTargetVelocity =
        (ShotCalculator.instance().getTargetFlywheelVelocity().in(RotationsPerSecond));
    inputs.flywheelVelocityActual = getFlywheelVelocity().in(RotationsPerSecond);
  }

  // Helper functions
  private AngularVelocity getFlywheelVelocity() {
    return flywheelLeftMotor.getVelocity().getValue();
  }

  @Override
  public void updateFlywheelSlot0Configs(Slot0Configs newConfig) {
    flywheelLeftMotor.getConfigurator().apply(newConfig);
    flywheelRightMotor.getConfigurator().apply(newConfig);
  }
}

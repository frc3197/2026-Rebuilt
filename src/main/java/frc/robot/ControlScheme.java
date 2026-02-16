package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class ControlScheme {

  private final CommandXboxController primaryController;
  private final CommandXboxController secondaryController;

  public ControlScheme(CommandXboxController pC, CommandXboxController sC) {
    this.primaryController = pC;
    this.secondaryController = sC;
  }

  // Drive button mappings

  public double getDriveX() {
    return primaryController.getLeftY();
  }

  public double getDriveY() {
    return primaryController.getLeftX();
  }

  public double getDriveRotation() {
    return primaryController.getRightX();
  }

  public Trigger getZeroGyro() {
    return primaryController.start();
  }

  // FLywheel modes
  public Trigger spoolFlywheel() {
    return primaryController.a();
  }

  public Trigger idleFlywheel() {
    return primaryController.b();
  }

  // Manual fallbacks ---------------------------------------------------------

  // Climber
  public Trigger getClimberRotateCW() {
    return secondaryController.leftBumper();
  }

  public Trigger getClimberRotateCWW() {
    return secondaryController.rightBumper();
  }

  // Index
  public Trigger getSpindexFeedFlywheelManual() {
    return secondaryController.a();
  }

  public Trigger getFeedManual() {
    return secondaryController.y();
  }

  // Intake
  public Trigger getIntakeSpinManual() {
    return secondaryController.x();
  }

  public DoubleSupplier getIntakeDeployManual() {
    return () -> 0.0;
  }

  public Trigger getIntakeExtendPreset() {
    return primaryController.povDown();
  }

  public Trigger getIntakeRetractPreset() {
    return primaryController.povUp();
  }

  // Shooter
  public Trigger getSpoolFlywheelManual() {
    return secondaryController.b();
  }

  public Trigger getHoodAngleMaximum() {
    return secondaryController.povUp();
  }

  public Trigger getHoodAngleMedium() {
    return secondaryController.povRight();
  }

  public Trigger getHoodAngleMinimum() {
    return secondaryController.povDown();
  }

  public DoubleSupplier getTurretVoltageManual() {
    return () ->
        secondaryController.getLeftTriggerAxis() - secondaryController.getRightTriggerAxis();
  }
}

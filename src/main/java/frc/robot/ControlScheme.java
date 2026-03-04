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

  public Trigger getAlignClimb() {
    return primaryController.y().and(() -> true);
  }

  public Trigger getSnap45() {
    return primaryController.leftStick();
  }

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

  // Flywheel modes
  public Trigger getPrepareFlywheel() {
    return primaryController.a();
  }

  public Trigger getShootingFlywheel() {
    return primaryController.rightTrigger();
  }

  public Trigger getIdleFlywheel() {
    return primaryController.b();
  }

  // Manual fallbacks ---------------------------------------------------------

  // Climber
  public Trigger getClimberRotateCW() {
    return secondaryController.b();
  }

  public Trigger getClimberRotateCWW() {
    return secondaryController.x();
  }

  // Index
  public Trigger getSpindexFeedFlywheelManual() {
    return secondaryController.a();
  }

  public Trigger getBackfeedManual() {
    return primaryController.back().or(secondaryController.y());
  }

  // Intake
  public Trigger getIntakeSpin() {
    return secondaryController.leftStick();
  }

  public Trigger getIntakeSpinStop() {
    return primaryController.rightStick();
  }

  public Trigger getIntakeSpinManual() {
    return new Trigger(() -> false);
  }

  public DoubleSupplier getIntakeDeployManual() {
    return () -> 0.0;
  }

  public Trigger getIntakeExtendPreset() {
    return primaryController.povUp();
  }

  public Trigger getIntakeRetractPreset() {
    return primaryController.povDown();
  }

  public Trigger intakeDeployAndSpin() {
    return primaryController.leftTrigger();
  }

  public Trigger startFloppping() {
    return secondaryController.rightStick();
  }

  // Shooter

  public Trigger getSpoolFlywheelManual() {
    return new Trigger(() -> false);
  }

  public Trigger autoZeroTurret() {
    return secondaryController.back();
  }

  public Trigger zeroTurret() {
    return secondaryController.start();
  }

  public Trigger getClimberUp() {
    return secondaryController.povUp();
  }

  public Trigger getClimberPull() {
    return secondaryController.povRight();
  }

  public Trigger getClimberStow() {
    return secondaryController.povDown();
  }

  public DoubleSupplier getTurretVoltageManual() {
    return () ->
        secondaryController.getLeftTriggerAxis() - secondaryController.getRightTriggerAxis();
  }

  public Trigger getTurretTrack() {
    return secondaryController.leftBumper();
  }

  public Trigger getTurretIdle() {
    return secondaryController.rightBumper();
  }
}

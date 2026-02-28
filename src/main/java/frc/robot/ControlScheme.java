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

  public Trigger getAlignClimb1() {
    return primaryController.y();
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
  public Trigger prepareFlywheel() {
    return primaryController.a();
  }

  public Trigger shootingFlywheel() {
    return primaryController.rightTrigger();
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

  public Trigger getBackfeedManual() {
    return secondaryController.y().or(primaryController.leftStick());
  }

  // Intake
  public Trigger getIntakeSpin() {
    return primaryController.rightStick();
  }

  public Trigger getIntakeSpinStop() {
    return primaryController.leftStick();
  }

  public Trigger getIntakeSpinManual() {
    return secondaryController.x();
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
    return secondaryController.b();
  }

  public Trigger autoZeroTurret() {
    return secondaryController.back();
  }

  public Trigger getBackfeedIndexManual() {
    return secondaryController.povLeft();
  }

  public Trigger zeroTurret() {
    return secondaryController.start();
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

  public Trigger getTurretTrack() {
    return primaryController.leftBumper();
  }

  public Trigger getTurretIdle() {
    return primaryController.rightBumper();
  }
}

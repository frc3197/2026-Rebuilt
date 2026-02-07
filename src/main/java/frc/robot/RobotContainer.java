// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.auto.AutoLookup;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.LoggingConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.enums.Modes.TurretMode;
import frc.robot.enums.RealAutos;
import frc.robot.managersubsystems.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.IndexIO;
import frc.robot.subsystems.index.IndexIOSim;
import frc.robot.subsystems.index.IndexIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelTalonFX;
import frc.robot.subsystems.shooter.turret.DefaultTurretCommand;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  // private final Climber climber;
  private final Drive drive;
  private final Flywheel flywheel;
  private final Index index;
  private final Intake intake;
  private final Turret turret;
  private final Vision vision;

  private final AutoLookup autoLookup;

  // Controllers
  private final CommandXboxController driveController = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Triggers
  private final Trigger enteredAllianceZoneTrigger =
      new Trigger(() -> RobotState.instance().inAllianceZone());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (LoggingConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations

        // TODO FIX THIS WHEN CLIMBER IS ADDED
        // climber = new Climber(new ClimberIOSim());

        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        flywheel = new Flywheel(new FlywheelTalonFX());

        index = new Index(new IndexIOTalonFX());

        intake = new Intake(new IntakeIOTalonFX());

        turret = new Turret(new TurretIOTalonFX());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        // climber = new Climber(new ClimberIOSim());

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        index = new Index(new IndexIOSim());

        intake = new Intake(new IntakeIOSim());

        flywheel = new Flywheel(new FlywheelSim());

        turret = new Turret(new TurretIOSim());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));

        break;

      default:
        // Replayed robot, disable IO implementations

        // climber = new Climber(new ClimberIO() {});

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        index = new Index(new IndexIO() {});

        intake = new Intake(new IntakeIO() {});

        flywheel = new Flywheel(new FlywheelIO() {});

        turret = new Turret(new TurretIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        break;
    }

    this.autoLookup = new AutoLookup(drive);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption("Right Bump Auto", autoLookup.getAuto(RealAutos.Right_Bump));

    // Configure the button bindings
    configureButtonBindings();

    // Configure trigger callbacks
    configureTriggerCallbacks();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));

    turret.setDefaultCommand(
        new DefaultTurretCommand(turret)
            .onlyWhile(
                () ->
                    RobotState.instance().getTurretMode() == TurretMode.TRACKING_HUB
                        || RobotState.instance().getTurretMode() == TurretMode.PASSING));

    // Switch to X pattern when X button is pressed
    // driveController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // driveController.leftBumper().onTrue(Commands.runOnce(() ->
    // setRobotMode("Left")));
    // driveController.rightBumper().onTrue(Commands.runOnce(() ->
    // setRobotMode("Right")));

    /*
     * driveController
     * .a()
     * .onTrue(climber.setClimbPostion(Meters.of(1.0)))
     * .onFalse(climber.setClimbPostion(Meters.of(0.0)));
     */

    /*
     * driveController
     * .rightBumper()
     * .onTrue(index.setIndexMotor(Volts.of(-4.0)))
     * .onFalse(index.setIndexMotor(Volts.of(0.0)));
     */

    driveController
        .x()
        .onTrue(intake.setIntakeSpinSpeed(0.5))
        .onFalse(intake.setIntakeSpinSpeed(0.0));

    driveController.povUp().onTrue(setTurretMode(TurretMode.TRACKING_HUB));
    driveController.povDown().onTrue(setTurretMode(TurretMode.IDLE));

    driveController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }

  private void configureTriggerCallbacks() {
    enteredAllianceZoneTrigger
        .onTrue(
            setTurretMode(
                RobotState.instance().getTurretMode() == TurretMode.IDLE
                    ? TurretMode.TRACKING_HUB
                    : RobotState.instance().getTurretMode()))
        .onFalse(
            setTurretMode(
                RobotState.instance().getTurretMode() == TurretMode.TRACKING_HUB
                    ? TurretMode.IDLE
                    : RobotState.instance().getTurretMode()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public static Command setTurretMode(TurretMode mode) {
    return Commands.runOnce(() -> RobotState.instance().setTurretMode(mode));
  }

  public static boolean isRed() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }
}

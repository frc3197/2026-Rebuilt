// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.auto.AutoLookup;
import frc.robot.commands.AlignClimb;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.DefaultFlywheelCommand;
import frc.robot.commands.DefaultIndexCommand;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.DefaultTurretHoodCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Sim.SimTurretCommand;
import frc.robot.commands.ZeroTurret;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.LoggingConstants;
import frc.robot.constants.LoggingConstants.Mode;
import frc.robot.constants.TunerConstants;
import frc.robot.enums.Modes.ClimbCameraMode;
import frc.robot.enums.Modes.FlywheelMode;
import frc.robot.enums.Modes.IntakeDeployMode;
import frc.robot.enums.Modes.IntakeSpinMode;
import frc.robot.enums.Modes.TurretMode;
import frc.robot.enums.RealAutos;
import frc.robot.managersubsystems.RobotState;
import frc.robot.subsystems.Align;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.IndexConstants;
import frc.robot.subsystems.index.IndexIO;
import frc.robot.subsystems.index.IndexIOSim;
import frc.robot.subsystems.index.IndexIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.quest.Quest;
import frc.robot.subsystems.quest.QuestIO;
import frc.robot.subsystems.quest.QuestIOReal;
import frc.robot.subsystems.quest.QuestIOSim;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelTalonFX;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOClimberLimelight;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Align align;
  private final Climber climber;
  private final Drive drive;
  private final Flywheel flywheel;
  private final Index index;
  private final Intake intake;
  private final Turret turret;
  private final Quest quest;
  private final Vision vision;

  // Autos
  private final AutoLookup autoLookup;

  // Controllers & mappings
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController secondaryController = new CommandXboxController(1);
  // TODO make it so both controllers arent the only ones haha :)
  private final ControlScheme controlScheme =
      new ControlScheme(driveController, secondaryController);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Triggers
  // -------------------------------------------------------------------------
  // Trigger for when robot enters the alliance zone, used to automatically begin
  // tracking hub
  private final Trigger enteredAllianceZoneAuto =
      new Trigger(
          () ->
              RobotState.instance().inAllianceZone()
                  && (LoggingConstants.currentMode != Mode.REAL || DriverStation.isAutonomous()));

  // Triggers only during autonomous period, deploys intake and begins spinning
  // when robot enters neutral zone
  private final Trigger enteredNeutralZoneAuto =
      new Trigger(
          () ->
              (RobotState.instance().inNeutralZone()
                  && (LoggingConstants.currentMode != Mode.REAL || DriverStation.isAutonomous())));

  private final Trigger startFlopping =
      new Trigger(
          () ->
              RobotState.instance().getRobotSpeedMPS()
                      < IntakeConstants.MAX_FLOP_VELOCITY.in(MetersPerSecond)
                  && (RobotState.instance().getFlywheelMode() == FlywheelMode.FRENZY
                      || RobotState.instance().getFlywheelMode() == FlywheelMode.SHOOTING));

  public RobotContainer() {

    this.align = new Align();

    switch (LoggingConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations

        climber = new Climber(new ClimberIOTalonFX());

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

        quest = new Quest(new QuestIOReal(), drive::addVisionMeasurement);

        vision =
            new Vision(
                quest::isQuestConnected,
                drive::addVisionMeasurement,
                quest::acceptVisionPose,
                new VisionIOLimelight(VisionConstants.LIMELIGHT_NAME, drive::getRotation),
                new VisionIOClimberLimelight(
                    VisionConstants.CLIMBER_LIMELIGHT_NAME, drive::getRotation));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        climber = new Climber(new ClimberIOSim());

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

        quest = new Quest(new QuestIOSim(), drive::addVisionMeasurement);

        /*
         * vision =
         * new Vision(
         * drive::addVisionMeasurement,
         * new VisionIOPhotonVisionSim(
         * VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
         * new VisionIOPhotonVisionSim(
         * VisionConstants.camera1Name, VisionConstants.robotToCamera1,
         * drive::getPose));
         */
        vision =
            new Vision(
                quest::isQuestConnected,
                drive::addVisionMeasurement,
                quest::acceptVisionPose,
                new VisionIO() {},
                new VisionIO() {});

        break;

      default:
        // Replayed robot, disable IO implementations

        climber = new Climber(new ClimberIO() {});

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

        quest = new Quest(new QuestIO() {}, drive::addVisionMeasurement);

        vision =
            new Vision(
                quest::isQuestConnected,
                drive::addVisionMeasurement,
                quest::acceptVisionPose,
                new VisionIO() {},
                new VisionIO() {});

        break;
    }

    // Initialize auto lookup with appropriate subsystems
    this.autoLookup = new AutoLookup(align, climber, drive, turret, vision);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Real auto routines
    autoChooser.addOption("Right Bump Twice Auto", autoLookup.getAuto(RealAutos.Right_Bump_Twice));
    autoChooser.addOption("Right Bump Climb Auto", autoLookup.getAuto(RealAutos.Right_Bump_Climb));

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

    // Configure the button bindings
    configureButtonBindings();

    // Configure trigger callbacks
    configureTriggerCallbacks();
  }

  private void configureButtonBindings() {

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controlScheme.getDriveX(),
            () -> -controlScheme.getDriveY(),
            () -> -controlScheme.getDriveRotation()));

    index.setDefaultCommand(new DefaultIndexCommand(index));

    intake.setDefaultCommand(
        new DefaultIntakeCommand(
            intake,
            controlScheme.getIntakeSpinManual(),
            controlScheme.getIntakeDeployManual(),
            controlScheme.getBackfeedManual()));

    // Flywheel is controlled based on FlywheelMode with manual override buttons
    flywheel.setDefaultCommand(
        new DefaultFlywheelCommand(flywheel, controlScheme.getSpoolFlywheelManual()));

    // Turret is controlled by TurretMode with manual overrides
    if (LoggingConstants.currentMode != Mode.SIM)
      turret.setDefaultCommand(
          new DefaultTurretHoodCommand(turret, controlScheme.getTurretVoltageManual()));
    else
      turret.setDefaultCommand(
          new SimTurretCommand(turret, controlScheme.getTurretVoltageManual()));

    // Switch to X pattern when X button is pressed
    // .onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Not gonna test now
    // .onTrue(setTurretMode(TurretMode.TRACKING_HUB));
    // .onTrue(setTurretMode(TurretMode.IDLE));

    controlScheme
        .getBackfeedManual()
        .onTrue(
            index
                .setSpindexMotorCommand(Volts.of(-5.35))
                .andThen(setIntakeSpinMode(IntakeSpinMode.OUTTAKING)))
        .onFalse(
            index
                .setSpindexMotorCommand(Volts.of(0.0))
                .andThen(setIntakeSpinMode(IntakeSpinMode.IDLE)));

    controlScheme
        .getSpindexFeedFlywheelManual()
        .onTrue(
            index
                .setSpindexMotorCommand(Volts.of(3.5))
                .andThen(
                    Commands.runOnce(
                        () ->
                            index.setFeedRequest(
                                IndexConstants.FEED_TORQUE_REQUEST.withVelocity(
                                    RotationsPerSecond.of(100))))))
        .onFalse(
            index
                .setSpindexMotorCommand(Volts.of(0.0))
                .andThen(
                    Commands.runOnce(() -> index.setFeedRequest(new VoltageOut(Volts.of(0.0))))));

    controlScheme.startFloppping().onTrue(setIntakeDeployMode(IntakeDeployMode.FLOPPING));

    // Zeroes the robot
    controlScheme
        .getZeroGyro()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                isRed() ? Rotation2d.k180deg : Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Driver controller bindings to spool flywheel
    controlScheme.getPrepareFlywheel().onTrue(setFlywheelMode(FlywheelMode.PREPARE));
    controlScheme.getIdleFlywheel().onTrue(setFlywheelMode(FlywheelMode.IDLE));
    controlScheme
        .getShootingFlywheel()
        .whileTrue(
            setFlywheelMode(FlywheelMode.FRENZY)
                .andThen(
                    Commands.runOnce(
                        () -> {
                          if (RobotState.instance().getTurretMode() == TurretMode.IDLE) {
                            RobotState.instance().setTurretMode(TurretMode.TRACKING_HUB);
                          }
                        })))
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (RobotState.instance().getFlywheelMode() == FlywheelMode.SHOOTING
                      || RobotState.instance().getFlywheelMode() == FlywheelMode.FRENZY) {
                    RobotState.instance().setFlywheelMode(FlywheelMode.PREPARE);
                  }
                  if (RobotState.instance().getIntakeDeployMode() == IntakeDeployMode.FLOPPING) {
                    RobotState.instance().setIntakeDeployMode(IntakeDeployMode.DEPLOYING);
                  }
                }));

    controlScheme
        .intakeDeployAndSpin()
        .onTrue(
            setIntakeDeployMode(IntakeDeployMode.DEPLOYING)
                .andThen(setIntakeSpinMode(IntakeSpinMode.INTAKING)));

    controlScheme.getIntakeSpin().onTrue(setIntakeSpinMode(IntakeSpinMode.INTAKING));
    controlScheme.getIntakeSpinStop().onTrue(setIntakeSpinMode(IntakeSpinMode.IDLE));

    // ------------------------------------------------------------------------
    // CLIMBER CONTROLS
    // -----------------------------------------------------------------------
    controlScheme
        .getClimberUp()
        .onTrue(
            Commands.runOnce(
                () ->
                    climber.setClimberControlType(
                        new PositionDutyCycle(ClimberConstants.climberUpAngle)),
                climber));
    controlScheme
        .getClimberPull()
        .onTrue(
            Commands.runOnce(
                () ->
                    climber.setClimberControlType(
                        new PositionDutyCycle(ClimberConstants.climberClimbAngle)),
                climber));
    controlScheme
        .getClimberStow()
        .onTrue(
            Commands.runOnce(
                () ->
                    climber.setClimberControlType(
                        new PositionDutyCycle(ClimberConstants.climberStowAngle)),
                climber));
    controlScheme
        .getAlignClimb()
        .whileTrue(
            new SequentialCommandGroup(
                    setClimbCameraMode(
                        isRed() ? ClimbCameraMode.CLIMB_RED : ClimbCameraMode.CLIMB_BLUE),
                    Commands.runOnce(
                        () ->
                            climber.setClimberControlType(
                                new PositionDutyCycle(ClimberConstants.climberUpAngle)),
                        climber),
                    new AlignCommand(
                            align,
                            drive,
                            (RobotContainer.isRed()
                                ? FlippingUtil.flipFieldPose(FieldConstants.CLIMB_ALIGN_POSE)
                                : FieldConstants.CLIMB_ALIGN_POSE))
                        .withTimeout(2.0),
                    new AlignClimb(() -> vision.getTargetX(1).getDegrees(), drive).withTimeout(1.0),
                    Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0, 0.4, 0)), drive)
                        .withTimeout(1.5),
                    Commands.runOnce(
                        () ->
                            climber.setClimberControlType(
                                new PositionDutyCycle(ClimberConstants.climberClimbAngle)),
                        climber),
                    setClimbCameraMode(ClimbCameraMode.APRIL_TAGS))
                .onlyIf(RobotState.instance().getIntakeFullyRetractedSupplier()));

    controlScheme
        .getClimberRotateCW()
        .onTrue(climber.setClimbSpeed(1.0))
        .onFalse(climber.setClimbSpeed(0.0));

    controlScheme
        .getClimberRotateCWW()
        .onTrue(climber.setClimbSpeed(-1.0))
        .onFalse(climber.setClimbSpeed(0.0));

    controlScheme.getIntakeExtendPreset().onTrue(setIntakeDeployMode(IntakeDeployMode.DEPLOYING));
    controlScheme.getIntakeRetractPreset().onTrue(setIntakeDeployMode(IntakeDeployMode.RETRACTING));

    // ------------------------------------------------------------------------
    // TURRET ROTATION CONTROLS
    // -----------------------------------------------------------------------

    controlScheme.getTurretTrack().onTrue(setTurretMode(TurretMode.TRACKING_HUB));
    controlScheme.getTurretIdle().onTrue(setTurretMode(TurretMode.IDLE));
    controlScheme.zeroTurret().onTrue(turret.zeroTurretPositionCommand().ignoringDisable(true));
    controlScheme.autoZeroTurret().whileTrue(new ZeroTurret(turret));
  }

  private void configureTriggerCallbacks() {
    enteredAllianceZoneAuto
        .onTrue(
            setTurretMode(TurretMode.TRACKING_HUB)
                .onlyIf(() -> RobotState.instance().getTurretMode() != TurretMode.MANUAL))
        .onFalse(
            setTurretMode(TurretMode.IDLE)
                .onlyIf(() -> RobotState.instance().getTurretMode() != TurretMode.MANUAL));

    enteredNeutralZoneAuto
        .onTrue(
            setIntakeDeployMode(IntakeDeployMode.DEPLOYING)
                .andThen(setIntakeSpinMode(IntakeSpinMode.INTAKING))
                .andThen(setFlywheelMode(FlywheelMode.IDLE)))
        .onFalse(
            setIntakeSpinMode(IntakeSpinMode.IDLE).andThen(setFlywheelMode(FlywheelMode.PREPARE)));

    startFlopping
        .onTrue(setIntakeDeployMode(IntakeDeployMode.FLOPPING))
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (RobotState.instance().getIntakeDeployMode() == IntakeDeployMode.FLOPPING) {
                    RobotState.instance().setIntakeDeployMode(IntakeDeployMode.DEPLOYING);
                  }
                }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public static Command setIntakeDeployMode(IntakeDeployMode mode) {
    return Commands.runOnce(() -> RobotState.instance().setIntakeDeployMode(mode));
  }

  public static Command setIntakeSpinMode(IntakeSpinMode mode) {
    return Commands.runOnce(() -> RobotState.instance().setIntakeSpinMode(mode));
  }

  public static Command setFlywheelMode(FlywheelMode mode) {
    return Commands.runOnce(() -> RobotState.instance().setFlywheelMode(mode));
  }

  public static Command setTurretMode(TurretMode mode) {
    return Commands.runOnce(() -> RobotState.instance().setTurretMode(mode));
  }

  public static Command setClimbCameraMode(ClimbCameraMode mode) {
    return Commands.runOnce(() -> RobotState.instance().setClimbCameraMode(mode));
  }

  public static boolean isRed() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }
}

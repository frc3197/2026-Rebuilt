package frc.robot.auto;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignClimb;
import frc.robot.commands.AlignCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.enums.Modes.ClimbCameraMode;
import frc.robot.enums.Modes.FlywheelMode;
import frc.robot.enums.Modes.IntakeDeployMode;
import frc.robot.enums.Modes.TurretMode;
import frc.robot.enums.RealAutos;
import frc.robot.managersubsystems.RobotState;
import frc.robot.subsystems.Align;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import java.io.IOException;
import org.json.simple.parser.ParseException;

/*
 * AUTOS ---------------------
 * I have no good way of enforcing autonomous routine names. Therefore, ensure the names match here and double check regularly.
 */

public class AutoLookup {

  private final Align align;
  private final Climber climber;
  private final Drive drive;
  private final Turret turret;
  private final Vision vision;

  public AutoLookup(Align align, Climber climber, Drive drive, Turret turret, Vision vision) {
    this.align = align;
    this.climber = climber;
    this.drive = drive;
    this.turret = turret;
    this.vision = vision;
  }

  private Command getRightBumpAuto() {
    return new SequentialCommandGroup(
        getCommonCommands(),
        setRobotPoseWithFlipping(new Pose2d(3.612, 2.398, Rotation2d.kZero)),
        RobotContainer.setFlywheelMode(FlywheelMode.FRENZY),
        RobotContainer.setTurretMode(TurretMode.TRACKING_HUB),
        new WaitCommand(2.0),
        loadPath("Start-Neutral-Shoot"),
        RobotContainer.setFlywheelMode(FlywheelMode.FRENZY),
        new WaitCommand(3.75),
        RobotContainer.setFlywheelMode(FlywheelMode.IDLE),
        loadPath("Neutral-Shoot-2"),
        RobotContainer.setFlywheelMode(FlywheelMode.FRENZY),
        new WaitCommand(3.75),
        Commands.runOnce(
            () ->
                RobotState.instance()
                    .setClimbCameraMode(
                        RobotContainer.isRed()
                            ? ClimbCameraMode.CLIMB_RED
                            : ClimbCameraMode.CLIMB_BLUE)),
        loadPath("Shoot-Tower"),
        new AlignCommand(
            align,
            drive,
            (RobotContainer.isRed()
                ? FlippingUtil.flipFieldPose(FieldConstants.CLIMB_ALIGN_POSE)
                : FieldConstants.CLIMB_ALIGN_POSE)),
        RobotContainer.setFlywheelMode(FlywheelMode.IDLE),
        new AlignClimb(null, drive));
  }

  private Command getRightBumpCLIMBAuto() {
    return new SequentialCommandGroup(
        getCommonCommands(),
        setRobotPoseWithFlipping(new Pose2d(3.612, 2.398, Rotation2d.kZero)),
        RobotContainer.setFlywheelMode(FlywheelMode.FRENZY),
        RobotContainer.setTurretMode(TurretMode.TRACKING_HUB),
        new WaitCommand(2.0),
        loadPath("Start-Neutral-Shoot-Climb"),
        RobotContainer.setFlywheelMode(FlywheelMode.FRENZY),
        new WaitCommand(3.75),
        RobotContainer.setFlywheelMode(FlywheelMode.IDLE),
        loadPath("Hub-Climb-Right"),
        RobotContainer.setFlywheelMode(FlywheelMode.FRENZY),
        new WaitCommand(3.75),
        getAutoClimb(),
        RobotContainer.setFlywheelMode(FlywheelMode.IDLE));
  }

  public Command getAuto(RealAutos auto) {
    if (auto == RealAutos.Right_Bump_Twice) {
      return getRightBumpAuto();
    }
    if (auto == RealAutos.Right_Bump_Climb) {
      return getRightBumpCLIMBAuto();
    }
    return Commands.print("NO/INVALID AUTO COMMAND SELECTED: " + auto);
  }

  private Command setRobotPoseWithFlipping(Pose2d pose) {
    return Commands.runOnce(
        () -> drive.setPose(RobotContainer.isRed() ? FlippingUtil.flipFieldPose(pose) : pose),
        drive);
  }

  private SequentialCommandGroup getCommonCommands() {
    return new SequentialCommandGroup(
        turret.zeroTurretPositionCommand(),
        RobotContainer.setClimbCameraMode(ClimbCameraMode.APRIL_TAGS),
        RobotContainer.setFlywheelMode(FlywheelMode.FRENZY),
        RobotContainer.setIntakeDeployMode(IntakeDeployMode.DEPLOYING),
        RobotContainer.setTurretMode(TurretMode.TRACKING_HUB));
  }

  private Command loadPath(String name) {
    try {
      return AutoBuilder.followPath(PathPlannerPath.fromPathFile(name));
    } catch (FileVersionException e) {
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    } catch (ParseException e) {
      e.printStackTrace();
    }
    return Commands.print("INVALID PATH NAME: " + name);
  }

  private Command getAutoClimb() {
    return new SequentialCommandGroup(
        Commands.waitUntil(RobotState.instance().getIntakeFullyRetractedSupplier()),
        RobotContainer.setClimbCameraMode(
            RobotContainer.isRed() ? ClimbCameraMode.CLIMB_RED : ClimbCameraMode.CLIMB_BLUE),
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
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0, 0.4, 0)), drive).withTimeout(1.5),
        Commands.runOnce(
            () ->
                climber.setClimberControlType(
                    new PositionDutyCycle(ClimberConstants.climberClimbAngle)),
            climber),
        RobotContainer.setClimbCameraMode(ClimbCameraMode.APRIL_TAGS));
  }
}

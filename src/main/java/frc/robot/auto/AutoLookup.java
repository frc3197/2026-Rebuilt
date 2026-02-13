package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.enums.Modes.FlywheelMode;
import frc.robot.enums.Modes.IntakeDeployMode;
import frc.robot.enums.Modes.TurretMode;
import frc.robot.enums.RealAutos;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.turret.Turret;
import java.io.IOException;
import org.json.simple.parser.ParseException;

/*
 * AUTOS ---------------------
 * I have no good way of enforcing autonomous routine names. Therefore, ensure the names match here and double check regularly.
 */

public class AutoLookup {

  private final Drive drive;
  private final Turret turret;

  public AutoLookup(Drive drive, Turret turret) {
    this.drive = drive;
    this.turret = turret;
  }

  private Command getRightBumpAuto() {
    return new SequentialCommandGroup(
        getCommonCommands(),
        setRobotPoseWithFlipping(new Pose2d(4.168, 2.398, Rotation2d.kZero)),
        RobotContainer.setFlywheelMode(FlywheelMode.FRENZY),
        new WaitCommand(0.5),
        loadPath("Start-Neutral-Shoot"),
        RobotContainer.setFlywheelMode(FlywheelMode.FRENZY),
        new WaitCommand(5.0),
        RobotContainer.setFlywheelMode(FlywheelMode.IDLE),
        loadPath("Neutral-Shoot-2"),
        RobotContainer.setFlywheelMode(FlywheelMode.FRENZY),
        new WaitCommand(5.0),
        loadPath("Shoot-Tower"),
        RobotContainer.setFlywheelMode(FlywheelMode.IDLE));
  }

  public Command getAuto(RealAutos auto) {
    if (auto == RealAutos.Right_Bump) {
      return getRightBumpAuto();
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
        RobotContainer.setFlywheelMode(FlywheelMode.IDLE),
        RobotContainer.setIntakeDeployMode(IntakeDeployMode.IDLE_RETRACTED),
        RobotContainer.setTurretMode(TurretMode.IDLE));
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
}

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
import frc.robot.enums.RealAutos;
import frc.robot.subsystems.drive.Drive;
import java.io.IOException;
import org.json.simple.parser.ParseException;

/*
 * AUTOS ---------------------
 * I have no good way of enforcing autonomous routine names. Therefore, ensure the names match here and double check regularly.
 */

public class AutoLookup {

  private final Drive drive;

  public AutoLookup(Drive drive) {
    this.drive = drive;
  }

  private Command getRightBumpAuto() {
    return new SequentialCommandGroup(
        setRobotPoseWithFlipping(new Pose2d(4.168, 2.398, Rotation2d.kZero)),
        new WaitCommand(0.2),
        loadPath("Start-Neutral-Shoot"));
  }

  public Command getAuto(RealAutos auto) {
    if (auto == RealAutos.Right_Bump) {
      return getRightBumpAuto();
    }
    return Commands.print("NO/INVALID AUTO COMMAND SELECTED");
  }

  private Command setRobotPoseWithFlipping(Pose2d pose) {
    return Commands.runOnce(
        () -> drive.setPose(RobotContainer.isRed() ? FlippingUtil.flipFieldPose(pose) : pose),
        drive);
  }

  private Pose2d getStartingPose(String name) {
    try {
      return PathPlannerPath.fromPathFile(name).getStartingDifferentialPose();
    } catch (FileVersionException e) {
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    } catch (ParseException e) {
      e.printStackTrace();
    }
    return Pose2d.kZero;
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

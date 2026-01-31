package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.managersubsystems.RobotState;
import frc.robot.util.FieldConstants;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator extends VirtualSubsystem {

    private String key;
    public static ShotCalculator instance;

    private Angle targetHoodAngle = Degrees.of(45.0);
    private Angle targetTurretAngle = Degrees.of(0.0);
    private AngularVelocity targetRPM = RotationsPerSecond.of(220);

    public ShotCalculator(String key) {
        this.key = key;
    }

    public static ShotCalculator instance() {
        if (instance == null) {
            instance = new ShotCalculator("calc");
        }
        return instance;
    }

    @Override
    public void periodic() {
        calculateTargetParameters();
        log();
    }

    private void log() {
        Logger.recordOutput(
                "Shot Calculator/Turret Field Position",
                RobotState.instance()
                        .getRobotPose3d()
                        .plus(
                                ShooterConstants.ROBOT_TO_TURRET_CENTER.plus(
                                        new Transform3d(
                                                0, 0, 0, new Rotation3d(0, 0, targetTurretAngle.in(Radians))))));

        Logger.recordOutput("Shot Calculator/Robot to hub angle", targetTurretAngle);
    }

    private void calculateTargetParameters() {
        Pose2d robotPose = RobotState.instance().getRobotPose();
        ChassisSpeeds robotVelocity = RobotState.instance().getRobotVelocity();
        ChassisSpeeds robotAcceleration = RobotState.instance().getRobotAcceleration();

        Vector<N2> vRobot = VecBuilder.fill(robotPose.getX(), robotPose.getY());
        Vector<N2> vHub = VecBuilder.fill(FieldConstants.Red.HUB_CENTER.getX(), FieldConstants.Red.HUB_CENTER.getY());

        Vector<N2> robotToHub = vRobot.minus(vHub);

        // targetTurretAngle =
        Angle sinAngle = Radians.of(
                Math.acos(
                        robotToHub.dot(VecBuilder.fill(0, 1))
                                / (1
                                        * Math.sqrt(
                                                Math.pow(robotToHub.get(0), 2) + Math.pow(robotToHub.get(1), 2)))))
                .minus(Degrees.of(90.0).plus(Radians.of(robotPose.getRotation().getRadians())));
    }

    // Getters --------------------------------------------
    public Angle getTargetHoodAngle() {
        return targetHoodAngle;
    }

    public Angle getTargetTurretAngle() {
        return targetTurretAngle;
    }
}

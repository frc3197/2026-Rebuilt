// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LoggingConstants;
import frc.robot.enums.Modes.IntakeSpinMode;
import frc.robot.managersubsystems.RobotState;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.IndexConstants;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultIndexCommand extends Command {

  private final Index index;
  private final Timer autoBackfeedTimer = new Timer();

  private LoggedTunableNumber feedKS =
      new LoggedTunableNumber("FEED KS", IndexConstants.Feed_SLOT0_CONFIGS.kS);
  private LoggedTunableNumber feedKA =
      new LoggedTunableNumber("FEED KA", IndexConstants.Feed_SLOT0_CONFIGS.kA);
  private LoggedTunableNumber feedKP =
      new LoggedTunableNumber("FEED KP", IndexConstants.Feed_SLOT0_CONFIGS.kP);
  private LoggedTunableNumber feedKV =
      new LoggedTunableNumber("FEED KV", IndexConstants.Feed_SLOT0_CONFIGS.kV);
  private LoggedTunableNumber feedKD =
      new LoggedTunableNumber("FEED KD", IndexConstants.Feed_SLOT0_CONFIGS.kD);

  private final BooleanSupplier backfeedManual;
  private final BooleanSupplier forwardfeedManual;

  /**
   * Creates a new DefaultfeedCommand.
   *
   * @param intake The intake subsystem.
   */
  public DefaultIndexCommand(
      Index index, BooleanSupplier backfeedManual, BooleanSupplier forwardfeedManual) {
    this.index = index;
    addRequirements(index);
    autoBackfeedTimer.start();
    this.backfeedManual = backfeedManual;
    this.forwardfeedManual = forwardfeedManual;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if (LoggingConstants.tuningMode
            && (feedKA.hasChanged(hashCode()) || feedKD.hasChanged(hashCode()))
        || feedKP.hasChanged(hashCode())
        || feedKS.hasChanged(hashCode())
        || feedKV.hasChanged(hashCode())) {
      Slot0Configs newConfigs = new Slot0Configs();
      newConfigs.kP = feedKP.getAsDouble();
      newConfigs.kA = feedKA.getAsDouble();
      newConfigs.kD = feedKD.getAsDouble();
      newConfigs.kS = feedKS.getAsDouble();
      newConfigs.kV = feedKV.getAsDouble();
      index.setGains(newConfigs);
    }

    if (RobotState.instance().getIntakeSpinMode() == IntakeSpinMode.OUTTAKING) {
      index.setFeedMotor(Volts.of(DriverStation.isAutonomous() ? 4 : -4));
      index.setSpindexMotor(Volts.of(DriverStation.isAutonomous() ? 5.35 : -5.35));
    }

    if (backfeedManual.getAsBoolean()) {
      index.setFeedMotor(Volts.of(-3.5));
      index.setSpindexMotor(Volts.of(-3.5));
      return;
    }

    if (forwardfeedManual.getAsBoolean()) {
      index.setFeedRequest(
          IndexConstants.FEED_TORQUE_REQUEST.withVelocity(RotationsPerSecond.of(45)));
      index.setSpindexMotor(Volts.of(3.5));
      return;
    }

    switch (RobotState.instance().getFlywheelMode()) {
      case IDLE:
        index.setFeedMotor(Volts.of(0.0));
        index.setSpindexMotor(Volts.of(0.0));
        break;
      case PREPARE:
        index.setFeedMotor(Volts.of(0.0));
        index.setSpindexMotor(Volts.of(0.0));
        break;
      case FRENZY:
        if (ShotCalculator.instance().getReadyToFeed()) {
          index.setFeedRequest(
              IndexConstants.FEED_TORQUE_REQUEST.withVelocity(RotationsPerSecond.of(45)));
          if ((DriverStation.isAutonomous() && autoBackfeedTimer.get() % 2 < 0.3)) {
            index.setSpindexMotor(Volts.of(-3.5));
          } else {
            // 3.5
            index.setSpindexMotor(Volts.of(3.5));
          }
        } else {
          index.setFeedMotor(Volts.of(0.0));
          index.setSpindexMotor(Volts.of(0.0));
        }
        break;
      case SHOOTING:
        if (ShotCalculator.instance().getReadyToFeed()) {
          index.setSpindexMotor(Volts.of(3.5));
          // index.setFeedMotor(Volts.of(10.0));
          index.setFeedRequest(
              IndexConstants.FEED_TORQUE_REQUEST.withVelocity(RotationsPerSecond.of(45)));
        } else {
          index.setFeedMotor(Volts.of(0.0));
          index.setSpindexMotor(Volts.of(0.0));
        }
        break;
      default:
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {}
}

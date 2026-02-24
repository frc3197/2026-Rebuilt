// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LoggingConstants;
import frc.robot.managersubsystems.RobotState;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.IndexConstants;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.LoggedTunableNumber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultIndexCommand extends Command {

  private final Index index;

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

  /**
   * Creates a new DefaultfeedCommand.
   *
   * @param intake The intake subsystem.
   */
  public DefaultIndexCommand(Index index) {
    this.index = index;
    addRequirements(index);
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

    switch (RobotState.instance().getFlywheelMode()) {
      case IDLE:
        index.setFeedMotor(Volts.of(0.0));
        index.setSpindexMotor(Volts.of(0.0));
        break;
      case FRENZY:
        if (ShotCalculator.instance().getReadyToFeed()) {
          index.setSpindexMotor(Volts.of(2.95));
          index.setFeedMotor(Volts.of(10.0));
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

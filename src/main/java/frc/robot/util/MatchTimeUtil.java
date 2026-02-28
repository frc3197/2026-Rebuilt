// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;

public class MatchTimeUtil extends VirtualSubsystem {

    private static MatchTimeUtil instance;
    private final String key;

    private Optional<Boolean> wonAuto = Optional.empty();
    private boolean isAuto = false;

    private MatchTimeUtil(String key) {
        this.key = key;
    }

    public static MatchTimeUtil instance() {
        if (instance == null) {
            instance = new MatchTimeUtil("measured");
        }
        return instance;
    }

    @Override
    public void periodic() {
        double time = DriverStation.getMatchTime();
        isAuto = DriverStation.isAutonomous();

        String gameData;
        gameData = DriverStation.getGameSpecificMessage();
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    wonAuto = Optional.of(!RobotContainer.isRed());
                    break;
                case 'R':
                    wonAuto = Optional.of(RobotContainer.isRed());
                    break;
                default:
                    wonAuto = Optional.empty();
                    DriverStation.reportError("INVALID AUTO WINNER", false);
                    break;
            }
        } else {
            wonAuto = Optional.empty();
        }


    }

    public Optional<Boolean> wonAuto() {
        return wonAuto;
    }

}
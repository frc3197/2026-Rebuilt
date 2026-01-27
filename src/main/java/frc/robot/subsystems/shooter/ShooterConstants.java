package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.HardwareID;

public class ShooterConstants implements HardwareID.ShooterHardwareID {
    
    public final static PIDController FLYWHEEL_PID_CONTROLLER = new PIDController(0.0, 0.0, 0.0);

}

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControlScheme {

    private final CommandXboxController primaryController;
    private final CommandXboxController secondaryController;

    public ControlScheme(CommandXboxController pC, CommandXboxController sC) {
        this.primaryController = pC;
        this.secondaryController = sC;
    }

    
}

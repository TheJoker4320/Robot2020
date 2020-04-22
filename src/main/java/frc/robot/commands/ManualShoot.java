package frc.robot.commands;


import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ManualShoot extends CommandBase {
    private final Shooter shooter;
    public BooleanSupplier toggle;

    public ManualShoot(Shooter shooter, BooleanSupplier toggle) {
        this.toggle = toggle;
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {        
        if(toggle.getAsBoolean())
            shooter.setState(!shooter.isActive());
        
        shooter.setOutput(Constants.Robot.Shooter.THREE_METERS_PERCENTAGE);
    }

    @Override
    public void end(boolean interrupted) {
        this.shooter.disable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

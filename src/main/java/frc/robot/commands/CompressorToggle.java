package frc.robot.commands;


import frc.robot.Constants;
import frc.robot.subsystems.CompressorWrapper;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class CompressorToggle extends CommandBase {
    // @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final CompressorWrapper compressorWrapper;
    public BooleanSupplier toggle;

    public CompressorToggle(CompressorWrapper compressorWrapper, BooleanSupplier toggle) {
        this.toggle = toggle;
        this.compressorWrapper = compressorWrapper;
        addRequirements(compressorWrapper);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //m_subsystem.active = toggle.getAsBoolean();
        
        if(toggle.getAsBoolean())
            compressorWrapper.setState(!compressorWrapper.isActive());;
        
        if (compressorWrapper.isActive())
            compressorWrapper.setState(true);
        else
            compressorWrapper.setState(false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        compressorWrapper.setState(false);
        compressorWrapper.activate(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

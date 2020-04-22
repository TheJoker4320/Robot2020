package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shifters;

public class ShiftPowerMode extends CommandBase {
  private Shifters shifters;
  private BooleanSupplier toggle;

  
  public ShiftPowerMode(Shifters shifters, BooleanSupplier toggle) {
    this.shifters = shifters;
    this.toggle = toggle;
    addRequirements(shifters);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    shifters.setMode(toggle.getAsBoolean());
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

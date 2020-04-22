package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeLifter;

public class IntakeLifterToggle extends CommandBase {
  private IntakeLifter intakeLifter;
    private BooleanSupplier toggle;

  public IntakeLifterToggle(IntakeLifter intakeLifter, BooleanSupplier toggle) {
      this.intakeLifter = intakeLifter;
      this.toggle = toggle;
    addRequirements(intakeLifter);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
      if(toggle.getAsBoolean()) 
          intakeLifter.setState(!intakeLifter.isActive());
      
      if(intakeLifter.isActive())
          intakeLifter.setSolenoidMode();
      else
        intakeLifter.setSolenoidMode();
      
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

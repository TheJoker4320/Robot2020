package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Crank;

public class MoveCrank extends CommandBase {
  private Crank crank;
  private BooleanSupplier down;

  public MoveCrank(Crank crank, BooleanSupplier down) {
      this.down = down;
      this.crank = crank;
    addRequirements(crank);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (down.getAsBoolean())
        this.crank.setOutput(Constants.ClimberConstants.CRANK_PERCENTAGE);
    else
        this.crank.setOutput(0);  
    }

  @Override
  public void end(boolean interrupted) {
    crank.setOutput(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

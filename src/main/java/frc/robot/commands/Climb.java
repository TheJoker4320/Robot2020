package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {
  private Climber climber;
  private double climbSpeed = 0;
  private BooleanSupplier up, down;

  public Climb(Climber climber, BooleanSupplier up, BooleanSupplier down) {
    this.up = up;
    this.down = down;
    this.climber = climber;
    addRequirements(climber);
    }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    climbSpeed = 0;

    if (up.getAsBoolean())
      climbSpeed = -ClimberConstants.CLIMB_PERCENTAGE;
    else if (down.getAsBoolean())
      climbSpeed = ClimberConstants.CLIMB_PERCENTAGE;
    climber.setOutput(climbSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    climber.setOutput(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

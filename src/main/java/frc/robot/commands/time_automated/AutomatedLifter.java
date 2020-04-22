package frc.robot.commands.time_automated;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeLifter;

public class AutomatedLifter extends CommandBase {
    private IntakeLifter intakeLifter;
    private boolean state;
    private boolean finished = false;
    private Timer timer = new Timer();

  public AutomatedLifter(IntakeLifter intakeLifter, boolean state) {
    this.state = state;
      this.intakeLifter = intakeLifter;
    addRequirements(intakeLifter);
  }

  @Override
  public void initialize() {
    timer.reset();
  }

  @Override
  public void execute() {
      intakeLifter.setState(state);
      intakeLifter.setSolenoidMode();
      finished = true;
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}

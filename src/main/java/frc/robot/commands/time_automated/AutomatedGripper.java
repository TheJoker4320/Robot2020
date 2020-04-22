package frc.robot.commands.time_automated;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeGripper;

public class AutomatedGripper extends CommandBase {
  private IntakeGripper intakeGripper;
  private boolean work;
  private boolean inward;
  private Timer timer;
  private double time;

  
  public AutomatedGripper(IntakeGripper intakeGripper, boolean work, boolean inward, double duration) {
    this.intakeGripper = intakeGripper;
    this.work = work;
    this.inward = inward;
    this.time = duration;
    this.timer = new Timer();
    addRequirements(intakeGripper);
  }

  @Override
  public void initialize() {
//this.timer.  
}

  @Override
  public void execute() {
    if(work){
        if(inward)
            this.intakeGripper.setOutput(Constants.IntakeConstants.GRIPPER_VOLTAGE);
        else
            this.intakeGripper.setOutput(-Constants.IntakeConstants.GRIPPER_VOLTAGE);
    }
    else{
        this.intakeGripper.setOutput(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.intakeGripper.setOutput(0);
  }

  @Override
  public boolean isFinished() {
    return timer.get() > time;
}
}

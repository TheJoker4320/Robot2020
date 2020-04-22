package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeGripper;

public class GripperToggle extends CommandBase {
  private IntakeGripper intakeGripper;
  private BooleanSupplier toggle;
  private DoubleSupplier trigger;

  
  public GripperToggle(IntakeGripper intakeGripper, BooleanSupplier toggle, DoubleSupplier trigger) {
    this.intakeGripper = intakeGripper;
    this.toggle = toggle;
    this.trigger = trigger;
    addRequirements(intakeGripper);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(toggle.getAsBoolean())
      intakeGripper.blocked  = !intakeGripper.blocked;

    if(!intakeGripper.blocked){
      if(trigger.getAsDouble() >= 0.2)
        intakeGripper.setState(IntakeGripper.States.BACKWARD);
      else
        intakeGripper.setState(IntakeGripper.States.FORWARD);
      intakeGripper.setOutput(Constants.IntakeConstants.GRIPPER_PERCENTAGE);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakeGripper.disable();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

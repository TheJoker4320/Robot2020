package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeGripper;
import frc.robot.subsystems.MainConveyer;


public class EmergencyConveyer extends CommandBase {
    private MainConveyer mainConveyer;
    private IntakeGripper intakeGripper;
    private Timer timer;
    private double time = 0;
    private boolean work;

    public EmergencyConveyer(MainConveyer conveyer, IntakeGripper gripper, boolean work, double activeTime) {
        this.mainConveyer = conveyer;
        this.intakeGripper = gripper;
        this.time = activeTime;
        this.timer = new Timer();
        this.work = work;
        addRequirements(conveyer);
    }

    @Override
    public void initialize() {
        
        this.timer.reset();
        this.timer.start();
    }

    @Override
    public void execute() {
        if(work){
            this.mainConveyer.setState(MainConveyer.States.BACKWARD);
            this.mainConveyer.setOutput(Constants.Robot.MainConveyer.ACTIVATION_SPEED);
            this.intakeGripper.setState(IntakeGripper.States.BACKWARD);
            this.intakeGripper.setOutput(Constants.IntakeConstants.GRIPPER_PERCENTAGE);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.mainConveyer.disable();
        this.intakeGripper.disable();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

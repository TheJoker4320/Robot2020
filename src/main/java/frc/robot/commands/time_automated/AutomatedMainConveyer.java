package frc.robot.commands.time_automated;

import frc.robot.Constants;
import frc.robot.subsystems.MainConveyer;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutomatedMainConveyer extends CommandBase {
    private MainConveyer instance;
    private Timer timer;
    private double time = 0;
    private boolean work;

    public AutomatedMainConveyer(MainConveyer conveyer, boolean work, double activeTime) {
        this.instance = instance;
        this.time = activeTime;
        this.timer = new Timer();
        instance = conveyer;
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
            this.instance.setOutput(Constants.Robot.MainConveyer.ACTIVATION_SPEED);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.instance.setOutput(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get() > time;
    }
}

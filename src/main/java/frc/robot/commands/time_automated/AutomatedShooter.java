package frc.robot.commands.time_automated;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;


public class AutomatedShooter extends CommandBase {
    // @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Shooter shooter;
    private boolean work;
    private double time;
    private Timer timer;

    public AutomatedShooter(Shooter shooter, boolean work, double time) {
        this.work = work;
        this.time = time;
        this.shooter = shooter;
        this.timer = new Timer();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        this.timer.reset();
        this.timer.start();
    }

    @Override
    public void execute() {
        if(work){
            this.shooter.setOutput(Constants.Robot.Shooter.ACTIVATION_VOLTS);
        }
        else{
            this.shooter.setOutput(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.shooter.setOutput(0);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > time;
    }
}

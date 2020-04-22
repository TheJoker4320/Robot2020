package frc.robot.commands.time_automated;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class AutomatedClimber extends CommandBase {
    private Climber climber;
    private Timer timer;
    private boolean work;
    private boolean up;
    private double time;

    public AutomatedClimber(Climber climber, boolean work, boolean up, double time) {
        this.time = time;
        this.climber = climber;
        this.up = up;
        this.work = work;
        this.timer = new Timer();
        this.work = work;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //if(!climber.active){
            if(work){
                if(up)
                    this.climber.setOutput(ClimberConstants.CLIMB_SPEED);
                else
                    this.climber.setOutput(-ClimberConstants.CLIMB_SPEED);
            }
            else{
                this.climber.setOutput(0);
            }
        //}
    }

    @Override
    public void end(boolean interrupted) {
        climber.setOutput(0);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > time;
    }
}

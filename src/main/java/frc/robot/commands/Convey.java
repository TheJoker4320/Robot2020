package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.MainConveyer;
import frc.robot.subsystems.MainConveyer.States;


public class Convey extends CommandBase {
    private final MainConveyer mainConveyer;
    public BooleanSupplier inside, outside;
    public double lastDistance;


    public Convey(MainConveyer conveyer, BooleanSupplier inside, BooleanSupplier outside) {
        this.inside = inside;
        this.outside = outside;
        mainConveyer = conveyer;
        addRequirements(conveyer);
    }

    @Override
    public void initialize() {
        mainConveyer.sonic.setAutomaticMode(true);
        lastDistance = mainConveyer.sonic.getRangeMM();
    }

    @Override
    public void execute() {
        if(inside.getAsBoolean())
            mainConveyer.setState(States.FORWARD);
        else if (outside.getAsBoolean())
            mainConveyer.setState(States.BACKWARD);
        else
            mainConveyer.setState(States.IDLE);
        
        boolean permission = !inside.getAsBoolean() && !outside.getAsBoolean();

        double currentDistance = mainConveyer.sonic.getRangeMM();
        if(permission){
            if(mainConveyer.shooterMode){
                if(currentDistance<lastDistance-20){
                    mainConveyer.setState(MainConveyer.States.FORWARD);
                }
                else{
                    mainConveyer.setState(MainConveyer.States.IDLE);
                }
            }
            else{
                if(currentDistance<lastDistance-20){
                    mainConveyer.setState(MainConveyer.States.IDLE);
                }
                else{
                    mainConveyer.setState(MainConveyer.States.FORWARD);
                }
            }
        }

        mainConveyer.setOutput(Constants.Robot.MainConveyer.ACTIVATION_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        mainConveyer.disable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

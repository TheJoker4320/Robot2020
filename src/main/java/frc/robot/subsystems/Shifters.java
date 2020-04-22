package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Solenoid;

public class Shifters extends SubsystemBase {
    private static Shifters instance;
    private Solenoid solenoid;
    boolean state = true;

    public void activateConveyer(){
        state = true;
    }

    public void deactivateConveyer(){
        state = false;
    }

    public boolean getState(){
        return state;
    }

    public void toggleState(){
        state = !state;
    }

    public static Shifters getInstance() {
        if (instance == null)
            instance = new Shifters();
        return instance;
    }

    private Shifters() {
        this.solenoid = new Solenoid(Constants.Robot.SHIFTER_SELENOID_PORT);
    }

    public void setMode(boolean value){
        this.solenoid.set(!value);
    }

    @Override
    public void periodic() {
        //System.out.println("Main conveyer placement: "+distanceInMeters());
        // This method will be called once per scheduler run
    }
}

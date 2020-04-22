package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MainConveyer extends SubsystemBase implements JokerSubsystem{
    private static MainConveyer instance;
    private WPI_TalonSRX motor;
    public Ultrasonic sonic;
    public static enum States{
        BACKWARD,
        IDLE,
        FORWARD
      };
    private States active = States.IDLE;
    public boolean blocked = true;
    public boolean shooterMode = false;

    public static MainConveyer getInstance() {
        if (instance == null)
            instance = new MainConveyer();
        return instance;
    }

    private MainConveyer() {
        this.sonic = new Ultrasonic(1, 2);
        this.sonic.setAutomaticMode(true);
        this.motor = new WPI_TalonSRX(Constants.Ports.MAIN_CONVEYER);
        this.motor.configFactoryDefault();
    }

    public void setOutput(double percentage) {
        if(active == States.IDLE)
            motor.set(0);
        if(active == States.FORWARD)
            motor.set(percentage);
        if(active == States.BACKWARD)
            motor.set(-percentage); 
    }

    public void setState(States state){
        this.active = state;
    }
    
    public States getState(){
      return this.active;
    }

    @Override
    public void periodic() {
    }

    @Override
    public boolean isActive() {
        return this.active.ordinal() != 0;
    }

    @Override
    public void disable() {
        this.active = States.IDLE;
        motor.set(0);
    }
}

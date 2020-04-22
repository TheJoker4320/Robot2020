/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase implements JokerSubsystem{
    private static Shooter instance;
    WPI_TalonSRX master;
    WPI_VictorSPX slave;
    
    private boolean active = false;

    public static Shooter getInstance() {
        if (instance == null)
            instance = new Shooter();
        return instance;
    }

    private Shooter() {
        this.master = new WPI_TalonSRX(Constants.Ports.SHOOTER_MASTER);
        this.master.configFactoryDefault();

        this.slave = new WPI_VictorSPX(Constants.Ports.SHOOTER_SLAVE);
        this.slave.configFactoryDefault();
        this.slave.follow(master);
        this.master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        // this.master.setSensorPhase(true);
    }

    public void setState(boolean state){
        active = state;
    }

    public void setOutput(double percentage) {
        if(active)
            this.master.set(percentage);
        else
            this.master.set(0);
    }

    public void setVoltage(double volts){
        if(volts> Constants.Robot.Shooter.MAX_VOLTAGE){
            master.set(1);
        }
        else if(volts< -Constants.Robot.Shooter.MAX_VOLTAGE){
            master.set(-1);
        }
        else{
            master.set(volts/Constants.Robot.Shooter.MAX_VOLTAGE);
        }
    }

    public double distanceInMeters() {
        //System.out.println("Shooter encoder: "+master.getSelectedSensorPosition());
        return Constants.Robot.Shooter.DIAMETER * this.master.getSelectedSensorPosition();
               // / Constants.Robot.Shooter.ENCODER_CPR;
    }

    public double velocityInMeters(){
        //System.out.println("Shooter encoder: "+master.getSelectedSensorPosition());
        //System.out.println("Shooter encoder velocity: "+master.getSelectedSensorVelocity());
        return 10 * Constants.Robot.Shooter.DIAMETER * this.master.getSelectedSensorVelocity() / Constants.Robot.Shooter.ENCODER_CPR;
    }

    public void resetDistance(){
        this.master.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        //System.out.println(velocityInMeters());
        //System.out.println("Shooter encoder: "+master.getSelectedSensorPosition());
        System.out.println("Shooter speed in m/s: " + velocityInMeters());
    }

    @Override
    public boolean isActive() {
        return active;
    }

    @Override
    public void disable() {
        active = false;
        master.set(0);
    }
}

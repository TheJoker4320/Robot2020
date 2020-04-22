package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PowerCellCounter extends SubsystemBase {
    private static PowerCellCounter instance;
    public static int powerCellsConveyor = 0;
    public static int powerCellsToShooter = 0;
    public Ultrasonic mainConveyorSonar;
    public Ultrasonic shooterConveyorSonar;
    //public Ultrasonic turretSonar;
    private double conveyorLast = 0;
    private double shooterLast = 0;

    public void start(){
        this.mainConveyorSonar.setAutomaticMode(true);
        this.shooterConveyorSonar.setAutomaticMode(true);
        this.conveyorLast = this.mainConveyorSonar.getRangeMM();
        this.shooterLast = this.mainConveyorSonar.getRangeMM();
    }

    public static PowerCellCounter getInstance() {
        if (instance == null)
            instance = new PowerCellCounter();
        return instance;
    }

    private PowerCellCounter() {
        this.mainConveyorSonar = new Ultrasonic(2, 3);
        this.shooterConveyorSonar = new Ultrasonic(4, 5);
        //this.turretSonar = new Ultrasonic(6, 7);

    }

    public boolean IsReadyForConveyer(){
        boolean answer = false;
        double temp = mainConveyorSonar.getRangeMM();
        //System.out.println(temp);
        if (conveyorLast-temp>= Constants.Robot.Sonars.CONVEYER_ENTRY_PLACEMENT)
            answer = true;
        conveyorLast = temp;
        return answer;
    }

    public boolean passedThroughConveyorEntry() {
        boolean answer = false;
        double temp = mainConveyorSonar.getRangeMM();
        //System.out.println(temp);
        if (temp- conveyorLast>= Constants.Robot.Sonars.CONVEYER_ENTRY_PLACEMENT)
            answer = true;
        conveyorLast = temp;
        return answer;
    }

    public boolean passedThroughConveyorExit() {
        boolean answer = false;
        double temp = shooterConveyorSonar.getRangeMM();
        if (shooterLast - temp >= Constants.Robot.Sonars.SHOOTER_CONVEYOR_PLACEMENT)
            answer = true;
        shooterLast = temp;
        return answer;
    }
    
/*
    public boolean passedThroughShooter() {
        boolean answer = false;
        double temp = turretSonar.getRangeMM();
        System.out.println(temp - turretLast);
        if (temp- turretLast >= Constants.Robot.Sonars.TURRET_PLACEMENT){
            System.out.println("Passsssssed");
            answer = true;
        }
        turretLast = temp;
        return answer;
    }
*/
    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        if (passedThroughConveyorEntry() && powerCellsConveyor < 3) {
            powerCellsConveyor += 1;
        }
        
        
        if (passedThroughConveyorExit() && powerCellsToShooter < 1) {
            powerCellsConveyor -= 1;
            powerCellsToShooter += 1;
        }
        
        // && powerCellsToShooter>0
        // if(passedThroughShooter()){
        //     powerCellsToShooter -= 1;
        // }
        System.out.println(mainConveyorSonar.getRangeMM());
        //System.out.println("Cells at Intake: "+powerCellsIntake);
        System.out.println("Cells at Conveyor Entry: "+powerCellsConveyor);
        System.out.println("Cells at Shooter Conveyor: "+powerCellsToShooter);
    }

    public void reset(){
        powerCellsConveyor = 0;
        powerCellsToShooter = 0;

        conveyorLast = 0;
        shooterLast = 0;
    }
}

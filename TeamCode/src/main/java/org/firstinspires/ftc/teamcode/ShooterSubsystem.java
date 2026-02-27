package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;



import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;


public class ShooterSubsystem   {

@Config
    public static class ShooterConfigurable{
        public static double  shooter_velocity = 1500;
        public static double kp = 0.006;
        public static double ki = 0;
        public static double kd = 0.0;


        public static double kv = 0.000356;
        public static double ka = 0;
        public static double ks = 0.033;

        public static boolean enabledPid = true;



    }
    private final DcMotorEx shooter;
    private static final double SLOPE = 2;
    private static final double Y_INTERCEPT = 1100.0;

    // Optional: Max RPM to prevent motor damage
    private static final double MAX_RPM = 2700.0;

    private boolean enablePid  = false;


    private ControlSystem controller;

    public ShooterSubsystem(HardwareMap hw){
        this.shooter = hw.get(DcMotorEx.class,"shooter");

        //launchMotora = robot.launchMotors;



    }
    public void init(){
        updatePIDCoeffients();
        //stopShooter();
    }

    public void updatePIDCoeffients(){
        controller = ControlSystem.builder()
                .basicFF(ShooterConfigurable.kv, ShooterConfigurable.ka, ShooterConfigurable.ks)
                .velPid(ShooterConfigurable.kp, ShooterConfigurable.ki, ShooterConfigurable.kd)
                .build();
    }

    //this will implement with the shooter pid
    public boolean getshoot(){
        return enablePid;
    }

    public boolean readyToShoot(){
        return getShooterVelocity() >= getShooterTargetVelocity() - 10;
    }
    public void shoot(boolean onOff){


            enablePid = onOff;

    }

    public void setShooterVelocity(double velocity){
        ShooterConfigurable.shooter_velocity = velocity;
       /// controller.setGoal(new KineticState(0.0,velocity));
    }

    public void periodic() {
//           controller.setGoal(new KineticState(0.0, ShooterConfigurable.shooter_velocity));
//             launchMotora.set(controller.calculate(new KineticState(0.0, -robot.launcherEncoder.getCorrectedVelocity())));


       updateShooter();
    }

    public void stopShooter(){
        ShooterConfigurable.shooter_velocity = 0;
    }

    public void updateShooter(){
       // updatePIDCoeffients();

        if(enablePid){
            controller.setGoal(new KineticState(0.0,getShooterTargetVelocity()));
            //setShooterVelocity(shooter.getTargetVelocity(limelight.getDistanceInches()));

            shooter.setPower(controller.calculate(new KineticState(0.0, -shooter.getVelocity())));
        } else{
            shooter.setPower(.5);
        }


    }


    // get shooter current Velocity
    public double getShooterVelocity(){
        return shooter.getVelocity();
    }
    public double getShooterTargetVelocity(){
        return ShooterConfigurable.shooter_velocity;
    }






    /**
     * Calculates the necessary RPM based on distance from target.
     * @param x Distance from the goal (via Odometry or Camera)
     * @return Target RPM for the shooter motor
     */
    public double getTargetVelocity(double x) {

            return Math.min(0.0000148438 * Math.pow(x, 4)
                    - 0.00555804  * Math.pow(x, 3)
                    + 0.763119    * Math.pow(x, 2)
                    - 41.38717    * x
                    + 1904.65257, 2000);
        }




}
package org.firstinspires.ftc.teamcode.tuning;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ShooterSubsystem;


@TeleOp(name = "shooter tuner")
public class shooterTuner extends OpMode {

   private  ShooterSubsystem shooter;
    ElapsedTime loopTimer = new ElapsedTime();


    @Override
    public void init() {

        //robot = new Robot(hardwareMap);
       // robot.initHardware();
       shooter = new ShooterSubsystem(hardwareMap);
       shooter.init();
       shooter.shoot(true);
    }

    @Override
    public void loop() {

        double loopTimeMs = loopTimer.milliseconds();
        loopTimer.reset(); // Reset for the next loop
       // shooter.setShooterVelocity(1500);
        shooter.updateShooter();



        telemetry.addData("Shooter Vel", shooter.getShooterVelocity());
       // telemetry.addData("Shooter current pos", shooter.getShooterCurrentPosition());
        telemetry.addData("Shooter Target ticks", shooter.getShooterTargetVelocity());
        telemetry.addData("loopTimes", loopTimeMs);

        telemetry.update();
    }
}

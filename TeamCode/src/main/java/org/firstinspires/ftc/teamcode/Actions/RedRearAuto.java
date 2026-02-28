package org.firstinspires.ftc.teamcode.Actions;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ShooterSubsystem;

@Autonomous(name="RedRearAuto")
public class RedRearAuto extends LinearOpMode {

    Transfer transfer;
    Intake intake;
    Shooter shoot;
    Kicker kicker;

    @Override
    public void runOpMode () throws InterruptedException {
        Transfer transfer = new Transfer(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        shooter.init();
        shoot = new Shooter(shooter);
        Kicker kicker = new Kicker(hardwareMap);


        Pose2d initialPose = new Pose2d(62.4,15.2, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        while((!isStopRequested() && !opModeIsActive())){
            shooter.shoot(true);
            shooter.updateShooter();
            shooter.setShooterVelocity(1350);
            telemetry.addData("shooter Vel", shooter.getShooterVelocity());
            telemetry.update();
        }


        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .strafeToLinearHeading(new Vector2d(61.0,11.8),Math.toRadians(153))
                        .stopAndAdd(new SequentialAction(intake.intakeCollect(),
                                new SleepAction(1.5), transfer.transferPass(),new SleepAction(3),
                                kicker.artifactRelease(),
                                new SleepAction(2.8),transfer.transferOff(),kicker.artifactCollect(),
                                new SleepAction(0.5)
                        ))
                        .strafeToLinearHeading(new Vector2d(29.8,26.2),Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(29.3,45.0),Math.toRadians(90))
                        .build()
        );
    }

}

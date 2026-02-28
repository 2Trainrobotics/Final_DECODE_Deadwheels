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

@Autonomous(name="RedFrontAuto")
public class RedFrontAuto extends LinearOpMode {

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


        Pose2d initialPose = new Pose2d(-54.6,49.0, Math.toRadians(45));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        shooter.shoot(true);

        while((!isStopRequested() && !opModeIsActive())){
            shooter.updateShooter();
            shooter.setShooterVelocity(1800);
            telemetry.addData("shooter Vel", shooter.getShooterVelocity());
            telemetry.update();
        }


        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .strafeToLinearHeading(new Vector2d(-47.0,32.0),Math.toRadians(225))
                        .stopAndAdd(new SequentialAction(intake.intakeCollect(),
                                new SleepAction(1.5), transfer.transferPass(),new SleepAction(3),
                                kicker.artifactRelease(),
                                new SleepAction(2.8),transfer.transferOff(),kicker.artifactCollect(),
                                new SleepAction(0.5)
                        ))
                        .strafeToLinearHeading(new Vector2d(-12.0,26.2),Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(-12.5,33.0),Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(-12.5,40.0),Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(-12.5,45),Math.toRadians(270))
                        .build()
        );
    }

}

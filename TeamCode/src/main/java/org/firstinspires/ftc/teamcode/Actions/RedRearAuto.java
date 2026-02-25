package org.firstinspires.ftc.teamcode.Actions;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="RedRearAuto")
public class RedRearAuto extends LinearOpMode {

    Transfer transfer;
    Intake intake;
    Shooter shooter;

    @Override
    public void runOpMode () throws InterruptedException {
        Transfer transfer = new Transfer(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);


        Pose2d initialPose = new Pose2d(62.4,15.2, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .strafeToLinearHeading(new Vector2d(55,14.13),Math.toRadians(135))
                        .stopAndAdd(new SequentialAction(shooter.shooterHighRange(),
                                new SleepAction(4), transfer.transferPass(),intake.intakeCollect(),
                                new SleepAction(6),transfer.transferOff()
                        ))
                        .strafeToLinearHeading(new Vector2d(35.8,26.2),Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(35.3,57.3),Math.toRadians(90))
                        .build()
        );
    }

}

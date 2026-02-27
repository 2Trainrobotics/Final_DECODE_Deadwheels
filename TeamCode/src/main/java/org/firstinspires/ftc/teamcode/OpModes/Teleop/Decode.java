package org.firstinspires.ftc.teamcode.OpModes.Teleop;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ShooterSubsystem;

@TeleOp(name = "RangeTest", group = "FTC")

public class Decode extends LinearOpMode {

    // Limelight Camera located in front of the robot:

    private Limelight3A limelight = null;
    private IMU imu;
    double minTa = Double.MAX_VALUE;
    double maxTa = 0.0;
    long sampleStartTime = 0;
    boolean sampling = false;
ShooterSubsystem shooter;
    private double distance;
    // Here we declare all Mecanum Drive Motors"

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    // Here we declare the subsystems' motors:

    private DcMotor jhoandryRightClimber = null;
    private DcMotor wilmerLeftClimber = null;
   // private DcMotorEx xr = null;
    private DcMotor secondIntake = null;

    // Here we declare the subsystems' servos:

    private CRServo turret = null;
    private CRServo intake = null;
    private Servo kicker = null;
    private Servo hood = null;

    // Here we declare the elevators' integer positions:

    final int HOME_POSITION = 10;
    final int PARK_POSITION = 6000;

    // Here we declare pre-set double positions for the turret:

    final double LONG_RANGE_RIGHT = 0.1;
    final double LONG_RANGE_LEFT = 0.3;
    final double TURRET_HOME_POSITION = 0.2;

    // Here we declare the double positions for the kicker:

    final double ARTIFACT_SHOOT = 1.0;
    final double ARTIFACT_COLLECT = 0.74;
    public static double HOOD = 0.0;

    // Here we declare the turret servo with a gradual increment mode:

    final double ROTATIONAL_CLAW_STOP = 0.0;
    final double ROTATIONAL_CLAW_CLOCKWISE = 0.9;
    final double ROTATIONAL_CLAW_COUNTERCLOCKWISE = -ROTATIONAL_CLAW_CLOCKWISE;

    // Here we program the turret servo position with its default home position:

    double turretPosition = TURRET_HOME_POSITION;

    // Here we program the encoders by configuring them to their default home position:

    int wilmerPosition = HOME_POSITION;
    int jhoandryPosition = HOME_POSITION;

    // Shooter Velocity Setup

    public static double SHOOTER_TARGET_VELOCITY = 1125;
    final double SHOOTER_MIN_VELOCITY = 1075;
    final double STOP_SPEED = 0.0;

    public static double p = 0;
    public static double f = 0;

    private enum LaunchState {

        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    @Override
    public void runOpMode () {


        /* Here's where we guide the class to find the different components
        within the Hardware Map configuration:
         */


        shooter = new ShooterSubsystem(hardwareMap);


        shooter.init();
        shooter.shoot(true);


        leftFront = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftRear = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightRear = hardwareMap.get(DcMotor.class, "rightRearDrive");
        jhoandryRightClimber = hardwareMap.get(DcMotor.class, "rightClimber");
        wilmerLeftClimber = hardwareMap.get(DcMotor.class, "leftClimber");
       // shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        secondIntake = hardwareMap.get(DcMotor.class, "secondIntake");

        // Here we do the same thing, but, for the servos:

        turret = hardwareMap.get(CRServo.class, "turret");
        intake = hardwareMap.get(CRServo.class, "intake");
        kicker = hardwareMap.get(Servo.class, "kicker");
        hood = hardwareMap.get(Servo.class,"hood");
        // Here's where we configure the Limelight and IMU hardwareMap Setup:

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); // april tag #20 & #24 pipeline
//        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
//                RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
//        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        /*Here, we set the direction of the Mecanum wheels to make
        sure they're moving forward relative to their installed
        positions:
         */




        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Here we set the directions for the climbers:

        wilmerLeftClimber.setDirection(DcMotor.Direction.FORWARD);
        jhoandryRightClimber.setDirection(DcMotor.Direction.REVERSE);
        turret.setDirection(CRServo.Direction.REVERSE);

        // Here we set the direction for the Second Intake

        secondIntake.setDirection(DcMotor.Direction.FORWARD);


        // Here's where we program the climbers' encoders

        wilmerLeftClimber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wilmerLeftClimber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        jhoandryRightClimber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jhoandryRightClimber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        waitForStart();

        limelight.start();

        while (opModeIsActive()) {

            /* Here's where we declare the meaning of the drive commands:
            Strafe & turn - through the way on how they would be playing their roles
            on the controller:
             */

            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            /* Here's where we calculate the direction of the motors
            based on the direction of the wheels while strafing:
             */

            double leftFrontPower = (x + y - rotation);
            double leftRearPower = (x - y + rotation);
            double rightFrontPower = (x - y - rotation);
            double rightRearPower = (x + y + rotation);

            // Here's where we assign the powers we declared to the motors:

            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);

            // Here we configure the commands for Gamepad #1:

            shooter.updateShooter();

            if (gamepad1.dpad_up) {

                wilmerPosition = PARK_POSITION;
                jhoandryPosition = PARK_POSITION;
            } else if (gamepad1.dpad_down) {

                wilmerPosition = HOME_POSITION;
                jhoandryPosition = HOME_POSITION;
            }

            if (gamepad1.left_trigger > 0.3) {
                intake.setPower(1);
            }

            else {
                intake.setPower(0);
            }
            if (gamepad1.a) {
                hood.setPosition(HOOD);
            }else if (gamepad1.x){
                hood.setPosition(0.7);

            }

            // Here we configure the commands for Gamepad #2:

            if (gamepad2.right_trigger >0.3) {
                secondIntake.setPower(1);
            }
            else {
                secondIntake.setPower(0);
            }

            if (gamepad2.dpad_left) {
                kicker.setPosition(ARTIFACT_SHOOT);
            }
            else if (gamepad2.dpad_right) {
                kicker.setPosition(ARTIFACT_COLLECT);
            }


//
//            if (gamepad2.a) {
//                shooter.setShooterVelocity();
//            }
//
//            else if(gamepad2.b) {
//                shooter.setPower(0.25);
//            }
//
//            else if (gamepad2.y) {
//                shooter.setPower(0.55);
//            }
//
//            else {
//                shooter.setPower(0);
//            }

            if (gamepad2.a) {
                shooter.setShooterVelocity(1200);
            }

             else if(gamepad2.b) {
                shooter.setShooterVelocity(1650);
            }

            else if (gamepad2.y) {
                shooter.setShooterVelocity(900);
            }



//            double turretIncrement = 0.02;

            if (gamepad2.left_bumper) {
                turret.setPower(-0.5);
            }
            else if (gamepad2.right_bumper) {
                turret.setPower(0.5);
            }
            else {
                turret.setPower(0);
            }

//            turretPosition = Math.max(0, Math.min(1, turretPosition));
//            turret.setPosition(turretPosition);

            wilmerLeftClimber.setTargetPosition(wilmerPosition);
            wilmerLeftClimber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wilmerLeftClimber.setPower(0.4);

            jhoandryRightClimber.setTargetPosition(jhoandryPosition);
            jhoandryRightClimber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            jhoandryRightClimber.setPower(0.4);

            // Giving the user the manual command of the speed of the shooter
           // shooter.setVelocity(SHOOTER_TARGET_VELOCITY);

            // Here we call the "Launch' function.

//            launch(gamepad2.xWasPressed());


            // Limelight Data Gathering

//            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//            limelight.updateRobotOrientation(orientation.getYaw());
            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                turret.setPower(llResult.getTx()*0.048);

                double ta = llResult.getTa();

                // Start sampling once

                if (!sampling) {
                    sampling = true;
                    sampleStartTime = System.currentTimeMillis();
                    minTa = ta;
                    maxTa = ta;
                }

                // Sample for 1.5 seconds
                if (System.currentTimeMillis() - sampleStartTime < 1500) {
                    minTa = Math.min(minTa, ta);
                    maxTa = Math.max(maxTa, ta);
                }

                double avgTa = (minTa + maxTa) / 2.0;
                distance = getDistanceFromTag(avgTa);
               // shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p,0,0,f));
                telemetry.addData("Calculated Distance", distance);
                telemetry.addData("Ta Avg",avgTa);
                telemetry.addData("Ta Min",minTa);
                telemetry.addData("Ta Max",maxTa);
                telemetry.addData("Tx", llResult.getTx());
                turret.setPower(llResult.getTx()*0.038);
                telemetry.addData("Ty",llResult.getTy());
                telemetry.addData("Ta",llResult.getTa());
                telemetry.addData("BotPose",llResult.getBotpose_MT2().toString());
             //   telemetry.addData("Shooter Velocity", shooter.getVelocity());
              //  telemetry.addData("target Vel", shooter.getTargetPosition());

                if (distance > 0) {
                    telemetry.addData("Calculated Distance (in)", distance);
                } else {
                    telemetry.addData("Calculated Distance", "No Tag");
                }
            }
            else {
                turret.setPower(0);
            }
            telemetry.addData("target Vel", shooter.getShooterTargetVelocity());
            telemetry.addData("shooter Vel", shooter.getShooterVelocity());

            telemetry.update();
        } // end while(opModeIsActive)
    }     // end runOpMode()
    public double getDistanceFromTag(double ta) {
        if (ta <= 0.001) {
            return -1; // invalid / no target
        }
        double scale = 3836.092;
        return Math.sqrt(scale / ta) *1.17;
    }
}

package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrists {

    Servo fourWrist, threeWrist;

    public Wrists(HardwareMap map) {
        fourWrist = map.get(Servo.class,"four_stage_claw");
        threeWrist = map.get(Servo.class,"three_stage_claw");

    }

    public class ThreeStagePick implements Action {


        double pick = 0.129;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            threeWrist.setPosition(pick);
            return false;
        }
    }

    public class ThreeStageTransfer implements Action {

        double transfer = 0.99;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            threeWrist.setPosition(transfer);
            return false;
        }
    }

    public class FourStageTransfer implements Action {

        double transfer = 0.65;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            fourWrist.setPosition(transfer);
            return false;
        }
    }

    public class FourStageScore implements Action {

        double score = 0.1;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            fourWrist.setPosition(score);
            return false;
        }
    }

    public Action threeStagePick() {
        return new ThreeStagePick();
    }

    public Action threeStageTransfer(){
        return new ThreeStageTransfer();
    }
    public Action fourStageTransfer() {
        return new FourStageTransfer();
    }
    public Action fourStageScore() {
        return new FourStageScore();
    }
}


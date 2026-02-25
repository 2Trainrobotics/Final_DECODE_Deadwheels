package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    Servo fourSClaw, threeSClaw;

    public Claw(HardwareMap map) {
        fourSClaw = map.get(Servo.class, "four_stage_claw");
        threeSClaw = map.get(Servo.class, "three_stage_claw");
    }


    public class FourStageOpen implements Action {

        double release = 1.0;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            fourSClaw.setPosition(release);
            return false;
        }

    }

    public class FourStageClose implements Action {

        double grip = 0.8;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            fourSClaw.setPosition(grip);
            return false;
        }


    }

    public class ThreeStageOpen implements Action {

        double release = 0.0;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            threeSClaw.setPosition(release);
            return false;
        }
    }
    public class ThreeStageClose implements Action {

        double grip = 0.62;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            threeSClaw.setPosition(grip);
            return false;
        }

    }
        public Action threeStageOpen() {
            return new ThreeStageOpen();
        }
        public Action threeStageClose() {
            return new ThreeStageClose();
        }
        public Action fourStageOpen() {
            return new FourStageOpen();
        }

        public Action fourStageClose() {
            return new FourStageClose();
    }
}


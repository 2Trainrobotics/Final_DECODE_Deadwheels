package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Kicker {

    Servo kicker;


    public Kicker(HardwareMap map) {kicker = map.get(Servo.class,"kicker");
    }

    public class ArtifactCollect implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            kicker.setPosition(0.74);
            return false;
        }
    }

    public class ArtifactRelease implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            kicker.setPosition(1.0);
            return false;
        }
    }

    public Action artifactCollect() {
        return new ArtifactCollect();
    }

    public Action artifactRelease() {
        return new ArtifactRelease();
    }
}

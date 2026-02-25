package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    DcMotor shooter;

    public Shooter(HardwareMap map) {
        shooter = map.get(DcMotor.class,"shooter");
    }

    public class ShooterHighRange implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shooter.setPower(0.8);
            return false;
        }
    }

    public class ShooterOff implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shooter.setPower(0);
            return false;
        }
    }

    public Action shooterHighRange() {
        return new ShooterHighRange();
    }
    public Action shooterOff() {
        return new ShooterOff();
    }
}




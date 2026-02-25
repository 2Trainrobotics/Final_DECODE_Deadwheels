package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer {

    DcMotor transfer;

    public Transfer(HardwareMap map) {
        transfer = map.get(DcMotor.class,"secondIntake");

        transfer.setDirection(DcMotor.Direction.FORWARD);
    }

    public class TransferPass implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transfer.setPower(1.0);
            return false;
        }
    }
    public class TransferOff implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transfer.setPower(0);
            return false;
        }
    }

    public Action transferPass() {
        return new TransferPass();
    }
    public Action transferOff() {
        return new TransferOff();
    }
}


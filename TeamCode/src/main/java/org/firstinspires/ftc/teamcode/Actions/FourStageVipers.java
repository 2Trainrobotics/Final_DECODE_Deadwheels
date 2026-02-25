package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FourStageVipers{

    DcMotor fourStageLeft, fourStageRight;

    // static int targetPostion = 0;


    public FourStageVipers(HardwareMap  map){
        fourStageRight = map.get(DcMotor.class, "four_stage_right_viper");
        fourStageLeft = map.get(DcMotor.class,"four_stage_left_viper");

        fourStageLeft.setDirection(DcMotor.Direction.REVERSE);
        fourStageRight.setDirection(DcMotor.Direction.FORWARD);

        fourStageLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fourStageRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fourStageRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fourStageLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

//    public static class SetTargetPosition implements Action {
//        int targetPos = 0;
//        public SetTargetPosition(int targetPos){
//             this.targetPos = targetPos;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            targetPostion = targetPos;
//            return false;
//        }
//    }



    public class MoveSlides implements Action{
         int targetPosition;

         public MoveSlides(){
             targetPosition = 0;
         }
        public MoveSlides(int targetPosition){
            this.targetPosition = targetPosition;
        }


        boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!init){
                fourStageRight.setTargetPosition(targetPosition);
                fourStageLeft.setTargetPosition(targetPosition);
                fourStageRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                fourStageLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                fourStageRight.setPower( .5);
                fourStageLeft.setPower( .5);

                init = true;
            }

            return fourStageRight.getCurrentPosition() <= targetPosition;
        }
    }



public Action moveSlide( int targetPos){
    return new MoveSlides(targetPos);
}


public Action moveSlide(){
    return new MoveSlides();
}


}

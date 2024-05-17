package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@TeleOp(name = "DebugCode", group = "A")
public class OffDebugCenter extends LinearOpMode {
    Baseauto rbg;




    boolean robo_drive = true;
    boolean end_flag = false;


    int lastError = 0;


    public static double kp = 0, ki = 0, kd = 0, ff = -0.3;




    public enum State {
        INTAKE,
        INTAKEDONE,
        LIFT,
        OUTTAKE,
        OUTTAKEDONE,
    }

    State state = State.INTAKE;

    ElapsedTime pidtimer = new ElapsedTime();


    public void slide_pid(int refrence){

        rbg.arm_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rbg.arm_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int lastError = refrence - rbg.arm_slide.getCurrentPosition();
        double integralSum = 0;
        while (opModeIsActive()){

            int encoderPosition = rbg.arm_slide.getCurrentPosition();
            // calculate the error
            int error = refrence - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastError ) / pidtimer.seconds();

            // sum of all error over time
            integralSum +=(error * pidtimer.seconds());


            double out = (kp * error) + (ki * integralSum) + (kd * derivative) + ff;

            rbg.arm_slide.setPower(out);

            lastError = error;

            // reset the timer for next time
            pidtimer.reset();

            if (gamepad1.share){
                rbg.arm_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rbg.arm_slide.setTargetPosition(0);
                rbg.slide_extend();
                break;
            }
            telemetry.addData("pos ", encoderPosition);
            telemetry.addData("error", error);
            telemetry.addData("target ", refrence);
            telemetry.addData("power", out);
            telemetry.update();
        }



    }





    @Override
    public void runOpMode() {




        telemetry.addLine("Wait ! Initializing............. ");
        telemetry.update();
        Pose2d p1 =new Pose2d(0,0,0);

        rbg = new Baseauto(this, p1);


      //  if(rbg.april_ready) telemetry.addLine("April tag setup ready!");
        telemetry.addLine("For practice only:");
        telemetry.addLine(" BLUE:Driver --Cross ");
        telemetry.addLine(" RED: Driver---Circle");
        telemetry.addLine("Press  driver: ∆  to reset the IMU angle to Zero " );

        telemetry.update();

        while (!isStarted() && !isStopRequested()) {

            if (gamepad1.triangle) {
                rbg.imu.initialize(rbg.imuparameters);
                rbg.imu.resetYaw();
                sleep(1000);
                break;

            }
            if (gamepad1.cross) {

                rbg.baseblue=true;
                rbg.base_align_angle=-90;
                rbg.base_apr_id=1;

            }
            if ( gamepad1.circle) {

                rbg.baseblue=false;
                rbg.base_align_angle=90;
                rbg.base_apr_id=4;

            }

        }

        telemetry.addLine("Press Start Now!:");
        telemetry.update();
         if (isStopRequested()) return;

        int arm_pos = 0;
        int slide_pos = 0;

        double handle_pos = 0;
        double arm_grab_pos = 0;
        double drone_pos = 0;


        waitForStart();

        rbg.drone.setPosition(0);

        rbg.arm_grab.setPosition(0);
        rbg.arm_handle.setPosition(0);

     rbg.endgame = rbg.runtime.seconds();

     //only for the cam outside duirng the autonomus



        while (opModeIsActive()) {
            rbg.arm_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x * 1.1;
            double rx = gamepad1.left_stick_x*0.75;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            rbg.front_left.setPower(-frontLeftPower);
            rbg.rear_left.setPower(-backLeftPower);
            rbg.front_right.setPower(frontRightPower);
            rbg.rear_right.setPower(backRightPower);

            if (gamepad1.cross){
                rbg.ZeroSlide();
            }

            //Arm rotate debug code

            if (gamepad2.right_bumper && arm_pos<3000){
                sleep(400);
                arm_pos+=100;
                rbg.arm_rotate.setTargetPosition(arm_pos);
                rbg.arm_rotate.setVelocity(1600);

            }

            if(gamepad2.left_bumper && arm_pos>0){
                sleep(400);
                arm_pos-=100;
                rbg.arm_rotate.setTargetPosition(arm_pos);
                rbg.arm_rotate.setVelocity(1600);

            }


            // Arm slide debug code

            if(gamepad2.circle && slide_pos < 0) {
                sleep(400);
                slide_pos += 100;
                rbg.arm_slide.setTargetPosition(slide_pos);
                rbg.arm_slide.setVelocity(2000);
            }

            if(gamepad2.square && slide_pos > -6000 ){
                sleep(400);
                slide_pos-=100;
                rbg.arm_slide.setTargetPosition(slide_pos);
                rbg.arm_slide.setVelocity(2000);
            }




            // Arm grabber claw debug code
            if (gamepad2.dpad_up && arm_grab_pos<1.0){
                sleep(400);
                arm_grab_pos  +=0.02;
                rbg.arm_grab.setPosition(arm_grab_pos);
            }

            if(gamepad2.dpad_down && arm_grab_pos>0){
                sleep(400);
                arm_grab_pos-=0.02;
                rbg.arm_grab.setPosition(arm_grab_pos);
            }

            //arm handler debug code


            if (gamepad2.dpad_right && handle_pos<1.0){
                sleep(400);
                handle_pos  +=0.01;
                rbg.arm_handle.setPosition(handle_pos);
            }

            if(gamepad2.dpad_left && handle_pos>0){
                sleep(400);
                handle_pos-=0.01;
                rbg.arm_handle.setPosition(handle_pos);
            }

            //Both lock
            if (gamepad1.circle){
                rbg.arm_grab.setPosition(0.6);
                sleep(500);
            }
            //First pixel release
            if (gamepad1.cross){
                rbg.arm_grab.setPosition(0.53);
                sleep(500);
            }

            if (gamepad2.right_trigger>0.6){
                drone_pos+=0.01;
                rbg.drone.setPosition(drone_pos);
                sleep(500);

            }

            if (gamepad2.left_trigger > 0.6){
                drone_pos -= 0.01;
                rbg.drone.setPosition(drone_pos);
                sleep(500);

            }


            //second Pixel release
            if (gamepad1.square){
                rbg.arm_grab.setPosition(0.15);
                sleep(500);

            }

            if (gamepad1.triangle){
                slide_pid(-2000);
            }

            telemetry.addData("Arm_rotate Gunner.right/left bumper", arm_pos );
            telemetry.addData("Arm_slide Gunner.circle/square", slide_pos);
            telemetry.addData("Arm_grab Gunner.dpad_up/dpad_down", arm_grab_pos);
            telemetry.addData("Arm_handler Gunner.dpad_right,dpad_left", handle_pos);
            telemetry.addData("Drone Gunner right_trigger, left_trigger", drone_pos);
            telemetry.update();










        }
    }
}










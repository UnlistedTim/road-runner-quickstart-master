package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


@Config
@TeleOp(name = "DebugCode", group = "A")
public class OffDebugCenter extends LinearOpMode {
    Baseauto rbg;




    boolean robo_drive = true;
    boolean end_flag = false;

    double top_claw_value = 1.0;
    double bottom_claw_value = 0;


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

        double handle_pos = 0.5;
        double arm_grab_pos = 0;
        double drone_pos = 0;

//        rbg.front_right.setDirection(DcMotorSimple.Direction.REVERSE);
//        rear_right.setDirection(DcMotorSimple.Direction.REVERSE);
//        rear_left.setDirection(DcMotorSimple.Direction.FORWARD);
//        front_left.setDirection(DcMotorSimple.Direction.FORWARD);

        if (rbg.USE_WEBCAM)  rbg.setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        waitForStart();
        rbg.imu.resetYaw();

        rbg.top_claw.setPosition(top_claw_value);
        rbg.bottom_claw.setPosition(bottom_claw_value);
        rbg.arm_handle.setPosition(0.5);

        rbg.drone.setPosition(0);

//        rbg.arm_grab.setPosition(0);
//        rbg.arm_handle.setPosition(0);
        rbg.arm_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
     rbg.endgame = rbg.runtime.seconds();

     //only for the cam outside duirng the autonomus



        while (opModeIsActive()) {





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
//            if (gamepad2.dpad_up && arm_grab_pos<1.0){
//                sleep(400);
//                arm_grab_pos  +=0.02;
//                rbg.arm_grab.setPosition(arm_grab_pos);
//            }
//
//            if(gamepad2.dpad_down && arm_grab_pos>0){
//                sleep(400);
//                arm_grab_pos-=0.02;
//                rbg.arm_grab.setPosition(arm_grab_pos);
//            }

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
//            if (gamepad1.circle){
//                rbg.arm_grab.setPosition(0.6);
//                sleep(500);
//            }
//            //First pixel release
//            if (gamepad1.cross){
//                rbg.arm_grab.setPosition(0.53);
//                sleep(500);
//            }

            if (gamepad1.dpad_up && top_claw_value<1.0){
                sleep(400);
                top_claw_value+=0.01;
                rbg.top_claw.setPosition(top_claw_value);
            }
            if (gamepad1.dpad_down && top_claw_value>0){
                sleep(400);
                top_claw_value-=0.01;
                rbg.top_claw.setPosition(top_claw_value);
            }
            if (gamepad1.dpad_right && bottom_claw_value<1.0){
                sleep(400);
                bottom_claw_value+=0.01;
                rbg.bottom_claw.setPosition(bottom_claw_value);
            }
            if (gamepad1.dpad_left && bottom_claw_value>0){
                sleep(400);
                bottom_claw_value-=0.01;
                rbg.bottom_claw.setPosition(bottom_claw_value);
            }

//            if (gamepad1.left_bumper){
//                //Both open
//                rbg.bottom_claw.setPosition(bottom_claw_value);
//                sleep(300);
//                rbg.top_claw.setPosition(top_claw_value);
//
//
//            }
            if (gamepad1.right_bumper){
                //Both closed
//                rbg.bottom_claw.setPosition(0.2);
//                sleep(200);
                rbg.bottom_claw.setPosition(rbg.bottom_claw_hold);
//                rbg.top_claw.setPosition(0.27);
//                sleep(200);
                rbg.top_claw.setPosition(rbg.top_claw_hold);


            }

            if (gamepad2.dpad_up){
                rbg.arm_handle.setPosition(rbg.arm_handle_idle);
                sleep(500);
                rbg.slide(rbg.arm_slide_idle);
                rbg.bottom_claw.setPosition(rbg.bottom_claw_idle);
                rbg.top_claw.setPosition(rbg.top_claw_idle);
                sleep(3000);
                rbg.rotate(rbg.arm_rotate_op1-200);
                sleep(1000);
                rbg.arm_handle.setPosition(rbg.arm_handle_op1);


            }
//            if (gamepad2.dpad_left){
//                rbg.arm_rotate.setTargetPosition(rbg.arm_rotate_ground);
//                rbg.arm_rotate.setVelocity(800);
//                rbg.arm_handle.setPosition(rbg.arm_handle_idle);
//                sleep(2000);
//
//
//            }
            if (gamepad2.dpad_down){
                rbg.slide(rbg.arm_slide_idle);
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


//            //second Pixel release
//            if (gamepad1.square){
//                rbg.arm_grab.setPosition(0.15);
//                sleep(500);
//
//            }

//            if (gamepad1.triangle){
//                slide_pid(-2000);
//            }

            if (gamepad1.left_bumper){
                rbg.autoback();
//                rbg.timer2(0);
//
//
//                while (opModeIsActive() && !rbg.timer2(3000)){
//                    rbg.targetFound = false;
//                    rbg.desiredTag  = null;
//
//                    // Step through the list of detected tags arnd look for a matching tag
//                    List<AprilTagDetection> currentDetections = rbg.aprilTag.getDetections();
//                    for (AprilTagDetection detection : currentDetections) {
//                        // Look to see if we have size info on this tag.
//                        if (detection.metadata != null) {
//                            //  Check to see if we want to track towards this tag.
//                            if ( detection.id == rbg.DESIRED_TAG_ID) {
//                                // Yes, we want to use this tag.
//                                rbg.targetFound = true;
//                                rbg.desiredTag = detection;
//                                telemetry.addData("Detection ID", detection.id);
//                                break;  // don't look any further.
//
//                            }
//                        }
//                    }
//
//                    if (rbg.targetFound) {
//
//                        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
//                        double  rangeError      = (rbg.front_dist.getDistance(DistanceUnit.INCH) - rbg.DESIRED_DISTANCE_T);
//                        double  headingError    = rbg.base_align_angle - rbg.imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
//                        double yawError = rbg.desiredTag.ftcPose.yaw - rbg.STRAFE_TARG;
//
//                        // Use the speed and turn "gains" to calculate how we want the robot to move.
//                        rbg.drive  = Range.clip(rangeError * rbg.AUTOSPEED_GAIN, -rbg.MAX_AUTO_SPEED, rbg.MAX_AUTO_SPEED);
//                        rbg.turn   = -Range.clip(headingError * rbg.TURN_GAIN, -rbg.MAX_AUTO_TURN, rbg.MAX_AUTO_TURN) ;
//                        rbg.strafe = -Range.clip(yawError * rbg.STRAFE_GAIN, -rbg.MAX_AUTO_STRAFE, rbg.MAX_AUTO_STRAFE);
//                        rbg.moveRobot(rbg.drive, rbg.strafe, rbg.turn,1,false);
//
////                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", rbg.drive, rbg.strafe, rbg.turn);
////                telemetry.addData("Range", "%5.1f inches", rbg.desiredTag.ftcPose.range);
////                telemetry.addData("Bearing", "%3.0f degrees", rbg.desiredTag.ftcPose.bearing);
//                        telemetry.addData("Yaw", "%3.0f degrees", rbg.desiredTag.ftcPose.yaw);
//                    }
//                    else {
//                        rbg.stop_drive();
//                    }
//
//                }
//
//                rbg.stop_drive();
//                sleep(500);
//
//                rbg.rotate(rbg.arm_rotate_op1);
//                sleep(500);
//                rbg.timer3(0);
//
//                while(opModeIsActive() && rbg.gap >=16 && !rbg.timer3(3000)){
//                    rbg.dist_align();
//                    telemetry.addData("front_dist", rbg.front_dist.getDistance(DistanceUnit.MM));
//                    telemetry.update();
//                }
//                rbg.stop_drive();
//                sleep(500);
//
//                // outake 2 release
//                rbg.top_claw.setPosition(rbg.top_claw_idle+0.03);
//
//                //arm_grab.setPosition(arm_grab_idle);
//                sleep(500);
//                rbg.rotate(rbg.arm_rotate_out_buffer);
//                rbg.slide(rbg.arm_slide_idle);
//                rbg.arm_handle.setPosition(rbg.arm_handle_idle);
//                //pause(500);
//                rbg.rotate(rbg.arm_rotate_buffer);
//                sleep(1000);
//                rbg.rotate(rbg.arm_rotate_ground);
//                rbg.slide(rbg.arm_slide_extend);
//                sleep(50000);



            }





            if (gamepad1.circle){
                rbg.rotate(rbg.arm_rotate_op1);

                while(opModeIsActive() && rbg.gap >=16){
                    rbg.dist_align();
                    telemetry.addData("front_dist", rbg.front_dist.getDistance(DistanceUnit.MM));
                    telemetry.update();
                }
                rbg.stop_drive();
                sleep(500);
                rbg.outtake_2release();
            }
//            telemetry.update();

            // Apply desired axes motions to the drivetrain.


            telemetry.addData("Arm_rotate Gunner.right/left bumper", arm_pos );
            telemetry.addData("Arm_slide Gunner.circle/square", slide_pos);
            telemetry.addData("Arm_grab Gunner.dpad_up/dpad_down", arm_grab_pos);
            telemetry.addData("Arm_handler Gunner.dpad_right,dpad_left", handle_pos);
            telemetry.addData("Drone Gunner right_trigger, left_trigger", drone_pos);

            telemetry.addData("Left Claw Value", top_claw_value);
            telemetry.addData("right claw value", bottom_claw_value);

            telemetry.addData("front_dist", rbg.front_dist.getDistance(DistanceUnit.MM));
            telemetry.addData("Left_dist", rbg.left_dist.getDistance(DistanceUnit.MM));
//            telemetry.addData("imu heading", rbg.imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES)));

            telemetry.update();










        }
    }
}










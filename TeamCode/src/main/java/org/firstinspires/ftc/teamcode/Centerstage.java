package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "CenterStageA", group = "A")
public class Centerstage extends LinearOpMode {
    Baseauto rbg;

    int pixels = 2;

    boolean right_flag = false;

    boolean buffer_to_zero_flag = false;

    double speed_factor = 1.0;

    public enum State {
        INTAKE,
        LIFT,
        OUTTAKE,
    }

    State state = State.INTAKE;

    ElapsedTime outtake_timer = new ElapsedTime();



    @Override
    public void runOpMode() {


        telemetry.addLine("Wait ! Initializing............. ");
        telemetry.update();
        Pose2d p1 =new Pose2d(0,0,0);

        rbg = new Baseauto(this, p1);

      //  if(rbg.april_ready) telemetry.addLine("April tag setup ready!");
//        telemetry.addLine("For practice only:");
//        telemetry.addLine(" BLUE:Driver --Cross ");
//        telemetry.addLine(" RED: Driver---Circle");
//        telemetry.addLine("Press  driver: ∆  to reset the IMU angle to Zero " );
//
//        telemetry.update();

//        while (!isStarted() && !isStopRequested()) {

//            if (gamepad1.triangle) {
//                rbg.imu.initialize(rbg.imuparameters);
//                rbg.imu.resetYaw();
//                sleep(1000);
//                break;
//
//            }
//            if (gamepad1.cross) {
//
//                rbg.baseblue=true;
//                rbg.base_align_angle=-90;
//                rbg.base_apr_id=1;
//
//            }
//            if ( gamepad1.circle) {
//
//                rbg.baseblue=false;
//                rbg.base_align_angle=90;
//                rbg.base_apr_id=4;
//
//            }
//
//        }
         if (isStopRequested()) return;
        rbg.slide(rbg.arm_slide_collapse);
        rbg.arm_handle.setPosition(rbg.arm_handle_idle);
        rbg.drone.setPosition(rbg.drone_idle);
        //rbg.arm_grab.setPosition(rbg.arm_grab_idle);

        telemetry.addLine("Press Start Now!:");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            switch (state) {

                case INTAKE:

                    if (buffer_to_zero_flag && rbg.arm_rotate.getCurrentPosition() < 200){
                        rbg.arm_rotate.setTargetPosition(rbg.arm_rotate_ground);
                        rbg.arm_rotate.setVelocity(500);
                        buffer_to_zero_flag = false;
                    }
                    if (gamepad1.left_bumper){
                        speed_factor = 0.4;
                        rbg.intake_ready();
                        right_flag = true;
                    }

                    if (gamepad1.right_bumper && right_flag){
                        rbg.intake_grab();
                        right_flag = false;

                    }
                    if (gamepad1.triangle){
                        speed_factor = 1.0;
                        state = State.LIFT;

                    }


                    break;
                case LIFT:

                    if (gamepad2.right_bumper){
                        rbg.outtake_turn();
                    }


                    if (rbg.arm_rotate.getCurrentPosition() > 1750){



                        rbg.outtake_ready();
                        state = State.OUTTAKE;

                        pixels = 2;
                    }

                    break;

                case OUTTAKE:
                    speed_factor = 0.4;

                    if (gamepad2.dpad_up && outtake_timer.milliseconds() > 400){
                        rbg.manual_slide_up();
                        outtake_timer.reset();
                    }
                    if (gamepad2.dpad_down && outtake_timer.milliseconds() > 400){

                        rbg.manual_slide_down();
                        outtake_timer.reset();
                    }

                    if (gamepad1.right_bumper && pixels == 2){
                        rbg.outtake_1release();
                        pixels--;
                    }
                    else if (gamepad1.right_bumper && pixels == 1){
                        rbg.outtake_2release();
                        pixels--;
                        state = State.INTAKE;
                        speed_factor = 1.0;

                        buffer_to_zero_flag = true;
                    }


                    break;


            }

            if (gamepad1.dpad_right){
                telemetry.addLine("Hang ready");
                telemetry.update();
                rbg.hang_ready();
            }
            if (gamepad2.dpad_right){
                telemetry.addLine("hang start");
                telemetry.update();
                rbg.hang();
            }

            if (gamepad2.dpad_left){
                rbg.drone_fly();
            }

            rbg.coordinate_input(gamepad2.triangle,gamepad2.circle,gamepad2.cross,gamepad2.square);
            rbg.coordinate_confirm(gamepad2.touchpad);

            if (gamepad1.share){
                rbg.ZeroSlide();
            }


            rbg.robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, speed_factor);


//            telemetry.addData("TempInput", rbg.tempinput);
//            telemetry.update();

//            if (rbg.driving) {
//                if (robo_drive)
//                    rbg.robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
//                else
//                    rbg.field_centric(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
//            }


        }
    }
}










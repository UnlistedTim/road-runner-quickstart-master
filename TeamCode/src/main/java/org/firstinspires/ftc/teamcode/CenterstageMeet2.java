package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;


@TeleOp(name = "CSMeet2", group = "A")
public class CenterstageMeet2 extends LinearOpMode {
    Baseauto rbg;

    int pixels = 2;

    boolean right_flag = false;

    boolean buffer_to_zero_flag = false;

    boolean hang_ready = false;

    double speed_factor = 1.0;

    boolean aligning = false;

    public enum State {
        INTAKE,
        ROT,
        ALIGN,
        OUTTAKE,
    }

    State state = State.INTAKE;

    ElapsedTime outtake_timer = new ElapsedTime();
    ElapsedTime loop_time = new ElapsedTime();



    @Override
    public void runOpMode() {


        telemetry.addLine("Wait ! Initializing............. ");
        telemetry.update();
        Pose2d p1 =new Pose2d(0,0,0);

        rbg = new Baseauto(this, p1);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
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
                    aligning = false;

                    if (buffer_to_zero_flag && rbg.arm_rotate.getCurrentPosition() < 400){
                        rbg.arm_rotate.setTargetPosition(rbg.arm_rotate_ground);
                        rbg.arm_rotate.setVelocity(400);
                        buffer_to_zero_flag = false;
                        rbg.bottom_claw.setPosition(rbg.bottom_claw_open);
                        rbg.top_claw.setPosition(rbg.top_claw_open);
                    }
                    if (gamepad1.left_bumper && !right_flag && rbg.arm_rotate.getCurrentPosition() < 100 && rbg.arm_slide.getCurrentPosition() < rbg.arm_slide_idle + 300){
                        speed_factor = 0.4;
                        rbg.intake_ready();
                        right_flag = true;
                    }

                    if (gamepad1.right_bumper && right_flag){
                        rbg.intake_grab();
                        speed_factor = 1.0;
                        right_flag = false;
                        state = State.ROT;


                    }



                    break;
                case ROT:

                    if (gamepad1.left_bumper ){
                        speed_factor = 0.4;
                        rbg.intake_ready();
                        right_flag = true;
                        state = State.INTAKE;
                    }

                    if (gamepad1.right_bumper && rbg.arm_slide.getCurrentPosition() < rbg.arm_slide_idle + 300 &&!right_flag ){
                        rbg.outtake_turn();

                        state = State.ALIGN;

                    }






                    break;
                case ALIGN:

                    if (gamepad1.right_bumper && rbg.arm_rotate.getCurrentPosition() > rbg.arm_rotate_op4 - 200 - 100){
                        rbg.outtake_ready();
                        state = State.OUTTAKE;
                        pixels = 2;
                    }

                    break;


                case OUTTAKE:
                    speed_factor = 0.4;
                    aligning = false;

                    if (gamepad1.right_bumper){
                        rbg.timer2(0);
                        while (opModeIsActive() && rbg.gap >= 10 && !rbg.timer2(3000) ){
//                            telemetry.addData("In ALign", rbg.gap);
//                            telemetry.update();
                            rbg.dist_align();
                            aligning = true;
                        }
                        rbg.gap = 100;


                    }


                    if (gamepad2.dpad_up && outtake_timer.milliseconds() > 400){
                        rbg.manual_slide_up();
                        outtake_timer.reset();
                    }
                    if (gamepad2.dpad_down && outtake_timer.milliseconds() > 400){

                        rbg.manual_slide_down();
                        outtake_timer.reset();
                    }

                    if (gamepad1.left_bumper && pixels == 2 && rbg.arm_rotate.getCurrentPosition() > (rbg.arm_rotate_op4 - 100)){
                        rbg.outtake_1release();
                        rbg.manual_slide_up();
                        pixels--;

                    }
                    else if (gamepad1.left_bumper && pixels == 1 && rbg.arm_rotate.getCurrentPosition() > (rbg.arm_rotate_op4 - 100)){
                        rbg.outtake_2release();
                        pixels--;
                        rbg.tempinput--;
                        state = State.INTAKE;
                        speed_factor = 1.0;

                        buffer_to_zero_flag = true;
                    }


                    break;


            }

            if (gamepad1.dpad_right){
                rbg.hang_ready();
                hang_ready = true;
                telemetry.addLine("Hang ready");
                telemetry.update();
            }
            if (gamepad2.dpad_right && hang_ready && rbg.arm_slide.getCurrentPosition() > rbg.arm_slide_idle - 300){
                telemetry.addLine("hang start");
                telemetry.update();
                rbg.hang();
            }

            if (gamepad2.dpad_left){
                rbg.stop_drive();
                sleep(400);
                if (gamepad2.dpad_left){
                    rbg.drone_fly();
                }


            }

            rbg.coordinate_input(gamepad2.triangle,gamepad2.circle,gamepad2.cross,gamepad2.square);
            rbg.coordinate_confirm(gamepad2.touchpad);

            if (gamepad1.share){
                rbg.ZeroSlide();
            }


            if (!aligning) {
                rbg.vel_robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, speed_factor);
//                rbg.robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, speed_factor);
            }

//            telemetry.addData("Loop time", loop_time.milliseconds());
//            telemetry.addData("front dist", rbg.front_dist.getDistance(DistanceUnit.MM));
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










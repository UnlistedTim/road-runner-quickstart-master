package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;


@TeleOp(name = "CenterStageA", group = "A")
public class Centerstage extends LinearOpMode {
    Baseauto rbg;

    int pixels = 2;


    boolean robo_drive = true;
    public enum State {
        INTAKE,
        LIFT,
        OUTTAKE,
    }

    State state = State.INTAKE;



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

        waitForStart();
        rbg.arm_handle.setPosition(rbg.arm_handle_idle);
        rbg.arm_grab.setPosition(rbg.arm_grab_idle);


        rbg.slide(rbg.arm_slide_idle);



     //only for the cam outside duirng the autonomus



        while (opModeIsActive()) {


            switch (state) {

                case INTAKE:

                    if (gamepad1.left_bumper){
                        rbg.rotate(0);
                        rbg.slide_extend();
                        rbg.arm_handle.setPosition(rbg.arm_handle_ip0);
                    }

                    if (gamepad1.right_bumper){
                        rbg.intake_grab();

                        state = State.LIFT;


                    }



                    break;



                case LIFT:

                    if (gamepad1.dpad_up){
                        rbg.outtake_turn();

                        state = State.OUTTAKE;

                        pixels = 2;
                    }


                    break;

                case OUTTAKE:

                    if (gamepad2.right_bumper && pixels == 2){
                        rbg.outtake_1release();
                        pixels--;
                    }
                    else if (gamepad2.right_bumper && pixels == 1){
                        rbg.outtake_2release();
                        pixels--;

                        state = State.INTAKE;


                    }


                    break;




            }


            if (gamepad1.share){
                rbg.ZeroSlide();
            }


// run every cycle below


            rbg.robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

//            if (rbg.driving) {
//                if (robo_drive)
//                    rbg.robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
//                else
//                    rbg.field_centric(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
//            }










        }
    }
}










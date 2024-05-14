package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "AutomeetState", group = "A")
//@Disabled
@Config
public class Automeet extends LinearOpMode {
   // private ElapsedTime runtime = new ElapsedTime();

    boolean setup_ready=false;
   // boolean debuging_mode=false;// Need turn off for game
    long delay=0;
   Pose2d pp;

    Baseauto rbga;
    @Override
    public void runOpMode()  {
    rbga=new Baseauto(this,pp);

        rbga.baseblue=false;
        rbga.baseright=false;


    configuration_info();

    rbga.index=huskydet();
    waitForStart();
        rbga.endgame= rbga.runtime.seconds()+30;

        if(rbga.index==0) {

             rbga.index=2;
         }



        while (opModeIsActive() ) {


            autoroute();
            break;
        }
    }


void configuration_info()

{
    telemetry.addLine("0.Load the Drone   ) ");
    telemetry.addLine("1.Load one yellow pixel inside the robot  ) ");
    telemetry.addLine("2.Load one purple pixel behind the claws  ) ");
    telemetry.update();

  //  rbga.intake_handle(0);//

    while(!setup_ready&& !isStopRequested()) {

//        if (gamepad2.dpad_up || gamepad2.dpad_down) {
//
//            if(gamepad2.dpad_up ) delay=delay+5000;
//            if(gamepad2.dpad_down ) delay=delay-5000;
//            if (delay>15000) delay=15000;
//            if (delay<0)  delay=0;
//            telemetry.addData("Delay Time",delay);
//            telemetry.update();
//        }

        if (gamepad1.dpad_up || gamepad1.dpad_down){
            if (gamepad1.dpad_up) rbga.delay[0]+=1000;
            if (gamepad1.dpad_down) rbga.delay[0]-=1000;
            sleep(300);
        }
        if (gamepad1.dpad_right || gamepad1.dpad_left){
            if (gamepad1.dpad_right) rbga.delay[1]+=1000;
            if (gamepad1.dpad_left) rbga.delay[1]-=1000;
            sleep(300);
        }

        if (gamepad1.b || gamepad1.circle) {
            rbga.baseright=true;
        }
        if (gamepad1.x || gamepad1.square) {

            rbga.baseright=false;

        }

        if (gamepad2.a || gamepad2.cross) {

            rbga.baseblue=true;

        }
        if (gamepad2.b || gamepad2.circle) {

            rbga.baseblue=false;

        }

//        if (gamepad2.y || gamepad2.triangle) {
//
//            rbga.lgrab();
//
//        }

        if(delay>0) telemetry.addData("Delay Time(ms)",delay);
        telemetry.addLine();

        if (rbga.delay[0]> 0){
            telemetry.addData("First Outtake Delay   (second)",rbga.delay[0]/1000);
            telemetry.addLine();
        }
        if (rbga.delay[1] > 0){
            telemetry.addData("Second Outtake Delay   (second)",rbga.delay[1]/1000);
            telemetry.addLine();
        }

        if(rbga.baseright)
            telemetry.addLine(" Right   SIDE ");
        else
            telemetry.addLine(" Left   SIDE ");
        telemetry.addLine();

        if(rbga.baseblue)
            telemetry.addLine(" Blue   Color ");
        else
            telemetry.addLine(" Red   Color ");

        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine("0. Load  the Drone ");
        telemetry.addLine("1. Load  yellow and purple pixels ");
        telemetry.addLine("2. Left/Right :Driver  LEFT:Square  RIGHT:Circle ");
        telemetry.addLine("3. Bule/Red: Gunner    BLUE:Cross(A)    RED:Circle(B)");
        telemetry.addLine("4. Place the Robot with right location  ") ;
        telemetry.addLine("6. Optional: Add delay time.  Gunner: Dpad-Up +5s(max 15s), Dapd-Down -5s  ") ;
        telemetry.addLine("7.  Coach double check and press Driver-- Triangle key ") ;
        telemetry.addLine("8.  Check the huskeylens screen to make sure the ID is correct") ;
        telemetry.update();


        if ( gamepad1.triangle) setup_ready=true;

    }




    if (rbga.baseblue) {
        telemetry.addLine("BLUE Color confirmed ");
       rbga.base_align_angle = -90;
        rbga.base_apr_id = 1;
    }
    else {
        telemetry.addLine("RED Color confirmed ");
        rbga.base_align_angle = 90;
        rbga.base_apr_id = 4;

    }

    telemetry.addLine();


    if (rbga.baseright)
        telemetry.addLine("RIGHT Side confirmed ");
    else
        telemetry.addLine("LEFT Side confirmed ");

    if (rbga.outtake_1_delay > 0){
        telemetry.addData("First Outtake Delay @ Backstage Wall side (ms)",rbga.outtake_1_delay);
    }
    if (rbga.outtake_2_delay > 0){
        telemetry.addData("Second Outtake Delay @ Backstage Wall side (ms)",rbga.outtake_2_delay);
    }

    telemetry.addLine("Check the huskeylens ID. If not correct, restart the program ");

    telemetry.addLine();

  //  if(delay>0)   telemetry.addData("Delay Time(seconds)",delay);
  //  telemetry.update();
    telemetry.addLine();
    /** Wait for the game to begin */
  //  telemetry.addData("Status", "Initialized");
    telemetry.addData(">", "Press Play to start op mode ");
    telemetry.update();

}





 public int huskydet() {
     HuskyLens.Block block;
     int out = 0;
     int READ_PERIOD = 1;
     int i = 0;
    int[] tags = {0,0,0,0,0};
     int[] count = {0,0,0,0};
    int max = 0;
    int ID_count = 0;
     Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
     rateLimit.expire();



     while (!isStopRequested()&&!isStarted()) {
        // if(out>0) out=0;
         if (!rateLimit.hasExpired()) {
             continue;
         }

         rateLimit.reset();


           //  telemetry.addData("ID Inside", block.id);




             //telemetry.addData("out",out);


         if(out>0) {
             if (i < 5) tags[i] = out;
             else {
                 tags[0] = tags[1];
                 tags[1] = tags[2];
                 tags[2] = tags[3];
                 tags[3] = tags[4];
                 tags[4] = out;
             }
             i++;
         }


     }


//
     for (int x = 0;x<5;x++){
         if(tags[x] == 1)count[1]++;
         else {
             if (tags[x] == 3)   count[3]++;
             else   count[2]++;
         }
     }


     // Initialize maximum element


     // Traverse array elements from second and
     // compare every element with current max
     for (int z = 1; z < 4; z++)
         if (count[z] > max){
             max = count[z];
             ID_count = z;
         }




    // if(out==0) out=out1;
    if(ID_count>0) return out;
     else return 2;
 }

 void autoroute() {







     sleep(1000);

 }


 }



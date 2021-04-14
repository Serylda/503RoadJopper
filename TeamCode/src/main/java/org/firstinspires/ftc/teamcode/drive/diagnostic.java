/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.DrivetrainHardware;
import org.firstinspires.ftc.teamcode.Vision;

@TeleOp(name="diagnostic", group="Arcade")
//@Disabled
public class diagnostic extends LinearOpMode {

    DrivetrainHardware mDrive = new DrivetrainHardware();

    static int currentState;
    static final int IDLE = 0;
    static final int MANEUVERING = 1;
    static final int SHOOTING = 2;
    static final int COLLECTING = 3;
    ElapsedTime waitTimer = new ElapsedTime();

    double mode = 1;

    double maxVelocity = 0;
    double maxVelocity2 = 0;
    double indexDirection = 1;
    ElapsedTime bruh = new ElapsedTime();

    @Override
    public void runOpMode()
    {

        mDrive.init(hardwareMap);


        Vision vision = new Vision(this);
        currentState = IDLE;


        mDrive.FlyWheel2.setVelocityPIDFCoefficients(1.622, 0.1622, 0, 16.22);
        mDrive.FlyWheel1.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);

        waitForStart();

        while (opModeIsActive())
        {
            telemetry();
            if(gamepad1.a){
                runFlyWheel();

            }
            telemetry.addData("elapsed time", waitTimer.milliseconds());
            telemetry.update();

        }
    }


    public void telemetry(){





    }

    public void runFlyWheel()
    {







                telemetry.addData("elapsed time", waitTimer.milliseconds());
                telemetry.update();

             //  mDrive.FlyWheel1.setVelocity(1500 * 13.5 / mDrive.getVoltage());
             //  mDrive.FlyWheel2.setVelocity(1500 * 13.5 / mDrive.getVoltage());

               waitTimer.reset();



                while (waitTimer.milliseconds() < 1500){

                    if (waitTimer.milliseconds() >= 250 && waitTimer.milliseconds() < 400){
                        mDrive.ringHopper.setPosition(0.9);



                    }
                    if (waitTimer.milliseconds() >= 400 && waitTimer.milliseconds() < 650){
                        mDrive.ringHopper.setPosition(0.5);

                    }
                    if (waitTimer.milliseconds() >= 650 && waitTimer.milliseconds() < 800){
                        mDrive.ringHopper.setPosition(0.9);



                    }
                    if (waitTimer.milliseconds() >= 800 && waitTimer.milliseconds() < 1050){
                        mDrive.ringHopper.setPosition(0.5);



                    }
                    if (waitTimer.milliseconds() >= 1050 && waitTimer.milliseconds() < 1200){
                        mDrive.ringHopper.setPosition(0.9);



                    }
                    if (waitTimer.milliseconds() >= 1200 && waitTimer.milliseconds() < 1450){
                        mDrive.ringHopper.setPosition(0.5);




                    }
                    if (waitTimer.milliseconds() >= 1450){
                        mDrive.FlyWheel1.setVelocity(0);
                        mDrive.FlyWheel2.setVelocity(0);

                    }


                }

            }




    public void doArm()
    {   if(gamepad2.right_trigger > 0.1) {


        ElapsedTime clock = new ElapsedTime();

        if (gamepad2.b == true){
            clock.reset();

        }
        mDrive.Arm.setPower(-0.5);

        if (clock.milliseconds() >= 350 && clock.milliseconds() < 500) {
            mDrive.Arm.setPower(0);

        }

        if (clock.milliseconds() >= 500 && clock.milliseconds() < 800) {

            mDrive.claw.setPosition(1);

        }

        if (clock.milliseconds() >= 800 && clock.milliseconds() < 1000) {

            mDrive.Arm.setPower(0.5);

        }

        if (clock.milliseconds() >= 1000 && clock.milliseconds() < 1200) {
            mDrive.claw.setPosition(0);


        }

        if (clock.milliseconds() >= 1200) {

            mDrive.Arm.setPower(0);


        }
    }

    }


    }



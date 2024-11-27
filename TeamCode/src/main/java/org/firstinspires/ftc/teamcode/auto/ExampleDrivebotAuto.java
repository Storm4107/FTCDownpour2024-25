package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperstructureSubsystem;



@Autonomous(name = "2025 - ExampleDrivebotAuto", group = "Autonomous")
public class ExampleDrivebotAuto extends LinearOpMode {
    //Instantiate mechanisms




    public SuperstructureSubsystem m_Superstructure;
    private MecanumDriveSubsystem m_Drive;




    @Override
    public void runOpMode() {



        //Run when initializing
        m_Superstructure = new SuperstructureSubsystem(hardwareMap, telemetry);
        m_Drive = new MecanumDriveSubsystem(hardwareMap, telemetry);




        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
            telemetry.addData("Auto", "Selected");
        }
        waitForStart();



        if (isStopRequested()) return;


        m_Superstructure.OpeningExtend();
        m_Superstructure.pincher.close();
        m_Superstructure.pincher.bucketHome();
        m_Superstructure.setAutoPosition(1000, 5);
        //m_Superstructure.pincher.open();
        sleep(50000);

        //Put auto steps here
        //m_Drive.AutoDriveRC(12, 0, 4);
        //m_Drive.AutoDriveRC(0, 5, 4);
      //  Elevator.setInches(3);
       // m_Superstructure.pincher.setPivotAngle(6);
        //m_Superstructure.Elevator.setInches(5);

        m_Drive.SetHeading(90, 3);


        /*/Drive the robot forward 1 foot.
        m_Drive.AutoDriveRC(0, 12, 5);
        //Drive the robot Left 1 foot.
        m_Drive.AutoDriveRC(-12, 0, 5);
        //Drive the robot backward 1 foot.
        m_Drive.AutoDriveRC(12, -12, 5);
        //Drive the robot right 1 foot.
        m_Drive.SetHeading(90, 3);
        //Set heading to 90 degrees*/

        //sleep(5000000);

    }
}

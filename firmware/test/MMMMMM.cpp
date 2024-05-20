#include <Arduino.h>
#include <Robot.h>

Robot r;

// Harvest Variable
bool ISGripUP = false, ISGripSlide = false, IS_13_Keep = false, IS_24_Keep = false;

// Ball Varialbe
int BallSpinPower = 255;
bool ISBallSpin = false, GotBall = false, ChargeBall = false, ArmUp = true;
unsigned long MacroTime = 0;

// Gamepad Variable
bool Logo_Pressed = false, Y_Pressed = false, K_Pressed = false,
     X_Pressed = false, B_Pressed = false, A_Pressed = false, M_Pressed = false;

void Move()
{
}

void MoveIMU()
{
}

void Slide_Transform()
{
    bool b = gamepad.B;

    if (!b)
    {
        B_Pressed = false;
        return;
    }
    if (B_Pressed)
        return;
    B_Pressed = true;
    if (!ISGripSlide)
    {
        digitalWrite(GripSlide, LOW);
        ISGripSlide = true;
        return;
    }
    digitalWrite(GripSlide, HIGH);
    ISGripSlide = false;
}

void UpDown_Transform()
{
    bool y = gamepad.Y;

    if (!y)
    {
        Y_Pressed = false;
        return;
    }
    if (Y_Pressed)
        return;
    Y_Pressed = true;
    if (!ISGripUP)
    {
        digitalWrite(GripUP, LOW);
        ISGripUP = true;
        return;
    }
    digitalWrite(GripUP, HIGH);
    ISGripUP = false;
}

void Keep_Harvest()
{
    bool lb = gamepad.leftBumper;
    bool rb = gamepad.rightBumper;
    bool lt = gamepad.leftTrigger > 0.3;
    bool rt = gamepad.rightTrigger > 0.3;
    if (!(lb || rb || rt || lt))
    {
        K_Pressed = false;
        return;
    }
    if (K_Pressed)
        return;
    if (lb && rb)
    {
        Grip1.write(49);
        Grip2.write(112);
        Grip3.write(111);
        Grip4.write(55);
        IS_13_Keep = IS_24_Keep = true;
        delay(300);
        return;
    }
    if (IS_13_Keep)
    {
        if (lb || lt)
        {
            if (lt)
            {
                Grip1.write(25);
                Grip3.write(86);
                delay(500);
            }
            Grip1.write(0);
            Grip3.write(0);
            IS_13_Keep = false;
            return;
        }
    }
    if (IS_24_Keep)
    {
        if (rb || rt)
        {
            if (rt)
            {
                Grip2.write(90);
                Grip4.write(30);
                delay(500);
            }
            Grip2.write(0);
            Grip4.write(0);
            IS_24_Keep = false;
            return;
        }
    }
}

void Keep_Ball()
{
    bool a = gamepad.A;
    unsigned long MTime = CurrentTime - MacroTime;
    if (MTime > 1000 && GotBall && !ChargeBall)
    {
        BallUP_DOWN.write(155);
        ArmUp = true;
        if (MTime > 2500)
        {
            BallUP_DOWN.write(125);
            if (MTime > 3000)
            {
                r.BallSpin(BallSpinPower);
                ISBallSpin = true;
                ChargeBall = true;
            }
        }
    }

    if (!a)
    {
        A_Pressed = false;
        return;
    }
    if (A_Pressed)
        return;
    A_Pressed = true;
    if (!GotBall && !ArmUp)
    {
        MacroTime = CurrentTime;
        BallLeftGrip.write(95);
        BallRightGrip.write(87);
        GotBall = true;
        return;
    }
    r.BallSpin(0);
    BallUP_DOWN.write(75);
    delay(400);
    BallLeftGrip.write(0);
    BallRightGrip.write(180);
    ArmUp = false;
    GotBall = false;
    ChargeBall = false;
}

void AdjustArm()
{
    bool x = gamepad.X;

    if (!x)
    {
        X_Pressed = false;
        return;
    }
    if (X_Pressed)
        return;
    X_Pressed = true;
    if (!ArmUp && !ChargeBall)
    {
        BallUP_DOWN.write(175);
        ArmUp = true;
        return;
    }
    if (ArmUp && !ChargeBall)
    {
        BallUP_DOWN.write(75);
        BallLeftGrip.write(0);
        BallRightGrip.write(180);
        ArmUp = false;
        BallUP_DOWN.write(75);
        BallLeftGrip.write(0);
        BallRightGrip.write(180);
        ArmUp = false;
        return;
    }
}

void Kick_Ball()
{
    bool x = gamepad.X;

    if (ChargeBall && ArmUp && x)
    {
        BallUP_DOWN.write(180);
        delay(500);
        r.BallSpin(0);
        BallUP_DOWN.write(75);
        delay(500);
        BallLeftGrip.write(0);
        BallRightGrip.write(180);
        ArmUp = false;
        ChargeBall = false;
        GotBall = false;
        return;
    }
}

void imu()
{
    yaw = ToRadians(IMUyaw);

    if (gamepad.screen)
        IMUyaw = 0;

    bool m = gamepad.menu;
    if (!m)
    {
        M_Pressed = false;
        return;
    }
    if (M_Pressed)
        return;
    M_Pressed = true;
    if (!UseIMU)
    {
        UseIMU = true;
        c.reset();
        return;
    }
    UseIMU = false;
}

void RobotSpinFaster()
{
    bool y = gamepad.Y;
    bool b = gamepad.B;
    int TSpeed = 255;
    if (y)
    {
        motor1Speed = -TSpeed;
        motor2Speed = -TSpeed;
        motor3Speed = -TSpeed;
    }
    if (b)
    {
        motor1Speed = TSpeed;
        motor2Speed = TSpeed;
        motor3Speed = TSpeed;
    }
}

// void SendDataToESP(){
//     int data = !ISBallSpin         ? 0 :
//                BallSpinPower > 200 ? 1 :
//                BallSpinPower > 150 ? 2 : 3;
//     if(ISBallSpin) t.WriteData((String)(data));
// }

void setup()
{
    r.init();
}

void loop()
{
    delay(40);
    CurrentTime = millis();
    r.loop(100);
    EmergencyStart();
    // SendDataToESP();
    if (gamepad.haveDataFromController && !EmergencyCutFromController)
    {
        EmergencyStop();
        imu();
        if (UseIMU)
        {
            MoveIMU();
            Slide_Transform();
            UpDown_Transform();
        }
        else
        {
            Move();
            RobotSpinFaster();
        }
        Keep_Harvest();
        AdjustArm();
        if (!(ISGripSlide || ISGripUP))
        {
            Keep_Ball();
            Kick_Ball();
        }
        r.MovePower(static_cast<int>(motor1Speed),
                    static_cast<int>(motor2Speed),
                    static_cast<int>(motor3Speed));
        return;
    }
    r.MovePower(0, 0, 0);
    r.BallSpin(0);
}
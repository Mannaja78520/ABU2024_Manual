#include <Arduino.h>
#include <Robot.h>

Robot r;
Controller c(0.8, 0.05, 0, 0);

// Movement Variable
//Speed is length 0-1 assum as 1-100%
float NormalSpeed = 1, trunSpeed = 0.3;
float maxSpeed = 255.0;
float setpoint = 0;
bool  r_disable = false, UseIMU = true;
float yaw;

float motor1Speed, motor2Speed, motor3Speed;

// Harvest Variable
bool ISGripUP = false, ISGripSlide = false, IS_13_Keep = false, IS_24_Keep = false;

// Ball Varialbe
int BallSpinPower = 255;
bool ISBallSpin = false, GotBall = false, ChargeBall = false, ArmUp = true;
unsigned long MacroTime = 0;

// Gamepad Variable
bool Logo_Pressed = false, Y_Pressed = false, K_Pressed = false,
     X_Pressed = false, B_Pressed = false, A_Pressed = false, M_Pressed = false;

// Analysis Variable
unsigned long CurrentTime = millis(), LastTime = millis(), lastRXTime = millis();

// Emergency Varible
bool Emergency_Pressed = false, EmergencyCutFromController = false;

void EmergencyStart(){
    bool l = gamepad.logo;
    
    if (!l){
        Emergency_Pressed = false;
        return;
    }
    if (Emergency_Pressed) return;
    Emergency_Pressed = true;

    if (EmergencyCutFromController) {
        r_disable = false, UseIMU = false;
        ISGripUP = ISGripSlide = IS_13_Keep = IS_24_Keep = false;
        ISBallSpin = GotBall = ChargeBall = false;
        Logo_Pressed = Y_Pressed = K_Pressed = X_Pressed = B_Pressed = A_Pressed = false;
        EmergencyCutFromController = false;
        return;
    }
}

void EmergencyStop() {
    bool lsd = gamepad.leftStickButton;
    bool rsd = gamepad.rightStickButton;
    if (lsd && rsd && !EmergencyCutFromController) {
        EmergencyCutFromController = true;
        return;
    }
}

void Move(){

    float lx =  gamepad.lx * NormalSpeed;
    float ly = -gamepad.ly * NormalSpeed;
    float rx =  gamepad.rx * trunSpeed;

    lx = gamepad.Dpad_left  ? -0.75 : 
         gamepad.Dpad_right ?  0.75 : lx;
    ly = gamepad.Dpad_down  ? -0.75 :
         gamepad.Dpad_up    ?  0.75 : ly;

    setpoint = yaw;

    float D = max(abs(lx)+abs(ly)+abs(rx), 1.0);

    float angle1 = 0.0;
    float angle2 = 2.0 * PI / 3.0;
    float angle3 = -2.0 * PI / 3.0;

    motor1Speed = (lx * cos(angle1) + ly * sin(angle1) + rx) / D * maxSpeed;
    motor2Speed = (lx * cos(angle2) + ly * sin(angle2) + rx) / D * maxSpeed;
    motor3Speed = (lx * cos(angle3) + ly * sin(angle3) + rx) / D * maxSpeed;
}

void MoveIMU(){
    float lx  =  gamepad.lx * NormalSpeed;
    float ly  = -gamepad.ly * NormalSpeed;
    float rx  =  gamepad.rx * trunSpeed;

    lx = gamepad.Dpad_left  ? -0.75 : 
         gamepad.Dpad_right ?  0.75 : lx;
    ly = gamepad.Dpad_down  ? -0.75 :
         gamepad.Dpad_up    ?  0.75 : ly;

    float x2  =  (cos(-yaw) * lx) - (sin(-yaw) * ly);
    float y2  =  (sin(-yaw) * lx) + (cos(-yaw) * ly);

    float R = c.Calculate(yaw - setpoint);
    // float R = rx;
    lastRXTime = rx != 0 ? CurrentTime : lastRXTime;
    if (rx != 0 ||  CurrentTime - lastRXTime < 300) {
        R = rx;
        setpoint = yaw;
    }

    // Serial.println(R);

    float D = max(abs(x2)+abs(y2)+abs(R), 1.0);

    float angle1 = 0.0;
    float angle2 = 2.0 * PI / 3.0;
    float angle3 = -2.0 * PI / 3.0;
    motor1Speed = x2 * cos(angle1) + y2 * sin(angle1) + R;
    motor2Speed = x2 * cos(angle2) + y2 * sin(angle2) + R;
    motor3Speed = x2 * cos(angle3) + y2 * sin(angle3) + R;

    motor1Speed = (motor1Speed / D) * maxSpeed;
    motor2Speed = (motor2Speed / D) * maxSpeed;
    motor3Speed = (motor3Speed / D) * maxSpeed;

}

void Slide_Transform(){
    bool b = gamepad.B;
    
    if (!b) {
        B_Pressed = false;
        return;
    }
    if (B_Pressed) return;
    B_Pressed = true;
    if(!ISGripSlide){
        digitalWrite(GripSlide, LOW);
        ISGripSlide = true;
        return;
    }
    digitalWrite(GripSlide, HIGH);
    ISGripSlide = false;
}

void UpDown_Transform(){
    bool y = gamepad.Y;
    
    if (!y) {
        Y_Pressed = false;
        return;
    }
    if (Y_Pressed) return;
    Y_Pressed = true;
    if (!ISGripUP){
        digitalWrite(GripUP, LOW);
        ISGripUP = true;
        return;
    }
    digitalWrite(GripUP, HIGH);
    ISGripUP = false;
}

void Keep_Harvest(){
    bool lb = gamepad.leftBumper;
    bool rb = gamepad.rightBumper;
    bool lt = gamepad.leftTrigger  > 0.3;
    bool rt = gamepad.rightTrigger > 0.3;
    if (!(lb || rb || rt || lt)){
        K_Pressed = false;
        return;
    }
    if (K_Pressed) return;
    if (lb && rb){
        Grip1.write(49);
        Grip2.write(112);
        Grip3.write(111);
        Grip4.write(55);setpoint = yaw;
        IS_13_Keep = IS_24_Keep = true;
        delay(300);
        return;
    }
    if (IS_13_Keep) {
        if (lb || lt) {
            if (lt) {
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
    if (IS_24_Keep) {
        if (rb || rt) {
            if (rt){
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

void Keep_Ball(){
    bool a = gamepad.A;
    unsigned long MTime = CurrentTime - MacroTime;
    if (MTime > 1000 && GotBall && !ChargeBall){
        BallUP_DOWN.write(155);
        ArmUp = true;
        if(MTime > 2500){
            BallUP_DOWN.write(125);
            if(MTime > 3000){
                r.BallSpin(BallSpinPower);
                ISBallSpin = true;
                ChargeBall = true;
            }
        }
    }
    
    if (!a) {
        A_Pressed = false;
        return;
    }
    if (A_Pressed) return;
    A_Pressed = true;
    if(!GotBall && !ArmUp){
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

void AdjustArm(){
    bool x = gamepad.X; 
  
    if (!x) {
        X_Pressed = false;
        return;
    }
    if (X_Pressed) return;
    X_Pressed = true;
    if (!ArmUp && !ChargeBall){
        BallUP_DOWN.write(175);
        ArmUp = true;
        return;
    }
    if(ArmUp && !ChargeBall){
        BallUP_DOWN.write(75);
        BallLeftGrip.write(0);
        BallRightGrip.write(180);
        ArmUp = false;
        return;
    }
}

void Kick_Ball(){
    bool x = gamepad.X; 

    if (ChargeBall && ArmUp && x){
        BallUP_DOWN.write(180);
        delay(500);
        r.BallSpin(0);
        BallUP_DOWN.write(75);
        delay(500);
        BallLeftGrip.write(0);
        BallRightGrip.write(180);
        ArmUp = false;
        ChargeBall = false;
        GotBall    = false;
        return;
    }
}

void imu(){
    yaw =  ToRadians(IMUyaw);

    if (gamepad.screen) IMUyaw = 0;

    bool m = gamepad.menu;
    if (!m) {
        M_Pressed = false;
        return;
    }
    if (M_Pressed) return;
    M_Pressed = true;
    if(!UseIMU){
        UseIMU = true;
        c.reset();
        return;
    }
    UseIMU = false;

}

void RobotSpinFaster(){
    bool y = gamepad.Y;
    bool b = gamepad.B;
    int TSpeed = 255;
    if (y) {
        motor1Speed = -TSpeed;
        motor2Speed = -TSpeed;
        motor3Speed = -TSpeed;
    }
    if (b) {
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

void setup(){
    r.init();
}

void loop(){
    delay(40);
    CurrentTime = millis();
    r.loop(100);
    EmergencyStart();
    // SendDataToESP();
    if (gamepad.haveDataFromController && !EmergencyCutFromController){
        EmergencyStop();
        imu();
        if(UseIMU) {
            MoveIMU();
            Slide_Transform();
            UpDown_Transform();
        } 
        else {
            Move();
            RobotSpinFaster();
        }
        Keep_Harvest();
        AdjustArm(); 
        if (!(ISGripSlide || ISGripUP)){
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
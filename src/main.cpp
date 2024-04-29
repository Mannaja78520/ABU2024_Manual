#include <Arduino.h>
#include <Robot.h>

Robot r;

// Movement Variable
//Speed is length 0-1 assum as 1-100%
float NormalSpeed = 0.9, trunSpeed = 0.4;
float maxSpeed = 255.0;
float setpoint = 0;
bool  r_disable = false, UseIMU = true;

// Harvest Variable
bool ISGripUP = false, ISGripSlide = false, IS_13_Keep = false, IS_24_Keep = false;

// Ball Varialbe
int BallSpinPower = 255;
bool ISBallSpin = false, GotBall = false, ChargeBall = false;
unsigned long MacroTime = 0;

// Gamepad Variable
bool Logo_Pressed = false, Y_Pressed = false, K_Pressed = false, 
     X_Pressed = false, B_Pressed = false, A_Pressed = false, M_Pressed = false;

// Analysis Variable
unsigned long CurrentTime = millis(), LastTime = millis();

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

    float D = max(abs(lx)+abs(ly)+abs(rx), 1.0);

    float angle1 = 0.0;
    float angle2 = 2.0 * PI / 3.0;
    float angle3 = -2.0 * PI / 3.0;

    float motor1Speed = (lx * cos(angle1) + ly * sin(angle1) + rx) / D * maxSpeed;
    float motor2Speed = (lx * cos(angle2) + ly * sin(angle2) + rx) / D * maxSpeed;
    float motor3Speed = (lx * cos(angle3) + ly * sin(angle3) + rx) / D * maxSpeed;

    r.MovePower(static_cast<int>(motor1Speed),
                static_cast<int>(motor2Speed),
                static_cast<int>(motor3Speed));
}

void MoveIMU(){
    float lx  =  gamepad.lx * maxSpeed;
    float ly  = -gamepad.ly * maxSpeed;
    float rx  =  gamepad.rx * trunSpeed;
    float yaw =  ToRadians(IMUyaw);
    float x2  =  (cos(-yaw) * lx) - (sin(-yaw) * ly);
    float y2  =  (sin(-yaw) * lx) + (cos(-yaw) * ly);
    
    float R = rx; 
    
    if (rx != 0){
        R = rx;
        setpoint = yaw;
    }

    float D = max(abs(x2)+abs(y2)+abs(R), 1.0);

    float angle1 = 0.0;
    float angle2 = 2.0 * PI / 3.0;
    float angle3 = -2.0 * PI / 3.0;
    float motor1Speed = x2 * cos(angle1) + y2 * sin(angle1);
    float motor2Speed = x2 * cos(angle2) + y2 * sin(angle2);
    float motor3Speed = x2 * cos(angle3) + y2 * sin(angle3);

    motor1Speed = (motor1Speed + R) / D * maxSpeed;
    motor2Speed = (motor2Speed + R) / D * maxSpeed;
    motor3Speed = (motor3Speed + R) / D * maxSpeed;

    r.MovePower(static_cast<int>(motor1Speed),
                static_cast<int>(motor2Speed),
                static_cast<int>(motor3Speed));

}

void Slide_Transform(){
    bool b = gamepad.B;
    Serial.println(ISGripSlide);
    
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

    if (!(lb || rb)){
        K_Pressed = false;
        return;
    }
    if (K_Pressed) return;
    if (lb && rb){
        Grip1.write(123);
        Grip2.write(112);
        Grip3.write(111);
        Grip4.write(115);
        IS_13_Keep = IS_24_Keep = true;
        delay(300);
        return;
    }
    if (IS_13_Keep && rb){
        Grip1.write(0);
        Grip3.write(0);
        IS_13_Keep = false;
        return;
    }
    if (IS_24_Keep && lb){
        Grip2.write(0);
        Grip4.write(0);
        IS_24_Keep = false;
        return;
    }
}

// void Keep_Ball(){
//     bool a = gamepad.A;
//     unsigned long MTime = CurrentTime - MacroTime;
//     if (MTime > 1000 && GotBall && !ChargeBall){
//         BallUP_DOWN.write(97);
//         if(MTime > 2500){
//             BallUP_DOWN.write(60);
//             if(MTime > 3500){
//                 // r.BallSpin(BallSpinPower);
//                 ISBallSpin = true;
//                 ChargeBall = true;
//             }
//         }
//     }
    
//     if (!a) {
//         A_Pressed = false;
//         return;
//     }
//     if (A_Pressed) return;
//     A_Pressed = true;
//     if(!GotBall){
//         MacroTime = CurrentTime;
//         BallLeftGrip.write(89);
//         BallRightGrip.write(91);
//         GotBall = true;
//         return;
//     }
//     BallLeftGrip.write(0);
//     BallRightGrip.write(180);
//     BallUP_DOWN.write(0);
//     r.BallSpin(0);
//     GotBall = false;
// }

// void Kick_Ball(){
//     bool x = gamepad.X; 
  
//     if (!x) {
//         X_Pressed = false;
//         return;
//     }
//     if (X_Pressed) return;
//     X_Pressed = true;
//     if (ChargeBall){
//         BallUP_DOWN.write(97);
//         delay(300);
//         r.BallSpin(0);
//         BallLeftGrip.write(0);
//         BallRightGrip.write(180);
//         BallUP_DOWN.write(0);
//     }
// }

// void SpinBall(){
//     bool x = gamepad.X; 
  
//     if (!x) {
//         X_Pressed = false;
//         return;
//     }
//     if (X_Pressed) return;
//     X_Pressed = true;
//     if (!ISBallSpin){
//         ISBallSpin = true;
//         r.BallSpin(BallSpinPower);
//         return;
//     }
//     r.BallSpin(0);
//     ISBallSpin = false;
//     ChargeBall = false;
// }
// Keep_Ball(){
//     bool a = gamepad.A;
//     unsigned long MTime = CurrentTime - MacroTime;
//     if (MTime > 1000 && GotBall && !ChargeBall){
//         BallUP_DOWN.write(97);
//         if(MTime > 2500){
//             BallUP_DOWN.write(60);
//             if(MTime > 3500){
//                 // r.BallSpin(BallSpinPower);
//                 ISBallSpin = true;
//                 ChargeBall = true;
//             }
//         }
//     }
    
//     if (!a) {
//         A_Pressed = false;
//         return;
//     }
//     if (A_Pressed) return;
//     A_Pressed = true;
//     if(!GotBall){
//         MacroTime = CurrentTime;
//         BallLeftGrip.write(89);
//         BallRightGrip.write(91);
//         GotBall = true;
//         return;
//     }
//     BallLeftGrip.write(0);
//     BallRightGrip.write(180);
//     BallUP_DOWN.write(0);
//     r.BallSpin(0);
//     GotBall = false;
// }

// void Kick_
void imu(){
    // if (gamepad.screen) {
    //     IMUyaw = 0; 
    //     UseIMU = true;
    // }else if(gamepad.menu) UseIMU = true;

    if (gamepad.Dpad_down) IMUyaw = 0;

    bool m = gamepad.menu;
    if (!m) {
        M_Pressed = false;
        return;
    }
    if (M_Pressed) return;
    M_Pressed = true;
    if(!UseIMU){
        UseIMU = true;
        return;
    }
    UseIMU = false;

}

void SendData(){
    int data[] = {UseIMU, ISGripUP, ISGripSlide, IS_13_Keep, IS_24_Keep, ISBallSpin, GotBall, ChargeBall, EmergencyCutFromController};
    // t.WriteData((String)(data));
}

void setup(){
    r.init();
    t.init();
}

void loop(){
    CurrentTime = millis();
    r.loop();
    EmergencyStart();
    SendData();
    // Serial.println(EmergencyCutFromController);
    if (gamepad.haveDataFromController && !EmergencyCutFromController){
        EmergencyStop();
        imu();
        if(UseIMU) MoveIMU(); 
        else Move();
        Slide_Transform();
        UpDown_Transform();
        Keep_Harvest();
        // if (!(ISGripSlide || ISGripUP)){
        //     Keep_Ball();
        //     Kick_Ball();
        // }
        return;
    }
    r.MovePower(0, 0, 0);
    // r.BallSpin(0);
    // // delay(10);
}
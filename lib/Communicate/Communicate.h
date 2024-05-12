#ifndef COMMUNICATE_H
#define COMMUNICATE_H

#include <Arduino.h>
#include <HardwareSerial.h>

class DataFromZigbeeJoystickPlayStation {
private:
    unsigned long LastTime = 0, LastDataTime = 0, DataSameAsLastTime = 0;
    
    float Last_lx, Last_ly, Last_rx, Last_ry, Last_rt, Last_lt;
    bool Last_A, Last_B, Last_X, Last_Y, Last_lb, Last_rb, Last_ltd, Last_rtd, Last_s, Last_m, Last_f, Last_lsb, Last_rsb;
    bool Last_dr, Last_dl, Last_du, Last_dd;
    
    void resetVariables() {
        lx = ly = rx = ry = rightTrigger = leftTrigger = 0.0;
        A = B = X = Y = leftBumper = rightBumper = leftTriggerDigital = rightTriggerDigital = screen = menu = logo = leftStickButton = rightStickButton = false;
        Dpad_right = Dpad_left = Dpad_up = Dpad_down = false;
        right_left_Dpad = up_down_Dpad = 0;
        Last_lx = Last_ly = Last_rx = Last_ry =  Last_rt =  Last_lt = 0.0;
        Last_A = Last_B = Last_X = Last_Y = Last_lb = Last_rb = Last_ltd = Last_rtd = Last_s = Last_m = Last_f = Last_lsb = Last_rsb = false;
        Last_dr = Last_dl = Last_du = Last_dd = false;
    }

    void UpdateLastData() {
    Last_lx = lx; 
    Last_ly = ly; 
    Last_rx = rx; 
    Last_ry = ry; 
    Last_rt = rightTrigger; 
    Last_lt = leftTrigger;
    
    Last_A = A; 
    Last_B = B; 
    Last_X = X; 
    Last_Y = Y; 
    Last_lb = leftBumper; 
    Last_rb = rightBumper; 
    Last_ltd = leftTriggerDigital; 
    Last_rtd = rightTriggerDigital; 
    Last_s = screen; 
    Last_m = menu; 
    Last_f = logo; 
    Last_lsb = leftStickButton; 
    Last_rsb = rightStickButton;
    
    Last_dr = Dpad_right; 
    Last_dl = Dpad_left; 
    Last_du = Dpad_up; 
    Last_dd = Dpad_down;
    }

    void UseLastData() {
    lx = Last_lx; 
    ly = Last_ly; 
    rx = Last_rx; 
    ry = Last_ry; 
    rightTrigger = Last_rt; 
    leftTrigger = Last_lt;

    A = Last_A; 
    B = Last_B; 
    X = Last_X; 
    Y = Last_Y; 
    leftBumper = Last_lb; 
    rightBumper = Last_rb; 
    leftTriggerDigital = Last_ltd; 
    rightTriggerDigital = Last_rtd; 
    screen = Last_s; 
    menu = Last_m; 
    logo = Last_f; 
    leftStickButton = Last_lsb; 
    rightStickButton = Last_rsb;

    Dpad_right = Last_dr; 
    Dpad_left = Last_dl; 
    Dpad_up = Last_du; 
    Dpad_down = Last_dd;
}

void compareData(unsigned long CurrentTime) {
    if (lx == Last_lx && ly == Last_ly && rx == Last_rx && ry == Last_ry && 
        rightTrigger == Last_rt && leftTrigger == Last_lt) {
        if (CurrentTime - DataSameAsLastTime > 1250){
            resetVariables();
            haveDataFromController = 0; 
            return;
        }
    }
    DataSameAsLastTime = CurrentTime;
}

public:
    int checksum;
    float lx, ly, rx, ry, rightTrigger, leftTrigger;
    bool A, B, X, Y, leftBumper, rightBumper, leftTriggerDigital, rightTriggerDigital, screen, menu, logo, leftStickButton, rightStickButton;
    bool Dpad_right, Dpad_left, Dpad_up, Dpad_down;
    int right_left_Dpad, up_down_Dpad;
    int haveDataFromController;

    DataFromZigbeeJoystickPlayStation() {
        haveDataFromController = 0;
        resetVariables();
    }

    void init() {
        Serial1.begin(115200);
    }

    void readData() {
        unsigned long CurrentTime = millis();
        // Serial1.println(String(CurrentTime - LastTime));
        // LastTime = CurrentTime;
        static const int expected_data_length = 40;  // Adjust as per your data format
        haveDataFromController = Serial1.available();
        // Serial.println(haveDataFromController);
        if (haveDataFromController >= expected_data_length) {
            LastDataTime = CurrentTime;
            String receivedData = Serial1.readStringUntil('\n');
            // Serial.println("Received data: " + receivedData);
            // Serial.println(receivedData.length());
            if (receivedData.length() >= expected_data_length) {
                char* token;
                char* ptr = const_cast<char*>(receivedData.c_str());
                int index = 0;
                int sumOfData = 0;
                while ((token = strtok_r(ptr, ",", &ptr)) != NULL && index < 22) {
                    int data = atoi(token);
                    sumOfData += abs(data);
                    switch (index) {
                        case 0: checksum = data; sumOfData = 0; break;
                        case 1: lx = data / 100.0; break;
                        case 2: ly = data / 100.0; break;
                        case 3: rx = data / 100.0; break;
                        case 4: ry = data / 100.0; break;
                        case 5: rightTrigger = data / 100.0; break;
                        case 6: leftTrigger = data / 100.0; break;
                        case 7: A = data; break;
                        case 8: B = data; break;
                        case 9: X = data; break;
                        case 10: Y = data; break;
                        case 11: leftBumper = data; break;
                        case 12: rightBumper = data; break;
                        case 13: leftTriggerDigital = data; break;
                        case 14: rightTriggerDigital = data; break;
                        case 15: screen = data; break;
                        case 16: menu = data; break; 
                        case 17: logo = data; break;
                        case 18: leftStickButton = data; break;
                        case 19: rightStickButton = data; break;
                        case 20: right_left_Dpad = data; break;
                        case 21: up_down_Dpad = data; break;
                    }
                    index++;
                }
                // Discard any remaining data in the buffer
                while (Serial1.available() > 0) {
                    Serial1.read();
                }
                int receivedChecksum = abs(sumOfData - checksum);
                if (receivedChecksum == 0) {
                    // Debugging: Print received data
                    // Serial.println("Data Match");
                    Dpad_right = right_left_Dpad ==  1;
                    Dpad_left  = right_left_Dpad == -1;
                    Dpad_up    = up_down_Dpad    ==  1;
                    Dpad_down  = up_down_Dpad    == -1;
                    // compareData(CurrentTime);
                    UpdateLastData();
                    return;
                }
                Serial.println("Checksum mismatch!");
                Serial1.readStringUntil('\n'); // Discard the incomplete data
                UseLastData();
                return;
            } 
            Serial.println("Incomplete data received!");
            Serial1.readStringUntil('\n'); // Discard the incomplete data
            UseLastData();
            return;
        }
        // Serial.println("No data received!");
        if(CurrentTime - LastDataTime < 100){
            haveDataFromController = 1;
            UseLastData();
            return;
        }
        haveDataFromController = 0;
        resetVariables();
        //// Serial.println(CurrentTime - LastDataTime);
        // UseLastData();
    }
};

class DataFromZigbeeJoystickXbox {
private:
    int BaudRate;
    unsigned long i = 0;
    unsigned long LastTime = 0, LastDelayTime = 0, LastDataTime = 0, DataSameAsLastTime = 0;

    bool flowControlEnabled = true, lastHaveControl = false;

    float Last_lx, Last_ly, Last_rx, Last_ry, Last_rt, Last_lt;
    bool Last_A, Last_B, Last_X, Last_Y, Last_lb, Last_rb, Last_s, Last_m, Last_f, Last_lsb, Last_rsb, Last_u;
    bool Last_dr, Last_dl, Last_du, Last_dd;
    
    void resetVariables() {
        lx = ly = rx = ry = rightTrigger = leftTrigger = 0.0;
        A = B = X = Y = leftBumper = rightBumper = screen = menu = logo = leftStickButton = rightStickButton = false;
        Dpad_right = Dpad_left = Dpad_up = Dpad_down = false;
        right_left_Dpad = up_down_Dpad = 0;
    }

    void resetLastVariable(){
        Last_lx = Last_ly = Last_rx = Last_ry = Last_rt = Last_lt = 0.0;
        Last_A = Last_B = Last_X = Last_Y = Last_lb = Last_rb = Last_s = Last_m = Last_f = Last_lsb = Last_rsb = Last_u = false;
        Last_dr = Last_dl = Last_du = Last_dd = false;
        i = 0;
        LastTime = LastDelayTime = LastDataTime = DataSameAsLastTime = 0;
    }

    void UpdateLastData() {
    Last_lx = lx; 
    Last_ly = ly; 
    Last_rx = rx; 
    Last_ry = ry; 
    Last_rt = rightTrigger; 
    Last_lt = leftTrigger;
    
    Last_A = A; 
    Last_B = B; 
    Last_X = X; 
    Last_Y = Y; 
    Last_lb = leftBumper; 
    Last_rb = rightBumper;
    Last_s = screen; 
    Last_m = menu; 
    Last_f = logo; 
    Last_lsb = leftStickButton; 
    Last_rsb = rightStickButton;
    Last_u = upload;
    
    Last_dr = Dpad_right; 
    Last_dl = Dpad_left; 
    Last_du = Dpad_up; 
    Last_dd = Dpad_down;
    }

    void UseLastData() {
    lx = Last_lx; 
    ly = Last_ly; 
    rx = Last_rx; 
    ry = Last_ry; 
    rightTrigger = Last_rt; 
    leftTrigger = Last_lt;

    A = Last_A; 
    B = Last_B; 
    X = Last_X; 
    Y = Last_Y; 
    leftBumper = Last_lb; 
    rightBumper = Last_rb; 
    screen = Last_s; 
    menu = Last_m; 
    logo = Last_f; 
    leftStickButton = Last_lsb; 
    rightStickButton = Last_rsb;
    upload = Last_u;

    Dpad_right = Last_dr; 
    Dpad_left = Last_dl; 
    Dpad_up = Last_du; 
    Dpad_down = Last_dd;
    }

    void compareData(unsigned long CurrentTime) {
        if (lx == Last_lx && ly == Last_ly && rx == Last_rx && ry == Last_ry && 
            rightTrigger == Last_rt && leftTrigger == Last_lt) {
            if (CurrentTime - DataSameAsLastTime > 1250){
                resetVariables();
                haveDataFromController = 0; 
                return;
            }
        }
        DataSameAsLastTime = CurrentTime;
    }

    void resetCommunication() {
        // Stop ongoing transmission or reception
        Serial1.end();

        // Reset internal variables
        resetVariables();
        // Reset last data
        resetLastVariable();

        // Reinitialize communication
        Serial1.begin(115200);
    }

public:
    int checksum;
    float lx, ly, rx, ry, rightTrigger, leftTrigger;
    bool A, B, X, Y, leftBumper, rightBumper, screen, menu, logo, leftStickButton, rightStickButton, upload;
    bool Dpad_right, Dpad_left, Dpad_up, Dpad_down;
    int right_left_Dpad, up_down_Dpad;
    unsigned int haveDataFromController;

    DataFromZigbeeJoystickXbox() {
        haveDataFromController = 0;
        resetVariables();
    }

    void init() {
        Serial1.begin(115200);
    }

    void readData(unsigned int DelayData) {
        unsigned long CurrentTime = millis();
        // if (CurrentTime - LastDelayTime > DelayData) {
        //     // Serial1.println(1);
        //     LastDelayTime = CurrentTime;
        // }
        // else{
            static const int expected_data_length = 40;
            haveDataFromController = Serial1.available();
            if (haveDataFromController) {
                resetVariables();
                LastDataTime = CurrentTime;
                String receivedData = Serial1.readStringUntil('\n');
                // Serial.println("Received data: " + receivedData);
                // Serial.println(receivedData.length());
                if (receivedData.length() >= expected_data_length && receivedData.length() <= 80) {
                    LastTime = CurrentTime;
                    char* token;
                    char* ptr = const_cast<char*>(receivedData.c_str());
                    int index = 0;
                    int sumOfData = 0;
                    while ((token = strtok_r(ptr, ",", &ptr)) != NULL && index < 21) {
                        int data = atoi(token);
                        sumOfData += abs(data);
                        switch (index) {
                            case 0:  checksum = data; sumOfData = 0; break;
                            case 1:  lx = data / 100.0; break;
                            case 2:  ly = data / 100.0; break;
                            case 3:  leftTrigger = data / 100.0; break;
                            case 4:  rx = data / 100.0; break;
                            case 5:  ry = data / 100.0; break;
                            case 6:  rightTrigger = data / 100.0; break;
                            case 7:  A = data; break;
                            case 8:  B = data; break;
                            case 9:  X = data; break;
                            case 10: Y = data; break;
                            case 11: leftBumper = data; break;
                            case 12: rightBumper = data; break;
                            case 13: screen = data; break;
                            case 14: menu = data; break; 
                            case 15: logo = data; break;
                            case 16: leftStickButton = data; break;
                            case 17: rightStickButton = data; break;
                            case 18: upload = data; break;
                            case 19: right_left_Dpad = data; break;
                            case 20: up_down_Dpad = data; break;
                        }
                        index++;
                    }
                    // Discard any remaining data in the buffer
                    while (Serial1.available() > 0 && CurrentTime - LastTime < 50) {
                        CurrentTime = millis();
                        Serial1.read();
                    }
                    int receivedChecksum = abs(sumOfData - checksum);
                    if (receivedChecksum == 0) {
                        // Debugging: Print received data
                        // Serial.println("Data Match");
                        Dpad_right = right_left_Dpad ==  1;
                        Dpad_left  = right_left_Dpad == -1;
                        Dpad_up    = up_down_Dpad    ==  1;
                        Dpad_down  = up_down_Dpad    == -1;
                        // compareData(CurrentTime);
                        UpdateLastData();
                        return;
                    }
                    Serial.println("Checksum mismatch!");
                    // while (Serial1.available() > 0 && CurrentTime - LastTime < 50) {
                    //     CurrentTime = millis();
                    //     Serial1.read();
                    // }
                    Serial1.readStringUntil('\n');
                    UseLastData();
                    return;
                } 
                Serial.println("Incomplete data received!");
                // while (Serial1.available() > 0 && CurrentTime - LastTime < 50) {
                //         CurrentTime = millis();
                //         Serial1.read();
                //     }
                Serial1.readStringUntil('\n');
                UseLastData();
                return;
            }
        // }
    // Serial.println("No data received!");
        if(CurrentTime - LastDataTime < 300){
            haveDataFromController = 1;
            UseLastData();
            return;
        }
        haveDataFromController = 0;
        // lastHaveControl = true;
        resetVariables();
        //// Serial.println(CurrentTime - LastDataTime);
        // UseLastData();
    }
};

class TransferData {
public:
    String Data;

    void init() {
        Serial2.begin(9600);
    }

    void WriteData(String data) {
        Serial2.println(data);
    }
    
    void readData() {
        
        if (Serial2.available()) {
            Data = Serial2.readStringUntil('\n');
            while (Serial2.available() > 0) {
                Serial2.read();
            }
        }
    }
};

#endif
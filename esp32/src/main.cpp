#include <Arduino.h>
#include <motor.h>
#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message
{
    bool stop;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
bool run = false;
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    memcpy(&myData, incomingData, sizeof(myData));
    // Serial.println(myData.stop);
    run = !run;
}
void setup()
{
    Serial.begin(115200);
    pinMode(15, INPUT_PULLUP);
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);
    motorsetup();
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(OnDataRecv);
    // while (digitalRead(15));
    // delay(2000);
    
}

void loop()
{
    // motor2F(255, 255);
    // if (run){
    //     motor2F(255, 255);
    // }
    // else{
    //     motor2F(0, 0);
    // }
    // if (digitalRead(15) == 0){
    //     myData.stop = false;
    // }
    if (!myData.stop)
    {
        if (Serial.available() > 1)
        {
            byte speed1 = Serial.read();
            byte speed2 = Serial.read();
            motor2((speed1 - 127) * 2, (speed2 - 127) * 2);
        }
        // motor2F(255,255);
    }
    else
    {
        motor2(0, 0);
    }
}
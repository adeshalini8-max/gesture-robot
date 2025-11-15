#include <Wire.h>

#define PWMA 32
#define AIN1 25
#define AIN2 33
#define PWMB 14
#define BIN1 26
#define BIN2 27



#include <WiFi.h>

const char* ssid = "ESP32_AP";   // WiFi AP Name
const char* password = "12345678";  // WiFi Password (8+ characters)

WiFiServer server(81);  // TCP Server on port 80                           this is the backend code which we have uploaded in esp32 through usb via ardiuno app 




void setup() {

    Serial.begin(115200);

    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);


    pinMode(2, OUTPUT);

    Wire.begin();

    WiFi.softAP(ssid, password);
    Serial.println("ESP32 AP Started");
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());

    server.begin();


}





void moveForward(int leftSpeed, int rightSpeed) {
    if (leftSpeed > 0 && rightSpeed > 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
    } else if (leftSpeed < 0 && rightSpeed > 0) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
    } else if (leftSpeed > 0 && rightSpeed < 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    } else if (leftSpeed < 0 && rightSpeed < 0) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    }

    analogWrite(PWMA, abs(leftSpeed));
    analogWrite(PWMB, abs(rightSpeed));
}




void loop() {
    
    
    WiFiClient client = server.available();  // Wait for a client

    if (client) {
        Serial.println("Client Connected!");
        client.setTimeout(1000);
        while (client.connected()) {
          if (client.available()){
            
             String message = client.readStringUntil('\n');
             Serial.println(message);
              int sep = message.indexOf('|');
              if (sep != -1) {
                  int left = message.substring(0, sep).toInt();
                  int right = message.substring(sep + 1).toInt();
                  moveForward(left, right);
              }

            }
            
            
        }
            Serial.println("clint disconnected");

        client.stop();  // Close connection
    }
  delay(100);
}

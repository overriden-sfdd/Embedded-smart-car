#include <Arduino.h>
#include <LittleFS.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Replace with your network credentials.
const char* ssid = "YOUR SSID HERE";
const char* password = "YOUR PASSWORD HERE";

// Create AsyncWebServer object on port 80.
AsyncWebServer server(80);

// Replaces placeholder with button section in your web page.
String processor(const String& var)
{
    return String();
}

void setup()
{
    // Serial port for debugging purposes.
    Serial.begin(9600);
    // Start filing subsystem.
    LittleFS.begin();

    // Connect to Wi-Fi.
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(3000);
    }

    // Send ESP Local IP Address to display.
    Serial.println(WiFi.localIP());

    // Route for root / web page.
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/index.html", String(), false, processor);
        int paramsNr = request->params();
        for (byte i = 0; i < paramsNr; ++i) {
            AsyncWebParameter* p = request->getParam(0);
            // Put command in com_name:com_value\n format.
            Serial.write((p->name() + ':' + p->value() + '\n').c_str());
        }
        });

    // Route to load style.css file.
    server.on("/jquery-1.11.3.min.js", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/jquery-1.11.3.min.js", "text/javascript");
        });

    // Route to load style.css file.
    server.on("/roundslider.min.js", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/roundslider.min.js", "text/javascript");
        });

    // Route to load style.css file.
    server.on("/roundslider.min.css", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/roundslider.min.css", "text/css");
        });

    // Start server.
    server.begin();
}

void loop()
{

}
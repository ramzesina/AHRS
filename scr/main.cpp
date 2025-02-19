#include "common.hpp"
#include "tasks.hpp"

void setup()
{
    Serial.begin(250000000);

    pinInit();
    protocolInit();

    // Model init
    initializeModel();

    // Logger init
    initializeModelLogger();
    initializeSensorsLogger();
    initializeUtilityLogger();
    startLogger(); // Has to be done after SPI init (protocolInit)

    //Calibrate Sensor
    TASKS_CONFIG.taskCalib.enable = true;

    // Configure RTOS tasks
    rtosSetup();
}

void loop()
{
	delay(1000); // NOLINT
}

#pragma once
#include "config.h"
#include "BMSModule.h"
#include <PubSubClient.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <ArduinoJson.h>
//#include <esp32_can.h>

extern int numFoundModules;                    // The number of modules that seem to exist
extern float lowestCellVolt;
extern float highestCellVolt;
extern float lowestPackTemp;
extern float highestPackTemp;

class BMSModuleManager
{
public:
    BMSModuleManager(AsyncWebServer* webServer);
    //BMSModuleManager();
    void balanceCells();
    void balanceCell(int cellNumber);
    void setupBoards();
    void findBoards();
    void renumberBoardIDs();
    void clearFaults();
    void sleepBoards();
    void wakeBoards();
    void getAllVoltTemp();
    void readSetpoints();
    void setBatteryID();
    float getPackVoltage();
    float getAvgTemperature();
    float getAvgCellVolt();
    float getLowCellVolt();
    float getHighestModuleVolt();
    void printPackSummary();
    void printPackDetails();
    void printJsonData();
    String buildJsonData();
    void publishIndividualData(PubSubClient& client, const char* baseTopic, const String systemName);
    void handleBatteryStats(AsyncWebServerRequest* request, const String& bmsJson);
    void sendBatteryStats(String systemName, String ftpServer, String ftpUser, String ftpPassword, const String& bmsJson);
    void broadcastBatteryStats(AsyncWebSocket* ws, const String& bmsJson);

private:
    float packVolt;                         // All modules added together
    float lowestPackVolt;
    float highestPackVolt;
    void publishSensorData(PubSubClient& client, const char* baseTopic, const String systemName, const String& cellID, const String& devClass, const String& unit, const int precision, const String& value);
    BMSModule modules[MAX_MODULE_ADDR + 1]; // store data for as many modules as we've configured for.
    
    bool isFaulted;
    int CellsBalancing;
    AsyncWebServer* server;
    
    //void sendBatterySummary(String systemName, String ftpServer, String ftpUser, String ftpPassword);
    //void sendModuleSummary(int module);
    void sendCellDetails(int module, int cell);
    
};
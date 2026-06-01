#include "config.h"
#include "BMSModuleManager.h"
#include "BMSUtil.h"
#include "Logger.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <FS.h>              // File System support
//#include <LittleFS.h>
#include <ESP32_FTPClient.h> // FTP Client library

extern EEPROMSettings settings;
int numFoundModules = 0;
float lowestCellVolt;
float highestCellVolt;
float lowestPackTemp;
float highestPackTemp;


BMSModuleManager::BMSModuleManager(AsyncWebServer* webServer)
{
    for (int i = 1; i <= MAX_MODULE_ADDR; i++) {
        modules[i].setExists(false);
        modules[i].setAddress(i);
    }
    lowestPackVolt = 1000.0f;
    highestPackVolt = 0.0f;
    lowestPackTemp = 200.0f;
    highestPackTemp = -100.0f;
    isFaulted = false;
    server = webServer;
}

// void BMSModuleManager::balanceCells()
// {  
//   float lowestCell = 10.0f;
//   for (int x = 1; x <= MAX_MODULE_ADDR; x++) //start cycling thru packs
//     {
//         if (modules[x].isExisting()) //end the loop if we're out of packs
//         {            
//             modules[x].readModuleValues(); //get some data
//             if (modules[x].getLowCellV() < lowestCell) lowestCell = modules[x].getLowCellV(); //Determine lowest value of of a pack and set lowestCell
//             // Serial.println("In balanceCells() " + String(lowestCell, 3) + " Module " + String(x));
        
//             if ( x%2 == 0) //Run this code if we're on an even numbered pack (so we do this for every two packs i.e, each string
//             {
//               modules[x-1].balanceCells(lowestCell); //Balance to the lowestCell (potentially) for the previously number pack. Tolerance is 0.007f
//               modules[x].balanceCells(lowestCell); //Balance to the lowestCell (potentially) for the current pack
//               // Serial.println("In balanceCells() 2 " + String(lowestCell, 3) + " Module " + String(x));
//               lowestCell = 10.0f; //Reset lowestCell value for the next two packs
//             }
//         }
//     }
// }
void BMSModuleManager::balanceCells()
{  
    float lowestCell = 10.0f;
    for (int x = 1; x <= MAX_MODULE_ADDR; x++) // Start cycling through packs
    {
        if (modules[x].isExisting()) // Process only if the module exists
        {            
            modules[x].readModuleValues(); // Get module data

            if (x % 2 != 0)  // If this is an odd-numbered pack, initialize lowestCell tracking
            {
                lowestCell = modules[x].getLowCellV(); 
            }
            else  // If this is an even-numbered pack, complete the pair
            {
                float secondLowestCell = modules[x].getLowCellV();
                lowestCell = min(lowestCell, secondLowestCell); // Get the lowest voltage of the pair
                
                modules[x - 1].balanceCells(lowestCell); // Balance previous pack
                modules[x].balanceCells(lowestCell);     // Balance current pack
                
                lowestCell = 10.0f; // Reset for the next pair
            }
        }
    }
}


void BMSModuleManager::balanceCell(int cellNumber)
{  
    for (int address = 1; address <= MAX_MODULE_ADDR; address++)
    {
        if (modules[address].isExisting()) modules[address].balanceCell(cellNumber);
    }
}

/*
 * Try to set up any uninitialized boards. Send a command to address 0 and see if there is a response. If there is then there is
 * still at least one uninitialized board. Go ahead and give it the first ID not registered as already taken.
 * If we send a command to address 0 and no one responds then every board is inialized and this routine stops.
 * Don't run this routine until after the boards have already been enumerated.
 * Note: The 0x80 conversion it is looking might in theory block the message from being forwarded so it might be required
 * To do all of this differently. Try with multiple boards. The alternative method would be to try to set the next unused
 * address and see if any boards respond back saying that they set the address. 
 */
void BMSModuleManager::setupBoards()
{
    uint8_t payload[3];
    uint8_t buff[10];
    int retLen;

    payload[0] = 0;
    payload[1] = 0;
    payload[2] = 1;

    while (1 == 1)
    {
        payload[0] = 0;
        payload[1] = 0;
        payload[2] = 1;
        retLen = BMSUtil::sendDataWithReply(payload, 3, false, buff, 4);
        if (retLen == 4)
        {
            if (buff[0] == 0x80 && buff[1] == 0 && buff[2] == 1)
            {
                Logger::debug("00 found");
                //look for a free address to use
                for (int y = 1; y < 63; y++) 
                {
                    if (!modules[y].isExisting())
                    {
                        payload[0] = 0;
                        payload[1] = REG_ADDR_CTRL;
                        payload[2] = y | 0x80;
                        BMSUtil::sendData(payload, 3, true);
                        delay(3);
                        if (BMSUtil::getReply(buff, 10) > 2)
                        {
                            if (buff[0] == (0x81) && buff[1] == REG_ADDR_CTRL && buff[2] == (y + 0x80)) 
                            {
                                modules[y].setExists(true);
                                numFoundModules++;
                                Logger::debug("Address assigned");
                            }
                        }
                        break; //quit the for loop
                    }
                }
            }
            else
            {
              Logger::debug("nobody responded properly to the zero address so our work here is done.");
              break; //nobody responded properly to the zero address so our work here is done.
            } 
        }
        else break;
    }
}

void BMSModuleManager::findBoards()
{
    uint8_t payload[3];
    uint8_t buff[8];

    numFoundModules = 0;
    payload[0] = 0;
    payload[1] = 0; // read registers starting at 0
    payload[2] = 1; // read one byte

    int lastGood = 0;

    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        modules[x].setExists(false);
        payload[0] = x << 1;

        Logger::debug("Probing module address: %X", x);
        BMSUtil::sendData(payload, 3, false);
        delay(20);

        if (BMSUtil::getReply(buff, 8) > 4)
        {
            if (buff[0] == (x << 1) && buff[1] == 0 && buff[2] == 1 && buff[4] > 0)
            {
                modules[x].setExists(true);
                numFoundModules++;
                lastGood = x;
                Logger::debug("Found module with address: %X", x);
            }
            else
            {
                Logger::debug("Received malformed or incorrect reply from module %X", x);
                break;
            }
        }
        else
        {
            Logger::debug("No response from module address: %X", x);
            break; // Stop searching — likely break in chain
        }

        delay(5);
    }

    if (lastGood > 0)
    {
        Logger::info("Last responding module: %X", lastGood);
    }
    else
    {
        Logger::warn("No modules responded to findBoards()");
    }
}


/*
 * Force all modules to reset back to address 0 then set them all up in order so that the first module
 * in line from the master board is 1, the second one 2, and so on.
*/
void BMSModuleManager::renumberBoardIDs()
{
    uint8_t payload[3];
    uint8_t buff[8];
    int attempts = 1;

    for (int y = 1; y < 63; y++) 
    {
        modules[y].setExists(false);
        numFoundModules = 0;
    }

    while (attempts < 3)
    {
        payload[0] = 0x3F << 1; //broadcast the reset command
        payload[1] = 0x3C;//reset
        payload[2] = 0xA5;//data to cause a reset
        BMSUtil::sendData(payload, 3, true);
        delay(100);
        BMSUtil::getReply(buff, 8);
        if (buff[0] == 0x7F && buff[1] == 0x3C && buff[2] == 0xA5 && buff[3] == 0x57) break;
        attempts++;
    }

    setupBoards();
}

/*
After a RESET boards have their faults written due to the hard restart or first time power up, this clears thier faults
*/
void BMSModuleManager::clearFaults()
{
    uint8_t payload[3];
    uint8_t buff[8];
    payload[0] = 0x7F; //broadcast
    payload[1] = REG_ALERT_STATUS;//Alert Status
    payload[2] = 0xFF;//data to cause a reset
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    payload[0] = 0x7F; //broadcast
    payload[2] = 0x00;//data to clear
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    payload[0] = 0x7F; //broadcast
    payload[1] = REG_FAULT_STATUS;//Fault Status
    payload[2] = 0xFF;//data to cause a reset
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    payload[0] = 0x7F; //broadcast
    payload[2] = 0x00;//data to clear
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    isFaulted = false;
}

/*
Puts all boards on the bus into a Sleep state, very good to use when the vehicle is a rest state. 
Pulling the boards out of sleep only to check voltage decay and temperature when the contactors are open.
*/

void BMSModuleManager::sleepBoards()
{
    uint8_t payload[3];
    uint8_t buff[8];
    payload[0] = 0x7F; //broadcast
    payload[1] = REG_IO_CTRL;//IO ctrl start
    payload[2] = 0x04;//write sleep bit
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);
}

/*
Wakes all the boards up and clears thier SLEEP state bit in the Alert Status Registery
*/

void BMSModuleManager::wakeBoards()
{
    uint8_t payload[3];
    uint8_t buff[8];
    payload[0] = 0x7F; //broadcast
    payload[1] = REG_IO_CTRL;//IO ctrl start
    payload[2] = 0x00;//write sleep bit
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);
  
    payload[0] = 0x7F; //broadcast
    payload[1] = REG_ALERT_STATUS;//Fault Status
    payload[2] = 0x04;//data to cause a reset
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);
    payload[0] = 0x7F; //broadcast
    payload[2] = 0x00;//data to clear
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);
}

// void BMSModuleManager::getAllVoltTemp()
// {
//     packVolt = 0.0f;
//     lowestCellVolt = 4.3f;
//     highestCellVolt = 0.0f;
//     lowestPackTemp = 50.0f;
//     highestPackTemp = -10.0f;
//     for (int x = 1; x <= MAX_MODULE_ADDR; x++)
//     {
//     if (modules[x].isExisting())
//     {
//       modules[x].stopBalance();
//     }
//   }
//   delay(1000);
//     for (int x = 1; x <= MAX_MODULE_ADDR; x++)
//     {
//         if (modules[x].isExisting()) 
//         {
//             Logger::debug("");
//             Logger::debug("Module %i exists. Reading voltage and temperature values", x);
//             modules[x].readModuleValues();
//             Logger::debug("Module voltage: %f", modules[x].getModuleVoltage());
//             Logger::debug("Lowest Cell V: %f     Highest Cell V: %f", modules[x].getLowCellV(), modules[x].getHighCellV());
//             Logger::debug("Temp1: %f       Temp2: %f", modules[x].getTemperature(0), modules[x].getTemperature(1));
//             packVolt += modules[x].getModuleVoltage();
//             if (modules[x].getLowCellV() < lowestCellVolt) lowestCellVolt = modules[x].getLowCellV();
//             if (modules[x].getHighCellV() > highestCellVolt) highestCellVolt = modules[x].getHighCellV();
//             if (modules[x].getLowTemp() < lowestPackTemp) lowestPackTemp = modules[x].getLowTemp();
//             if (modules[x].getHighTemp() > highestPackTemp) highestPackTemp = modules[x].getHighTemp();            
//         }
//     }
//     // Debug summary
//     //Serial.println("");
//     //Serial.println("High temp: " + String(highestPackTemp));
//     //Serial.println("Low temp: " + String(lowestPackTemp));
//     //Serial.println("High volt: " + String(highestCellVolt));
//     //Serial.println("Low volt: " + String(lowestCellVolt));

//     if (packVolt > highestPackVolt) highestPackVolt = packVolt;
//     if (packVolt < lowestPackVolt) lowestPackVolt = packVolt;
// //You can uncomment this code if you do have the module fault chain attached. Change the pin number to where it is attached
// /*
//     if (digitalRead(13) == LOW) {
//         if (!isFaulted) Logger::error("One or more BMS modules have entered the fault state!");
//         isFaulted = true;
//     }
//     else
//     {
//         if (isFaulted) Logger::info("All modules have exited a faulted state");
//         isFaulted = false;
//     }
// */
// }

void BMSModuleManager::getAllVoltTemp()
{
    packVolt = 0.0f;
    lowestCellVolt = 4.3f;
    highestCellVolt = 0.0f;
    lowestPackTemp = 50.0f;
    highestPackTemp = -10.0f;

    // First, stop balancing on all existing modules
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting())
        {
            modules[x].stopBalance();
        }
    }

    delay(1000);

    // Then, attempt to read all modules' values
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting())
        {
            Logger::debug("");
            Logger::debug("Module %i exists. Reading voltage and temperature values", x);

            // Try reading the module values, retrying if necessary
            if (modules[x].readModuleValues()) {
                // Only process the module if the data was successfully read
                Logger::debug("Module voltage: %f", modules[x].getModuleVoltage());
                Logger::debug("Lowest Cell V: %f     Highest Cell V: %f", modules[x].getLowCellV(), modules[x].getHighCellV());
                Logger::debug("Temp1: %f       Temp2: %f", modules[x].getTemperature(0), modules[x].getTemperature(1));

                packVolt += modules[x].getModuleVoltage();
                
                // Update the lowest and highest values
                if (modules[x].getLowCellV() < lowestCellVolt) lowestCellVolt = modules[x].getLowCellV();
                if (modules[x].getHighCellV() > highestCellVolt) highestCellVolt = modules[x].getHighCellV();
                if (modules[x].getLowTemp() < lowestPackTemp) lowestPackTemp = modules[x].getLowTemp();
                if (modules[x].getHighTemp() > highestPackTemp) highestPackTemp = modules[x].getHighTemp();            
            }
            else {
                // Log failure to read module
                Logger::error("Failed to read module %i. Skipping module...", x);
            }
        }
    }

    // Update the overall pack voltage bounds
    if (packVolt > highestPackVolt) highestPackVolt = packVolt;
    if (packVolt < lowestPackVolt) lowestPackVolt = packVolt;

    // Optionally handle fault state, uncomment if needed
    /*
    if (digitalRead(13) == LOW) {
        if (!isFaulted) Logger::error("One or more BMS modules have entered the fault state!");
        isFaulted = true;
    }
    else
    {
        if (isFaulted) Logger::info("All modules have exited a faulted state");
        isFaulted = false;
    }
    */
}


float BMSModuleManager::getPackVoltage()
{
    return packVolt;
}

float BMSModuleManager::getAvgTemperature()
{
    float avg = 0.0f;    
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting()) avg += modules[x].getAvgTemp();
    }
    avg = avg / (float)numFoundModules;

    return avg;
}

float BMSModuleManager::getAvgCellVolt()
{
    float avg = 0.0f;    
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting()) avg += modules[x].getAverageV();
    }
    avg = avg / (float)numFoundModules;

    return avg;
}

void BMSModuleManager::printPackSummary()
{
    uint8_t faults;
    uint8_t alerts;
    uint8_t COV;
    uint8_t CUV;

    Logger::console("");
    Logger::console("");
    Logger::console("");
    Logger::console("                                     Pack Status:");
    if (isFaulted) Logger::console("                                       FAULTED!");
    else Logger::console("                                   All systems go!");
    Logger::console("Modules: %i    System Voltage: %fV   Avg Cell Voltage: %fV     Avg Temp: %fC ", numFoundModules, 
                    getPackVoltage(),getAvgCellVolt(), getAvgTemperature());
    Logger::console("");
    for (int y = 1; y < 63; y++)
    {
        if (modules[y].isExisting())
        {
            faults = modules[y].getFaults();
            alerts = modules[y].getAlerts();
            COV = modules[y].getCOVCells();
            CUV = modules[y].getCUVCells();

            Logger::console("                               Module #%i", y);

            Logger::console("  Voltage: %fV   (%fV-%fV)     Temperatures: (%fC-%fC)", modules[y].getModuleVoltage(), 
                            modules[y].getLowCellV(), modules[y].getHighCellV(), modules[y].getLowTemp(), modules[y].getHighTemp());

            Serial.print("  Currently balancing cells: ");
            for (int i = 0; i < 6; i++)
            {                
                if (modules[y].getBalancingState(i) == 1) 
                {                    
                    Serial.print(i);
                    Serial.print(" ");
                }
            }
            Serial.println();

            if (faults > 0)
            {
                Logger::console("  MODULE IS FAULTED:");
                if (faults & 1)
                {
                    Serial.print("    Overvoltage Cell Numbers (1-6): ");
                    for (int i = 0; i < 6; i++)
                    {
                        if (COV & (1 << i)) 
                        {
                            Serial.print(i+1);
                            Serial.print(" ");
                        }
                    }
                    Serial.println();
                }
                if (faults & 2)
                {
                    Serial.print("    Undervoltage Cell Numbers (1-6): ");
                    for (int i = 0; i < 6; i++)
                    {
                        if (CUV & (1 << i)) 
                        {
                            Serial.print(i+1);
                            Serial.print(" ");
                        }
                    }
                    Serial.println();
                }
                if (faults & 4)
                {
                    Logger::console("    CRC error in received packet");
                }
                if (faults & 8)
                {
                    Logger::console("    Power on reset has occurred");
                }
                if (faults & 0x10)
                {
                    Logger::console("    Test fault active");
                }
                if (faults & 0x20)
                {
                    Logger::console("    Internal registers inconsistent");
                }
            } 
            if (alerts > 0)
            {
                Logger::console("  MODULE HAS ALERTS:");
                if (alerts & 1)
                {
                    Logger::console("    Over temperature on TS1");
                }
                if (alerts & 2)
                {
                    Logger::console("    Over temperature on TS2");
                }
                if (alerts & 4)
                {
                    Logger::console("    Sleep mode active");
                }
                if (alerts & 8)
                {
                    Logger::console("    Thermal shutdown active");
                }
                if (alerts & 0x10)
                {
                    Logger::console("    Test Alert");
                }
                if (alerts & 0x20)
                {
                    Logger::console("    OTP EPROM Uncorrectable Error");
                }
                if (alerts & 0x40)
                {
                    Logger::console("    GROUP3 Regs Invalid");
                }
                if (alerts & 0x80)
                {
                    Logger::console("    Address not registered");
                }
            }
            if (faults > 0 || alerts > 0) Serial.println();
        }
    }
}
/*
void BMSModuleManager::printPackDetails()
{
    uint8_t faults;
    uint8_t alerts;
    uint8_t COV;
    uint8_t CUV;
    int cellNum = 0;
    Serial.println("");

    //Logger::console("");
    //Logger::console("");
    //Logger::console("");
    //Logger::console("                                         Pack Status:");
    //if (isFaulted) Logger::console("                                           FAULTED!");
    //else Logger::console("                                      All systems go!");
    //Logger::console("Modules: %i    Voltage: %fV   Avg Cell Voltage: %fV     Avg Temp: %fC ", numFoundModules, 
    //                getPackVoltage(),getAvgCellVolt(), getAvgTemperature());
    //Logger::console("");
    for (int y = 1; y < 63; y++)
    {
        if (modules[y].isExisting())
        {
            faults = modules[y].getFaults();
            alerts = modules[y].getAlerts();
            COV = modules[y].getCOVCells();
            CUV = modules[y].getCUVCells();

            Serial.print("Module #");
            Serial.print(y);
            if (y < 10) Serial.print(" ");
            //Serial.print("  ");
            //Serial.print(modules[y].getModuleVoltage());
            //Serial.print("V");
            for (int i = 0; i < 6; i++)
            {
                if (cellNum < 10) Serial.print(" ");
                Serial.print("  Cell");
                if (cellNum < 10){Serial.print("0");}
                Serial.print(cellNum++ + 1);
                Serial.print(": ");
                Serial.print(modules[y].getCellVoltage(i), 3);
                Serial.print("V");
                if (modules[y].getBalancingState(i) == 1) Serial.print("*");
                else Serial.print(" ");
            }
            Serial.print("  Neg Term Temp: ");
            Serial.print(modules[y].getTemperature(0));
            Serial.print("C  Pos Term Temp: ");
            Serial.print(modules[y].getTemperature(1)); 
            Serial.print("C");
            if(isFaulted) Serial.println(" FAULTED!");
            else Serial.println("");
        }
    }
}
*/
void BMSModuleManager::printPackDetails()
{
    uint8_t faults;
    uint8_t alerts;
    uint8_t COV;
    uint8_t CUV;
    int cellNum = 0;
    Serial.println("");

    //Logger::console("");
    //Logger::console("");
    //Logger::console("");
    //Logger::console("                                         Pack Status:");
    //if (isFaulted) Logger::console("                                           FAULTED!");
    //else Logger::console("                                      All systems go!");
    //Logger::console("Modules: %i    Voltage: %fV   Avg Cell Voltage: %fV     Avg Temp: %fC ", numFoundModules, 
    //                getPackVoltage(),getAvgCellVolt(), getAvgTemperature());
    //Logger::console("");
    for (int y = 1; y < 63; y++)
    {
        if (modules[y].isExisting())
        {
            faults = modules[y].getFaults();
            alerts = modules[y].getAlerts();
            COV = modules[y].getCOVCells();
            CUV = modules[y].getCUVCells();

            Serial.printf("Module #%02d  ", y);

            for (int i = 0; i < 6; i++)
            {
                Serial.printf("  Cell%03d: %.3fV", cellNum + 1, modules[y].getCellVoltage(i));
                if (modules[y].getBalancingState(i) == 1)
                    Serial.print("*");
                else
                    Serial.print(" ");
                cellNum++;
            }

            Serial.printf("  Neg Term Temp: %.2fC  Pos Term Temp: %.2fC", 
                          modules[y].getTemperature(0), modules[y].getTemperature(1));
            Serial.println("");
        }
    }
}
void BMSModuleManager::printJsonData()
{
    for (int y = 1; y < 63; y++) {
        if (modules[y].isExisting()) {
            String msg = String(y);
            msg += ",";
            for (int i = 0; i < 6; i++) {
                msg += String(modules[y].getCellVoltage(i), 3);
                if (modules[y].getBalancingState(i) == 1) msg += "*";
                // Serial.print(modules[y].getBalancingState(i));
                msg += ",";
            }
            msg += modules[y].getTemperature(0);
            msg += ",";
            msg += modules[y].getTemperature(1);
            
            Serial.println(msg);
        }
    }
}
String BMSModuleManager::buildJsonData()
{
    String jsonResponse = "{\"packs\":[";
    bool firstModule = true;

    for (int y = 1; y < 63; y++) {
        if (modules[y].isExisting()) {
            if (!firstModule) {
                jsonResponse += ",";
            }
            firstModule = false;
            jsonResponse += "{";
            jsonResponse += "\"+\":\"" + String(modules[y].getTemperature(0), 2) + "\",";
            jsonResponse += "\"-\":\"" + String(modules[y].getTemperature(1), 2) + "\",";

            for (int i = 0; i < 6; i++) { // Loop through each cell in the module
            jsonResponse += "\"c" + String(i + 1) + "\":\"";
            jsonResponse += String(modules[y].getCellVoltage(i), 3); // Append cell voltage
            
            if (modules[y].getBalancingState(i) == 1) { // Check if cell is balancing
                jsonResponse += "*"; // Append asterisk if balancing
            }
            
            jsonResponse += "\""; // Close the value
            
            if (i < 5) jsonResponse += ","; // Add a comma except for the last cell
        }
            jsonResponse += ",\"module\":\"" + String(y) + "\"";
            jsonResponse += "}";
        }
    }
    jsonResponse += "]}";

    return jsonResponse;
}

void BMSModuleManager::handleBatteryStats(AsyncWebServerRequest* request, const String& bmsJson) {
    request->send(200, "application/json", bmsJson);
}

void BMSModuleManager::broadcastBatteryStats(AsyncWebSocket* ws, const String& bmsJson){
    ws->textAll(bmsJson);
    //Serial.println("Broadcasted WS JSON");
}

void BMSModuleManager::sendBatteryStats(String systemName, String ftpServer, String ftpUser, String ftpPassword, const String& bmsJson) {
    String systemNameLower = systemName;
    systemNameLower.toLowerCase();

    // Convert to C-style strings
    char ftpServerChar[64];
    char ftpUserChar[64];
    char ftpPasswordChar[64];

    ftpServer.toCharArray(ftpServerChar, sizeof(ftpServerChar));
    ftpUser.toCharArray(ftpUserChar, sizeof(ftpUserChar));
    ftpPassword.toCharArray(ftpPasswordChar, sizeof(ftpPasswordChar));

    // File names
    String tempFilename = systemNameLower + "_batterystats.json.tmp";
    String finalFilename = systemNameLower + "_batterystats.json";
    char tempChar[64];
    char finalChar[64];
    tempFilename.toCharArray(tempChar, sizeof(tempChar));
    finalFilename.toCharArray(finalChar, sizeof(finalChar));

    // This is the path on the ftp server where this system will drop off the  _batterystats.json file. Originally /tmp.
    String path = ".";

    ESP32_FTPClient* ftp = nullptr;  // Declare ftp pointer here so catch can see it

    try {
        ftp = new ESP32_FTPClient(ftpServerChar, ftpUserChar, ftpPasswordChar, 5000);

        ftp->OpenConnection();

        if (!ftp->isConnected()) {
            Serial.println("FTP error: could not connect to server");
            delete ftp;
            ftp = nullptr;
            return;
        }


        ftp->ChangeWorkDir(path.c_str());
        ftp->InitFile("Type I");

        ftp->NewFile(tempChar);
        ftp->WriteData((uint8_t*)bmsJson.c_str(), bmsJson.length());
        ftp->CloseFile();

        ftp->RenameFile(tempChar, finalChar);
//        Serial.println("FTP upload success: " + finalFilename);

        ftp->CloseConnection();
        delete ftp;
        ftp = nullptr;

    } catch (...) {
        Serial.println("Unknown FTP error occurred");
        if (ftp) {
            ftp->CloseConnection();
            delete ftp;
            ftp = nullptr;
        }
    }
}

void BMSModuleManager::publishIndividualData(PubSubClient& client, const char* baseTopic, String systemName) {
    for (int y = 1; y < 63; y++) {
        if (!modules[y].isExisting()) {
            continue;
        }
        // Publish cell voltages
        for (int i = 0; i < 6; i++) {
            String cellID = "p" + String(y) + "c" + String(i + 1);
            String cellValue = String(modules[y].getCellVoltage(i), 3);
            if(modules[y].getCellVoltage(i) > 2.5 && modules[y].getCellVoltage(i) < 4.29) {
              publishSensorData(client, baseTopic, systemName, cellID, "voltage", "V", 3, cellValue);
            }
            
        }
        // Publish temperatures
        const String terminalIDs[] = {"_neg", "_pos"};
        for (int t = 0; t < 2; t++) {
            String cellID = "p" + String(y) + terminalIDs[t];
            String cellValue = String(modules[y].getTemperature(t), 2);
            if(modules[y].getTemperature(t) > -10 && modules[y].getTemperature(t) < 50) {
              publishSensorData(client, baseTopic, systemName, cellID, "temperature", "°C", 2, cellValue);
            }
            
        }
    }

    // Publish system-wide max/min voltages and temperatures
    const String cellMetrics[] = {"max_value", "min_value"};
    const float cellValues[] = {highestCellVolt, lowestCellVolt};
    for (int i = 0; i < 2; i++) {
        if(cellValues[i] > 2.5 && cellValues[i] < 4.29){
          publishSensorData(client, baseTopic, systemName, cellMetrics[i], "voltage", "V", 3, String(cellValues[i], 3));
        }
        
    }

    const String tempMetrics[] = {"max_temp", "min_temp"};
    const float tempValues[] = {highestPackTemp, lowestPackTemp};
    for (int i = 0; i < 2; i++) {
      if(tempValues[i] > -10 && tempValues[i] < 50) {
        publishSensorData(client, baseTopic, systemName, tempMetrics[i], "temperature", "°C", 2, String(tempValues[i]));
      }
        
    }
    //Serial.println("Published data to Home Assistant.");

    // Debug summary
    //Serial.println("");
    //Serial.println("High temp: " + String(highestPackTemp));
    //Serial.println("Low temp: " + String(lowestPackTemp));
    //Serial.println("High volt: " + String(highestCellVolt));
    //Serial.println("Low volt: " + String(lowestCellVolt));
}

void BMSModuleManager::publishSensorData(PubSubClient& client, const char* baseTopic, String systemName, const String& cellID, const String& devClass, const String& unit, const int precision, const String& value) {
    String cellIDUpper = cellID;
    String systemNameLower = systemName;
    cellIDUpper.toUpperCase();
    systemNameLower.toLowerCase();

    String msg = "{\"name\":\"" + cellIDUpper + "\",\"dev_cla\":\"" + devClass + "\",\"unit_of_meas\":\"" + unit + "\",\"suggested_display_precision\":\"" + precision + "\",\"stat_t\":\"bms/sensor/" + systemNameLower + "_" +
             cellID + "/state\",\"uniq_id\":\"" + systemNameLower + "_" + cellID + "\",\"dev\":{\"ids\":[\"" + systemNameLower + "_bms\"],\"name\":\"" + systemName + "\", \"mf\":\"Bobby Martin\"}}";

    String config = baseTopic + systemNameLower + "_" + cellID + "/config";
    String state = "bms/sensor/" + systemNameLower + "_" + cellID + "/state";

    // Publish data
    client.publish(config.c_str(), msg.c_str(), true);
    client.publish(state.c_str(), value.c_str());

    // Debug logs
    //Serial.println("");
    //Serial.println("Published Sensor: " + cellID);
    //Serial.println(config + " | " + msg);
    //Serial.println(state + " | " + value);
}


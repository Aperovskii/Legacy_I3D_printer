#include "CommandManager.h"
#include <Arduino.h>
#include "Dosage.h"

//const int CommandManager::commandCount = 3;


const int CommandManager::commandCount = 21;

const CommandManager::CommandEntry CommandManager::commandTable[] = {

  // add your commands here. If you use external variables and functions, add them to .h file
  { "|", &CommandManager::degripPump },
  { "*", &CommandManager::startList },
  { "$", &CommandManager::connectStatus },
  { "?", &CommandManager::infoStatus },
  { "w", &CommandManager::FWD },
  { "s", &CommandManager::BWD },
  { "a", &CommandManager::LFT },
  { "d", &CommandManager::RGT },
  { "q", &CommandManager::UP },
  { "e", &CommandManager::DN },
  { ">", &CommandManager::toggleMix },
  { "b", &CommandManager::setZero },

  { "(", &CommandManager::accPlus },
  { ")", &CommandManager::accMinus },
  { "{", &CommandManager::pumpPlus },
  { "}", &CommandManager::pumpMinus },
  { ":", &CommandManager::mixPlus },
  { ";", &CommandManager::mixMinus },
  { "+", &CommandManager::frPlus },
  { "-", &CommandManager::frMinus },
  { "&", &CommandManager::empty },
  { "!", &CommandManager::evacuateTable }
};


CommandManager::CommandManager() {
}

void CommandManager::handleCommand(String command) {
  command.trim();
  Serial.println(logM("Command received: '"+command));
  //Serial.println(logM("', length: "+String(command.length())));
  //command.toUpperCase();
  for (int i = 0; i < commandCount; ++i) {
    Serial.println(logM(commandTable[i].commandName));
    if (command.equals(commandTable[i].commandName)) {
      CommandFunction func = commandTable[i].function;
      (this->*func)();
      return;
    }
  }
  Serial.println(logM("Unknown command: " + command));
}

// Pump turns for a short period of time
void CommandManager::degripPump() {
  pumpActive();
}

// Builds a list and receives next special command
void CommandManager::startList() {
  //Serial.println(logM("Command list started. Send *** to end it"));
  queueBuild();
  while (true) {
    //Serial.println(logM("A"));
    if (Serial.available() > 0) {
      String playCommand = Serial.readStringUntil('\n');
      playCommand.trim();

      if (playCommand == "play") {
        //Serial.println(logM("Processing list..."));
        queueProcess();
        clearQueue();  // Обработка очереди
        break;         // Выход из цикла
      } else if (playCommand == "cancel") {
        Serial.println(logM("Queue cleared."));
        clearQueue();  // Очистка очереди
        break;         // Выход из цикла
      } else if (playCommand == "read") {
        //Serial.println("Reading list of commands");
        queueRead();
        //break;
      } else {
        Serial.println(logM("Unknown command. Send 'play' to process or 'cancel' to clear the queue."));
      }
    }
  }
  if (moveList) {
    Serial.println("cccc");
  }

  moveList = false;
}

// Exchange data with the app to get parameters and tell state
void CommandManager::connectStatus(){
  connectUpdate();
}

// Send data report to the app
void CommandManager::infoStatus(){
  Serial.println(logM("Info received"));
  info();
}

// cinematics

void CommandManager::FWD(){
  stepUp();
}

void CommandManager::BWD(){
  stepDown();
}

void CommandManager::LFT(){
  stepLeft();
}

void CommandManager::RGT(){
  stepRight();
}

void CommandManager::UP(){
  stepElevate();
}

void CommandManager::DN(){
  stepLower();
}

// Change mixer state

void CommandManager::toggleMix(){
   if(flagMix){
          // switch on/off the motor
          stopMotor();
        } else {
          startMotor();
        }
}

// for now sets current point as zero

void CommandManager::setZero(){
  flagZero = true;

  // change state in app
  Serial.println("zzz");
}

// Dosage ajustments

void CommandManager::accPlus(){
  EcoPen.modifyValue(1);
  // we dont put in directly accelerator value, as it depends on the flowrate
  // this function assigns it correctly
  recalcEco();
}

void CommandManager::accMinus(){
  EcoPen.modifyValue(0);
  // we dont put in directly accelerator value, as it depends on the flowrate
  // this function assigns it correctly
  recalcEco();
}

void CommandManager::pumpPlus(){
  PumpPro.modifyValue(1);
  AD5593RR.write_DAC(PumpPro.getId(), 5 * (PumpPro.getValue()) / 100);
  // we dont put in directly accelerator value, as it depends on the flowrate
  // this function assigns it correctly
  recalcEco();
}

void CommandManager::pumpMinus(){
  PumpPro.modifyValue(0);
  AD5593RR.write_DAC(PumpPro.getId(), 5 * (PumpPro.getValue()) / 100);
  // we dont put in directly accelerator value, as it depends on the flowrate
  // this function assigns it correctly
  recalcEco();
}

void CommandManager::mixPlus(){
  Mixer.modifyValue(1);
  AD5593RR.write_DAC(Mixer.getId(), 5 * (Mixer.getValue()) / 100);
}

void CommandManager::mixMinus(){
  Mixer.modifyValue(0);
  AD5593RR.write_DAC(Mixer.getId(), 5 * (Mixer.getValue()) / 100);
}

// table feedrate
void CommandManager::frPlus(){
  feedrate += sps * mtos;
}

void CommandManager::frMinus(){
  feedrate -= sps * mtos;
}

// evacuate table on demand - app

void CommandManager::empty(){
 stopEmpty();
} 

void CommandManager::evacuateTable(){
  //evacuation procedure
  evacuate();
} 



#ifndef CommandManager_h
#define CommandManager_h

#include "Arduino.h"
#include "AD5593R.h"
#include "Dosage.h"

// Объявление глобальной переменной a как extern
extern int a;
extern String logM(String line);
extern void pumpActive();
extern void queueBuild();
extern void queueProcess();
extern void clearQueue();
extern void queueRead();
extern float getPress();
extern void stepUp();
extern void stepDown();
extern void stepLeft();
extern void stepRight();
extern void stepElevate();
extern void stepLower();
extern void stopMotor();
extern void startMotor();
extern void stopEmpty();
extern void connectUpdate();
extern void info();
extern void recalcEco();
extern void evacuate();

extern bool flagZero;
extern bool flagMix;
extern int feedrate;
extern long int mtos;
extern int sps;

extern AD5593R AD5593RR;

extern Dosage EcoPen;
extern Dosage PumpPro;
extern Dosage Mixer;
extern bool moveList;
extern bool flagZero;

class CommandManager{
private:
  typedef void (CommandManager::*CommandFunction)();
    struct CommandEntry {
        String commandName;
        CommandFunction function;
    };

    void startCommand();
    void degripPump();

    void startList();
    void connectStatus();
    void infoStatus();
    void FWD();
    void BWD();
    void LFT();
    void RGT();
    void UP();
    void DN();
    void toggleMix();
    void setZero();
    void accPlus();
    void accMinus();
    void pumpPlus();
    void pumpMinus();
    void mixPlus();
    void mixMinus();
    void frPlus();
    void frMinus();
    void empty();
    void evacuateTable();


    static const int commandCount;
    static const CommandEntry commandTable[];
public: 
  CommandManager(); // Конструктор
  void handleCommand(String command); // Метод для обработки команд
};

#endif
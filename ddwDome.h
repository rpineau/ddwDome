//
//  ddwDome.h
//
//  DDW rotation drive unit for TI Dome X2 plugin
//
//  Created by Rodolphe Pineau on 2017-04-13.

#ifndef __DDW_DOME__
#define __DDW_DOME__
#include <math.h>
#include <string.h>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"

#define SERIAL_BUFFER_SIZE 4096
#define MAX_TIMEOUT 5000
#define ND_LOG_BUFFER_SIZE 256

// field indexes in GINF
#define gVersion     0
#define gDticks      1
#define gHomeAz      2
#define gCoast       3
#define gADAZ        4
#define gSlave       5
#define gShutter     6
#define gDSR         7
#define gHome        8
#define gHTICK_CCLK  9
#define gHTICK_CLK  10
#define gUPINS      11
#define gWEAAGE     12
#define gWINDDIR    13
#define gWINDSPD    14
#define gTEMP       15
#define gHUMID      16
#define gWETNESS    17
#define gSNOW       18
#define gWINDPEAK   19
#define gSCOPEAZ    20
#define gINTDZ      21
#define gINTOFF     22
#define gCR1        23
#define gCR2        24

// error codes
// Error code
enum ddwDomeErrors {DDW_OK=0, NOT_CONNECTED, DDW_CANT_CONNECT, DDW_BAD_CMD_RESPONSE, COMMAND_FAILED};
//  0=indeterminate, 1=closed, 2=open
enum ddwDomeShutterState {UNKNOWN=0, CLOSED, OPEN};

class CddwDome
{
public:
    CddwDome();
    ~CddwDome();

    int        Connect(const char *szPort);
    void        Disconnect(void);
    bool        IsConnected(void) { return bIsConnected; }

    void        SetSerxPointer(SerXInterface *p) { pSerx = p; }
    void        setLogger(LoggerInterface *pLogger) { mLogger = pLogger; };

    // Dome commands
    int syncDome(double dAz, double dEl);
    int parkDome(void);
    int unparkDome(void);
    int gotoAzimuth(double newAz);
    int openShutter();
    int closeShutter();
    int getFirmwareVersion(char *version, int strMaxLen);
    int getModel(char *model, int strMaxLen);
    int goHome();
    int calibrate();

    // command complete functions
    int isGoToComplete(bool &complete);
    int isOpenComplete(bool &complete);
    int isCloseComplete(bool &complete);
    int isParkComplete(bool &complete);
    int isUnparkComplete(bool &complete);
    int isFindHomeComplete(bool &complete);
    int isCalibratingComplete(bool &complete);

    int abortCurrentCommand();

    // getter/setter
    int getNbTicksPerRev();
    int getBatteryLevel();

    double getHomeAz();
    int setHomeAz(double dAz);

    double getCurrentAz();
    double getCurrentEl();

    int getCurrentShutterState();

    void setDebugLog(bool enable);

protected:
    
    int             readResponse(char *respBuffer, int bufferLen);
    int             getDomeAz(double &domeAz);
    int             getDomeEl(double &domeEl);
    int             getDomeHomeAz(double &Az);
    int             getShutterState(int &state);
    int             getDomeStepPerRev(int &stepPerRev);

    bool            isDomeMoving();
    bool            isDomeAtHome();
    
    int             domeCommand(const char *cmd, char *result, int resultMaxLen);

    int             parseGINF(char *ginf);

    LoggerInterface *mLogger;
    bool            bDebugLog;
    
    bool            bIsConnected;
    bool            mHomed;
    bool            mParked;
    bool            bCalibrating;
    
    int             mNbStepPerRev;
    double          mShutterBatteryVolts;
    double          mShutterBatteryPercent;
    double          mHomeAz;
    
    double          mCurrentAzPosition;
    double          mCurrentElPosition;

    double          mGotoAz;
    
    SerXInterface   *pSerx;
    
    char            firmwareVersion[SERIAL_BUFFER_SIZE];
    int             mShutterState;
    bool            mHasShutter;
    bool            mShutterOpened;

    char            mLogBuffer[ND_LOG_BUFFER_SIZE];
    std::vector<std::string>    gInf;


};

#endif

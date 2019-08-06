//
//  ddwDome.h
//
//  DDW rotation drive unit for TI Dome X2 plugin
//
//  Created by Rodolphe Pineau on 2017-04-13.

#ifndef __DDW_DOME__
#define __DDW_DOME__

#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <memory.h>

#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif

#ifdef SB_WIN_BUILD
#include <time.h>
#endif

#include <math.h>
#include <string.h>

#include <string>
#include <vector>
#include <sstream>
#include <iostream>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"

#include "StopWatch.h"

// #define DDW_DEBUG 2

#define SERIAL_BUFFER_SIZE 4096
#define MAX_TIMEOUT 1500
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
enum ddwDomeErrors {DDW_OK=0, NOT_CONNECTED, DDW_CANT_CONNECT, DDW_BAD_CMD_RESPONSE, DDW_TIMEOUT, COMMAND_FAILED};
//  0=indeterminate, 1=closed, 2=open
enum ddwDomeShutterState {UNKNOWN=0, CLOSED, OPEN};

enum ddwDomeHomeStatus {AT_HOME = 0, NOT_AT_HOME};

class CddwDome
{
public:
    CddwDome();
    ~CddwDome();

    int        Connect(const char *szPort,  bool bHardwareFlowControl = true);
    void        Disconnect(void);
    bool        IsConnected(void) { return m_bIsConnected; }

    void        SetSerxPointer(SerXInterface *p) { m_pSerx = p; }
    void        setSleeper(SleeperInterface *pSleeper) { m_pSleeper = pSleeper; };

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

    double getCurrentAz();
    double getCurrentEl();

    int getCurrentShutterState();

    void setDebugLog(bool enable);

protected:
    
    int             domeCommand(const char *szCmd, char *szResult, unsigned int nResultMaxLen, unsigned int nTimeout = MAX_TIMEOUT);
    int             readResponse(char *szRrespBuffer, unsigned int nBufferLen, unsigned int nTimeout = MAX_TIMEOUT);
    int             readAllResponses(char *respBuffer, unsigned int bufferLen);   // read all the response, only keep the last one.
    int             getInfRecord();

    int             getDomeAz(double &domeAz);
    int             getDomeEl(double &domeEl);
    int             getDomeHomeAz(double &Az);
    int             getShutterState(int &state);
    int             getDomeStepPerRev(int &stepPerRev);
    int             getCoast(double &dDeg);

    bool            isDomeMoving();
    bool            isShutterMoving();
    bool            isDomeAtHome();
    

    int             parseGINF(char *ginf);
    int             parseFields(const char *pszIn, std::vector<std::string> &svFields, const char &cSeparator);
    
    
    LoggerInterface *mLogger;    
    bool            m_bIsConnected;
    bool            m_bParked;
    bool            m_bDomeIsMoving;
    bool            m_bShutterIsMoving;
    int             m_nNbStepPerRev;
    int             m_nNbStepCoast;

    double          m_dShutterBatteryVolts;
    double          m_dShutterBatteryPercent;
    double          m_dHomeAz;
    
    double          m_dCurrentAzPosition;
    double          m_dCurrentElPosition;

    double          m_dGotoAz;

    SerXInterface   *m_pSerx;
    SleeperInterface    *m_pSleeper;

    char            m_szFirmwareVersion[SERIAL_BUFFER_SIZE];
    int             m_nShutterState;
    bool            m_bHasShutter;
    bool            m_bShutterOpened;

    std::vector<std::string>    m_svGinf;

    CStopWatch      timer;
    CStopWatch      dataReceivedTimer;
    float           m_dInfRefreshInterval;;

#ifdef DDW_DEBUG
    std::string m_sLogfilePath;
    // timestamp for logs
    char *timestamp;
    time_t ltime;
    FILE *Logfile;      // LogFile
#endif


};

#endif

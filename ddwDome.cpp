//
//  ddwDome.cpp
//  DDW rotation drive unit for TI Dome X2 plugin
//
//  Created by Rodolphe Pineau on 2017-04-13.


#include "ddwDome.h"
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <memory.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif

CddwDome::CddwDome()
{
    // set some sane values
    bDebugLog = true;
    
    pSerx = NULL;
    bIsConnected = false;

    mNbStepPerRev = 0;
    mShutterBatteryVolts = 0.0;
    
    mHomeAz = 180;

    mCurrentAzPosition = 0.0;
    mCurrentElPosition = 0.0;

    bCalibrating = false;

    mHasShutter = false;
    mShutterOpened = false;
    
    mParked = true;
    mHomed = false;
    memset(firmwareVersion,0,SERIAL_BUFFER_SIZE);
    memset(mLogBuffer,0,ND_LOG_BUFFER_SIZE);
}

CddwDome::~CddwDome()
{

}

int CddwDome::Connect(const char *szPort)
{
    int err;
    int state;

    // 9600 8N1
    if(pSerx->open(szPort, 115200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        bIsConnected = true;
    else
        bIsConnected = false;

    if(!bIsConnected)
        return ERR_COMMNOLINK;

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CddwDome::Connect] Connected.\n");
        mLogger->out(mLogBuffer);

        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CddwDome::Connect] Getting Firmware.\n");
        mLogger->out(mLogBuffer);
    }
    // if this fails we're not properly connected.
    err = getFirmwareVersion(firmwareVersion, SERIAL_BUFFER_SIZE);
    if(err) {
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CddwDome::Connect] Error Getting Firmware.\n");
            mLogger->out(mLogBuffer);
        }
        bIsConnected = false;
        pSerx->close();
        return FIRMWARE_NOT_SUPPORTED;
    }

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CddwDome::Connect] Got Firmware.\n");
        mLogger->out(mLogBuffer);
    }
    // assume the dome was parked at home
    getDomeHomeAz(mCurrentAzPosition);

    syncDome(mCurrentAzPosition,mCurrentElPosition);
    err = getShutterState(state);


    return SB_OK;
}


void CddwDome::Disconnect()
{
    if(bIsConnected) {
        pSerx->purgeTxRx();
        pSerx->close();
    }
    bIsConnected = false;
}


int CddwDome::readResponse(char *respBuffer, int bufferLen)
{
    int err = DDW_OK;
    unsigned long nBytesRead = 0;
    unsigned long totalBytesRead = 0;
    char *bufPtr;

    memset(respBuffer, 0, (size_t) bufferLen);
    bufPtr = respBuffer;

    do {
        err = pSerx->readFile(bufPtr, 1, nBytesRead, MAX_TIMEOUT);
        if(err) {
            if (bDebugLog) {
                snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CddwDome::readResponse] readFile error.\n");
                mLogger->out(mLogBuffer);
            }
            return err;
        }

        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CddwDome::readResponse] respBuffer = %s\n",respBuffer);
            mLogger->out(mLogBuffer);
        }
        
        if (nBytesRead !=1) {// timeout
            if (bDebugLog) {
                snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CddwDome::readResponse] readFile Timeout.\n");
                mLogger->out(mLogBuffer);
            }
            err = DDW_BAD_CMD_RESPONSE;
            break;
        }
        totalBytesRead += nBytesRead;
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CddwDome::readResponse] nBytesRead = %lu\n",nBytesRead);
            mLogger->out(mLogBuffer);
        }
    } while (*bufPtr++ != 0x0D && totalBytesRead < bufferLen );


    if(totalBytesRead)
        *(bufPtr-1) = 0; //remove the \r

    return err;
}


int CddwDome::domeCommand(const char *cmd, char *result, int resultMaxLen)
{
    int err = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];
    unsigned long  nBytesWrite;

    pSerx->purgeTxRx();
    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CddwDome::domeCommand] Sending %s\n",cmd);
        mLogger->out(mLogBuffer);
    }
    err = pSerx->writeFile((void *)cmd, strlen(cmd), nBytesWrite);
    pSerx->flushTx();
    if(err)
        return err;
    // read response
    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CddwDome::domeCommand] Getting response.\n");
        mLogger->out(mLogBuffer);
    }
    err = readResponse(resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    if(result)
        strncpy(result, &resp[1], resultMaxLen);

    return err;

}

int CddwDome::getDomeAz(double &domeAz)
{
    int err = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];
    double domeAzticks;
    double domeTicksPerRev;
    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return err;

    err = domeCommand("GINF", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;
    
    // parse INF packet to get current ADAZ
    parseGINF(resp);

    domeAzticks =  std::stof(m_svGinf[gADAZ]);
    domeTicksPerRev = std::stof(m_svGinf[gDticks]);

    domeAz = (359.0/domeTicksPerRev) * domeAzticks;
    mCurrentAzPosition = domeAz;

    return err;
}

int CddwDome::getDomeEl(double &domeEl)
{
    int err = DDW_OK;
    int shutterState;

    if(!bIsConnected)
        return NOT_CONNECTED;

    getShutterState(shutterState);

    if(!mShutterOpened || !mHasShutter)
    {
        domeEl = 0.0;
    }
    else {
        domeEl = 90.0;
    }

    mCurrentElPosition = domeEl;
    
    return err;
}


int CddwDome::getDomeHomeAz(double &Az)
{
    int err = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];
    double HomeAzticks;
    double domeTicksPerRev;

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return err;

    err = domeCommand("GINF", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    // parse INF packet to get home1 position
    parseGINF(resp);

    HomeAzticks = std::stof(m_svGinf[gHomeAz]);
    domeTicksPerRev = std::stof(m_svGinf[gDticks]);

    Az = (359.0/domeTicksPerRev) * HomeAzticks;
    mHomeAz = Az;

    return err;
}



int CddwDome::getShutterState(int &state)
{
    int err = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];
    int shutterState;

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return err;

    err = domeCommand("GINF", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    // parse INF packet to get Shutter position
    parseGINF(resp);

    shutterState = std::stoi(m_svGinf[gShutter]);

    switch(shutterState) {
        case OPEN:
            mShutterOpened = true;
            break;

        case CLOSED:
            mShutterOpened = false;
            break;

        case UNKNOWN:
            mShutterOpened = false;
            break;
        default:
            mShutterOpened = false;
            
    }

    // state = atoi(resp);

    return err;
}


int CddwDome::getDomeStepPerRev(int &stepPerRev)
{
    int err = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    err = domeCommand("GINF", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    // parse INF packet to get DTICKS
    parseGINF(resp);
    stepPerRev =  std::stoi(m_svGinf[gDticks]);

    mNbStepPerRev = stepPerRev;
    return err;
}


void CddwDome::setDebugLog(bool enable)
{
    bDebugLog = enable;
}

bool CddwDome::isDomeMoving()
{
    bool isMoving;
    int tmp;
    int err = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    err = domeCommand("MSTATE\r", resp, SERIAL_BUFFER_SIZE);
    if(err) {
        return false;   // Not really correct but will do for now.
    }

    isMoving = false;
    tmp = atoi(resp);
    if(tmp != 0 || tmp != 3)
        isMoving = true;

    return isMoving;
}

bool CddwDome::isDomeAtHome()
{
    bool athome;
    int tmp;
    int err = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];
    
    if(!bIsConnected)
        return NOT_CONNECTED;
    
    err = domeCommand("HOME ?\r", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return false;

    athome = false;
    tmp = atoi(resp);
    if(tmp)
        athome = true;
    
    return athome;
  
}

int CddwDome::syncDome(double dAz, double dEl)
{
    int err = DDW_OK;
    char buf[SERIAL_BUFFER_SIZE];
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    mCurrentAzPosition = dAz;
    snprintf(buf, SERIAL_BUFFER_SIZE, "ANGLE K %3.1f\r", dAz);
    err = domeCommand(buf, resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;
    if(strncmp(resp,"A",1) == 0) {
        err = DDW_OK;
    }
    else {
        err = DDW_BAD_CMD_RESPONSE;
    }
    return err;
}

int CddwDome::parkDome()
{
    int err = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return SB_OK;

    err = domeCommand("GHOM", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    if(strncmp(resp,"A",1) == 0) {
        err = DDW_OK;
    }
    else {
        err = DDW_BAD_CMD_RESPONSE;
    }
    return err;
}

int CddwDome::unparkDome()
{
    mParked = false;
    mCurrentAzPosition = mHomeAz;
    syncDome(mCurrentAzPosition,mCurrentElPosition);
    return 0;
}

int CddwDome::gotoAzimuth(double newAz)
{

    int err = DDW_OK;
    char buf[SERIAL_BUFFER_SIZE];
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    snprintf(buf, SERIAL_BUFFER_SIZE, "G%03f", newAz);
    err = domeCommand(buf, resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;
    if(strncmp(resp,"A",1) == 0) {
        err = DDW_OK;
    }
    else {
        err = DDW_BAD_CMD_RESPONSE;
    }

    mGotoAz = newAz;

    return err;
}

int CddwDome::openShutter()
{
    int err = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return SB_OK;

    err = domeCommand("GOPN", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    return err;
}

int CddwDome::closeShutter()
{
    int err = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return SB_OK;

    err = domeCommand("GCLS", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    return err;
}

int CddwDome::getFirmwareVersion(char *version, int strMaxLen)
{
    int err = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];
    double fVersion;

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return SB_OK;

    err = domeCommand("VER\r", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    fVersion = atof(resp);
    snprintf(version,strMaxLen, "%.2f",fVersion);
    return err;
}

int CddwDome::getModel(char *model, int strMaxLen)
{
    int err = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return SB_OK;

    err = domeCommand("PULSAR\r", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    strncpy(model, resp, strMaxLen);
    return err;
}

int CddwDome::goHome()
{
    int err = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return SB_OK;

    err = domeCommand("GHOM", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    if(strncmp(resp,"A",1) == 0) {
        err = DDW_OK;
    }
    else {
        err = DDW_BAD_CMD_RESPONSE;
    }
    return err;
}

int CddwDome::calibrate()
{
    int err = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return SB_OK;

    err = domeCommand("CALIBRATE\r", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    if(strncmp(resp,"A",1) == 0) {
        err = DDW_OK;
    }
    else {
        err = DDW_BAD_CMD_RESPONSE;
        return err;
    }

    bCalibrating = true;
    
    return err;
}

int CddwDome::isGoToComplete(bool &complete)
{
    int err = 0;
    double domeAz = 0;

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(isDomeMoving()) {
        complete = false;
        getDomeAz(domeAz);
        return err;
    }

    getDomeAz(domeAz);

    if (ceil(mGotoAz) == ceil(domeAz))
        complete = true;
    else {
        // we're not moving and we're not at the final destination !!!
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CddwDome::isGoToComplete] domeAz = %f, mGotoAz = %f\n", ceil(domeAz), ceil(mGotoAz));
            mLogger->out(mLogBuffer);
        }
        complete = false;
        err = ERR_CMDFAILED;
    }

    return err;
}

int CddwDome::isOpenComplete(bool &complete)
{
    int err=0;
    int state;

    if(!bIsConnected)
        return NOT_CONNECTED;

    err = getShutterState(state);
    if(err)
        return ERR_CMDFAILED;
    if(state == OPEN){
        mShutterOpened = true;
        complete = true;
        mCurrentElPosition = 90.0;
    }
    else {
        mShutterOpened = false;
        complete = false;
        mCurrentElPosition = 0.0;
    }

    return err;
}

int CddwDome::isCloseComplete(bool &complete)
{
    int err=0;
    int state;

    if(!bIsConnected)
        return NOT_CONNECTED;

    err = getShutterState(state);
    if(err)
        return ERR_CMDFAILED;
    if(state == CLOSED){
        mShutterOpened = false;
        complete = true;
        mCurrentElPosition = 0.0;
    }
    else {
        mShutterOpened = true;
        complete = false;
        mCurrentElPosition = 90.0;
    }

    return err;
}


int CddwDome::isParkComplete(bool &complete)
{
    int err = 0;
    double domeAz=0;

    if(!bIsConnected)
        return NOT_CONNECTED;

    getDomeAz(domeAz);

    if(isDomeMoving()) {
        complete = false;
        return err;
    }

    if (ceil(mHomeAz) == ceil(domeAz))
    {
        mParked = true;
        complete = true;
    }
    else {
        // we're not moving and we're not at the final destination !!!
        complete = false;
        mHomed = false;
        mParked = false;
        err = ERR_CMDFAILED;
    }

    return err;
}

int CddwDome::isUnparkComplete(bool &complete)
{
    int err=0;

    if(!bIsConnected)
        return NOT_CONNECTED;

    mParked = false;
    complete = true;

    return err;
}

int CddwDome::isFindHomeComplete(bool &complete)
{
    int err = 0;

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(isDomeMoving()) {
        mHomed = false;
        complete = false;
        return err;
    }

    if(isDomeAtHome()){
        mHomed = true;
        complete = true;
    }
    else {
        // we're not moving and we're not at the home position !!!
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CddwDome::isFindHomeComplete] Not moving and not at home !!!\n");
            mLogger->out(mLogBuffer);
        }
        complete = false;
        mHomed = false;
        mParked = false;
        err = ERR_CMDFAILED;
    }

    return err;
}


int CddwDome::isCalibratingComplete(bool &complete)
{
    int err = 0;
    double domeAz = 0;

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(isDomeMoving()) {
        getDomeAz(domeAz);
        mHomed = false;
        complete = false;
        return err;
    }

    
    err = getDomeAz(domeAz);

    if (ceil(mHomeAz) != ceil(domeAz)) {
        // We need to resync the current position to the home position.
        mCurrentAzPosition = mHomeAz;
        syncDome(mCurrentAzPosition,mCurrentElPosition);
        mHomed = true;
        complete = true;
    }

    err = getDomeStepPerRev(mNbStepPerRev);
    mHomed = true;
    complete = true;
    bCalibrating = false;
    return err;
}


int CddwDome::abortCurrentCommand()
{
    if(!bIsConnected)
        return NOT_CONNECTED;

    bCalibrating = false;

    return (domeCommand("STOP\r", NULL, SERIAL_BUFFER_SIZE));
}

#pragma mark - Getter / Setter

int CddwDome::getNbTicksPerRev()
{
    if(bIsConnected)
        getDomeStepPerRev(mNbStepPerRev);
    return mNbStepPerRev;
}


double CddwDome::getHomeAz()
{
    if(bIsConnected)
        getDomeHomeAz(mHomeAz);

    return mHomeAz;
}

int CddwDome::setHomeAz(double dAz)
{
    int err = DDW_OK;
    char buf[SERIAL_BUFFER_SIZE];
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    snprintf(buf, SERIAL_BUFFER_SIZE, "HOME %3.1f\r", dAz);
    err = domeCommand(buf, resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;
    if(strncmp(resp,"A",1) == 0) {
        err = DDW_OK;
    }
    else {
        err = DDW_BAD_CMD_RESPONSE;
    }
    mHomeAz = dAz;
    return err;
}



double CddwDome::getCurrentAz()
{
    if(bIsConnected)
        getDomeAz(mCurrentAzPosition);
    
    return mCurrentAzPosition;
}

double CddwDome::getCurrentEl()
{
    if(bIsConnected)
        getDomeEl(mCurrentElPosition);
    
    return mCurrentElPosition;
}

int CddwDome::getCurrentShutterState()
{
    if(bIsConnected)
        getShutterState(mShutterState);

    return mShutterState;
}


// V4,701,527,4,526,0,1,1,0,522,532,0,128,255,255,255,255,255,255,255,999,5,0


int CddwDome::parseGINF(char *ginf)
{
    int nErr = DDW_OK;
    int n;
    std::vector<std::string> vFieldsData;

    nErr = parseFields(ginf, vFieldsData, ',');
    // do we have all the fields ?
    if (vFieldsData[0] == "V1")
        n =9;
    else
        n = 23;

    if(vFieldsData.size() < n)
        return DDW_BAD_CMD_RESPONSE;

    m_svGinf = vFieldsData;
    return DDW_OK;
}

int CddwDome::parseFields(const char *pszIn, std::vector<std::string> &svFields, const char &cSeparator)
{
    int nErr = DDW_OK;
    std::string sSegment;
    std::stringstream ssTmp(pszIn);

    svFields.clear();
    // split the string into vector elements
    while(std::getline(ssTmp, sSegment, cSeparator))
    {
        svFields.push_back(sSegment);
    }

    if(svFields.size()==0) {
        nErr = ERR_BADFORMAT;
    }
    return nErr;
}


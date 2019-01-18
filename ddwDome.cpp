//
//  ddwDome.cpp
//  DDW rotation drive unit for TI Dome X2 plugin
//
//  Created by Rodolphe Pineau on 2017-04-13.


#include "ddwDome.h"


CddwDome::CddwDome()
{
    // set some sane values
    pSerx = NULL;
    m_bIsConnected = false;

    m_nNbStepPerRev = 0;
    m_dShutterBatteryVolts = 0.0;
    
    m_dHomeAz = 180;

    m_dCurrentAzPosition = 0.0;
    m_dCurrentElPosition = 0.0;

    m_bCalibrating = false;

    m_bHasShutter = false;
    m_bShutterOpened = false;
    
    m_bParked = true;
    m_bHomed = false;

    memset(m_szFirmwareVersion,0,SERIAL_BUFFER_SIZE);

    timer.Reset();
    m_dInfRefreshInterval = 2;

#ifdef DDW_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\DDWLog.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/DDWLog.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/DDWLog.txt";
#endif
    Logfile = fopen(m_sLogfilePath.c_str(), "w");
#endif

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CddwDome Constructor Called.\n", timestamp);
    fflush(Logfile);
#endif

}

CddwDome::~CddwDome()
{

}

int CddwDome::Connect(const char *szPort)
{
    int nErr;
    int nState;

    if(pSerx->open(szPort, 9600, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::Connect] Connected.\n", timestamp);
    fprintf(Logfile, "[%s] [CddwDome::Connect] Getting Firmware.\n", timestamp);
    fflush(Logfile);
#endif

    // if this fails we're not properly connected.
    nErr = getFirmwareVersion(m_szFirmwareVersion, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined DDW_DEBUG && DDW_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::Connect] Error Getting Firmware.\n", timestamp);
        fflush(Logfile);
#endif
        m_bIsConnected = false;
        pSerx->close();
        return FIRMWARE_NOT_SUPPORTED;
    }

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::Connect] Got Firmware : %s\n", timestamp, m_szFirmwareVersion);
    fflush(Logfile);
#endif

    // assume the dome was parked at home
    getDomeHomeAz(m_dCurrentAzPosition);

    syncDome(m_dCurrentAzPosition, m_dCurrentElPosition);
    nErr = getShutterState(nState);


    return SB_OK;
}


void CddwDome::Disconnect()
{
    if(m_bIsConnected) {
        pSerx->purgeTxRx();
        pSerx->close();
    }
    m_bIsConnected = false;
}


int CddwDome::domeCommand(const char *cmd, char *result, unsigned int resultMaxLen)
{
    int nErr = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];
    unsigned long  nBytesWrite;
    int nNbTimeout = 0;
    int nMaxNbTimeout = 3;

    do {
        pSerx->purgeTxRx();
    #if defined DDW_DEBUG && DDW_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::domeCommand] Sending %s\n", timestamp, cmd);
        fflush(Logfile);
    #endif

        nErr = pSerx->writeFile((void *)cmd, strlen(cmd), nBytesWrite);
        pSerx->flushTx();
        if(nErr)
            return nErr;
        // read response
    #if defined DDW_DEBUG && DDW_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::domeCommand] Getting response.\n", timestamp);
        fflush(Logfile);
    #endif
        nErr = readResponse(resp, SERIAL_BUFFER_SIZE);
        if (nErr == DDW_TIMEOUT) {
            if(nNbTimeout >= nMaxNbTimeout) // make sure we don't end up in an infinite loop
                return ERR_CMDFAILED;
            nNbTimeout++;
            m_pSleeper->sleep(1500);    // wait 1.5 second and resend command
        }
    } while (nErr == DDW_TIMEOUT);
    
    if(result)
        strncpy(result, resp, resultMaxLen);

    return nErr;

}

int CddwDome::readResponse(char *respBuffer, unsigned int bufferLen)
{
    int nErr = DDW_OK;
    unsigned long nBytesRead = 0;
    unsigned long totalBytesRead = 0;
    char *bufPtr;

    memset(respBuffer, 0, (size_t) bufferLen);
    bufPtr = respBuffer;

    do {
        nErr = pSerx->readFile(bufPtr, 1, nBytesRead, MAX_TIMEOUT);
        if(nErr) {
#if defined DDW_DEBUG && DDW_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CddwDome::readResponse] readFile error : %d\n", timestamp, nErr);
            fflush(Logfile);
#endif
            return nErr;
        }

#if defined DDW_DEBUG && DDW_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::readResponse] respBuffer = %s\n", timestamp, respBuffer);
        fflush(Logfile);
#endif

        if (nBytesRead !=1) {// timeout
#if defined DDW_DEBUG && DDW_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CddwDome::readResponse] readFile Timeout\n", timestamp);
            fflush(Logfile);
#endif
            nErr = DDW_TIMEOUT;
            return nErr;
            break;
        }
        totalBytesRead += nBytesRead;
#if defined DDW_DEBUG && DDW_DEBUG >= 3
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::readResponse] totalBytesRead = %lu\n", timestamp, totalBytesRead);
        fprintf(Logfile, "[%s] [CddwDome::readResponse] respBuffer = %s\n", timestamp, respBuffer);
        fflush(Logfile);
#endif
    } while (*bufPtr++ != 0x0D && totalBytesRead < bufferLen );


    if(totalBytesRead)
        *(bufPtr-1) = 0; //remove the \r

    return nErr;
}

// read all the response, only keep the last one.
int CddwDome::readAllResponse(char *respBuffer, unsigned int bufferLen)
{
    int nErr = DDW_OK;
    int nbByteWaiting = 0;

    do {
        pSerx->bytesWaitingRx(nbByteWaiting);
        if(nbByteWaiting)
            nErr = readResponse(respBuffer, bufferLen);
    } while(nbByteWaiting);

    return nErr;
}

int CddwDome::getDomeAz(double &domeAz)
{
    int nErr = DDW_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating || m_bIsHoming || m_bIsGoToing) {
        domeAz = m_dCurrentAzPosition;  // should be updated when checking if dome is moving
        return nErr;
    }

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getDomeAz]\n", timestamp);
    fflush(Logfile);
#endif


    nErr = getInfRecord();
    if(nErr)
        return nErr;

    m_nNbStepPerRev = std::stoi(m_svGinf[gDticks]);
    m_dCurrentAzPosition = (359.0/m_nNbStepPerRev) * std::stof(m_svGinf[gADAZ]);

    domeAz = m_dCurrentAzPosition;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getDomeAz] m_dCurrentAzPosition = %3.2f\n", timestamp, m_dCurrentAzPosition);
    fflush(Logfile);
#endif

    return nErr;
}

int CddwDome::getDomeEl(double &domeEl)
{
    int nErr = DDW_OK;
    int shutterState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    getShutterState(shutterState);

    if(!m_bShutterOpened || !m_bHasShutter)
    {
        domeEl = 0.0;
    }
    else {
        domeEl = 90.0;
    }

    m_dCurrentElPosition = domeEl;
    
    return nErr;
}


int CddwDome::getDomeHomeAz(double &Az)
{
    int nErr = DDW_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating || m_bIsHoming || m_bIsGoToing)
        return nErr;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getDomeHomeAz]\n", timestamp);
    fflush(Logfile);
#endif

    nErr = getInfRecord();
    if(nErr)
        return nErr;

    m_nNbStepPerRev = std::stoi(m_svGinf[gDticks]);

    Az = (359.0/m_nNbStepPerRev) * std::stof(m_svGinf[gHomeAz]);
    m_dHomeAz = Az;

    return nErr;
}


int CddwDome::getShutterState(int &state)
{
    int nErr = DDW_OK;
    int shutterState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getShutterState]\n", timestamp);
    fflush(Logfile);
#endif


    if(!m_bCalibrating && !m_bIsHoming && !m_bIsGoToing)  {
        nErr = getInfRecord();
        if(nErr)
            return nErr;
    }

    shutterState = std::stoi(m_svGinf[gShutter]);

    switch(shutterState) {
        case OPEN:
            m_bShutterOpened = true;
            break;

        case CLOSED:
            m_bShutterOpened = false;
            break;

        case UNKNOWN:
            m_bShutterOpened = false;
            break;
        default:
            m_bShutterOpened = false;
            
    }

    state = shutterState;

    return nErr;
}


int CddwDome::getDomeStepPerRev(int &stepPerRev)
{
    int nErr = DDW_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getDomeStepPerRev]\n", timestamp);
    fflush(Logfile);
#endif

    if(!m_bCalibrating && !m_bIsHoming && !m_bIsGoToing)  {
        nErr = getInfRecord();
        if(nErr)
            return nErr;
    }

    stepPerRev =  std::stoi(m_svGinf[gDticks]);

    m_nNbStepPerRev = stepPerRev;
    return nErr;
}


bool CddwDome::isDomeMoving()
{
    bool bIsMoving;
    int nErr = DDW_OK;
    int nConvErr = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];
    std::vector<std::string> vFieldsData;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    bIsMoving = false;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isDomeMoving]\n", timestamp);
    fflush(Logfile);
#endif


    nErr = readAllResponse(resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    switch(resp[0]) {
        case 'L':
        case 'R':
        case 'T':
            bIsMoving  = true;
            break;
        case 'P':
            bIsMoving  = true;
            nConvErr = parseFields(resp, vFieldsData, 'P');
            if(!nConvErr) {
                m_dCurrentAzPosition = (359.0/m_nNbStepPerRev) * std::stof(vFieldsData[0]);
            }
            break;

        case 'V':   // GINF
            parseGINF(resp);
            bIsMoving  = false;
            m_nNbStepPerRev = std::stoi(m_svGinf[gDticks]);
            m_dCurrentAzPosition = (359.0/m_nNbStepPerRev) * std::stof(m_svGinf[gADAZ]);

            break;

        default :
            bIsMoving  = false;
            break;
    }

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isDomeMoving] isMoving = %s\n", timestamp, bIsMoving?"True":"False");
    fflush(Logfile);
#endif

    return bIsMoving;
}

bool CddwDome::isDomeAtHome()
{
    int nErr = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(!m_bIsHoming)
        return false;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isDomeAtHome]\n", timestamp);
    fflush(Logfile);
#endif

    nErr = readAllResponse(resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    switch(resp[0]) {
        case 'L':
        case 'R':
        case 'T':
        case 'P':
            m_bHomed  = false;
            break;

        case 'V':   // GINF
            m_bHomed  = true;
            m_bIsHoming = false;
            break;

        default :
            m_bHomed  = false;
            break;
    }

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isDomeAtHome] m_bHomed = %s\n", timestamp, m_bHomed?"True":"False");
    fflush(Logfile);
#endif

    return m_bHomed;
  
}

int CddwDome::syncDome(double dAz, double dEl)
{
    return ERR_COMMANDNOTSUPPORTED;
}

int CddwDome::parkDome()
{
    int nErr = DDW_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating || m_bIsHoming || m_bIsGoToing)
        return ERR_COMMANDINPROGRESS;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::parkDome]\n", timestamp);
    fflush(Logfile);
#endif

    nErr = goHome();
    if(nErr)
        return nErr;

    m_bIsHoming = true;

    return nErr;
}

int CddwDome::unparkDome()
{
    int nErr = DDW_OK;

    if(m_bCalibrating || m_bIsHoming || m_bIsGoToing)
        return ERR_COMMANDINPROGRESS;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::unparkDome]\n", timestamp);
    fflush(Logfile);
#endif

    m_bParked = false;
    nErr = goHome();
    if(nErr)
        return nErr;

    m_bIsHoming = true;
    return nErr;
}

int CddwDome::gotoAzimuth(double dNewAz)
{

    int nErr = DDW_OK;
    char buf[SERIAL_BUFFER_SIZE];
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::gotoAzimuth] GoTo %3.2f\n", timestamp, dNewAz);
    fflush(Logfile);
#endif

    snprintf(buf, SERIAL_BUFFER_SIZE, "G%03d", int(dNewAz));
    nErr = domeCommand(buf, resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    switch(resp[0]) {
        case 'L':
        case 'R':
        case 'T':
            nErr = DDW_OK;
            m_dGotoAz = dNewAz;
            break;

        case 'P':
            nErr = DDW_OK;
            m_dGotoAz = dNewAz;
            break;

        default :
            nErr = DDW_BAD_CMD_RESPONSE;
            break;
    }

    return nErr;
}



int CddwDome::openShutter()
{
    int nErr = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating || m_bIsHoming || m_bIsGoToing)
        return ERR_COMMANDINPROGRESS;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::openShutter]\n", timestamp);
    fflush(Logfile);
#endif

    nErr = domeCommand("GOPN", resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    return nErr;
}

int CddwDome::closeShutter()
{
    int nErr = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating || m_bIsHoming || m_bIsGoToing)
        return ERR_COMMANDINPROGRESS;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::closeShutter]\n", timestamp);
    fflush(Logfile);
#endif

    nErr = domeCommand("GCLS", resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    return nErr;
}

int CddwDome::getFirmwareVersion(char *version, int strMaxLen)
{
    int nErr = DDW_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getFirmwareVersion] asking for firmware version from INF packet\n", timestamp);
    fflush(Logfile);
#endif

    if(strlen(m_szFirmwareVersion)){
        strncpy(version, m_szFirmwareVersion, strMaxLen);
        return nErr;
    }

    if(m_bCalibrating || m_bIsHoming || m_bIsGoToing) {
        strncpy(version, "NA", strMaxLen);
        return nErr;
    }

    nErr = getInfRecord();
    if(nErr)
        return nErr;

    strncpy(version, m_svGinf[gVersion].c_str(), strMaxLen);
    strncpy(m_szFirmwareVersion, version, SERIAL_BUFFER_SIZE);

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getFirmwareVersion] Firmware version : %s\n", timestamp, m_szFirmwareVersion);
    fflush(Logfile);
#endif

    return nErr;
}


int CddwDome::goHome()
{
    int nErr = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating || m_bIsHoming || m_bIsGoToing)
        return ERR_COMMANDINPROGRESS;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::goHome]\n", timestamp);
    fflush(Logfile);
#endif

    nErr = domeCommand("GHOM", resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    switch(resp[0]) {
        case 'L':
        case 'R':
        case 'T':
            nErr = DDW_OK;
            break;
        case 'P':
            nErr = DDW_OK;
            break;
        default :
            nErr = DDW_BAD_CMD_RESPONSE;
            break;
    }
    return nErr;
}


int CddwDome::calibrate()
{

    int nErr = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating || m_bIsHoming || m_bIsGoToing)
        return ERR_COMMANDINPROGRESS;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::calibrate]\n", timestamp);
    fflush(Logfile);
#endif

    nErr = domeCommand("GTRN", resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    switch(resp[0]) {
        case 'L':
        case 'R':
        case 'T':
            nErr = DDW_OK;
            m_bCalibrating = true;
            break;
        case 'P':
            nErr = DDW_OK;
            m_bCalibrating = true;
            break;
        default :
            nErr = DDW_BAD_CMD_RESPONSE;
            break;
    }

    return nErr;
}

int CddwDome::isGoToComplete(bool &bComplete)
{
    int nErr = DDW_OK;
    double dDomeAz = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    bComplete = false;

    if(!m_bIsGoToing)
        return SB_OK;

    if(isDomeMoving()) {
        return nErr;
    }

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isGoToComplete]\n", timestamp);
    fflush(Logfile);
#endif

    nErr = getDomeAz(dDomeAz);
    if(nErr)
        return nErr;


    if (ceil(m_dGotoAz) == ceil(dDomeAz)) {
        bComplete = true;
        m_bIsGoToing = false;
    }
    else {
        // we're not moving and we're not at the final destination !!!
#if defined DDW_DEBUG && DDW_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::isGoToComplete] domeAz = %f, mGotoAz = %f.\n", timestamp, ceil(dDomeAz), ceil(m_dGotoAz));
        fflush(Logfile);
#endif
        bComplete = false;
        nErr = ERR_CMDFAILED;
    }

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isGoToComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
    fflush(Logfile);
#endif

    return nErr;
}

int CddwDome::isOpenComplete(bool &bComplete)
{
    int nErr = DDW_OK;
    int state;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating || m_bIsHoming || m_bIsGoToing)
        return nErr;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isOpenComplete]\n", timestamp);
    fflush(Logfile);
#endif

    nErr = getShutterState(state);
    if(nErr)
        return ERR_CMDFAILED;
    if(state == OPEN){
        m_bShutterOpened = true;
        bComplete = true;
        m_dCurrentElPosition = 90.0;
    }
    else {
        m_bShutterOpened = false;
        bComplete = false;
        m_dCurrentElPosition = 0.0;
    }

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isOpenComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
    fflush(Logfile);
#endif

    return nErr;
}

int CddwDome::isCloseComplete(bool &bComplete)
{
    int nErr = DDW_OK;
    int state;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating || m_bIsHoming || m_bIsGoToing)
        return nErr;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isCloseComplete]\n", timestamp);
    fflush(Logfile);
#endif

    nErr = getShutterState(state);
    if(nErr)
        return ERR_CMDFAILED;
    if(state == CLOSED){
        m_bShutterOpened = false;
        bComplete = true;
        m_dCurrentElPosition = 0.0;
    }
    else {
        m_bShutterOpened = true;
        bComplete = false;
        m_dCurrentElPosition = 90.0;
    }

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isOpenComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
    fflush(Logfile);
#endif

    return nErr;
}


int CddwDome::isParkComplete(bool &bComplete)
{
    int nErr = DDW_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isParkComplete]\n", timestamp);
    fflush(Logfile);
#endif

    nErr = isFindHomeComplete(bComplete);
    if(nErr)
        return nErr;

    if(bComplete)
        m_bParked = true;
    return nErr;
}

int CddwDome::isUnparkComplete(bool &bComplete)
{
    int nErr = DDW_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isUnparkComplete]\n", timestamp);
    fflush(Logfile);
#endif

    nErr = isFindHomeComplete(bComplete);
    if(nErr)
        return nErr;

    if(bComplete) {
        m_bParked = false;
    }

    return nErr;
}

int CddwDome::isFindHomeComplete(bool &bComplete)
{
    int nErr = DDW_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isFindHomeComplete]\n", timestamp);
    fflush(Logfile);
#endif

    if(isDomeMoving()) {
        m_bHomed = false;
        bComplete = false;
        return nErr;
    }

    if(isDomeAtHome()){
        m_bHomed = true;
        bComplete = true;
    }
    else {
        // we're not moving and we're not at the home position !!!
#if defined DDW_DEBUG && DDW_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::isFindHomeComplete] Not moving and not at home !!!\n", timestamp);
        fflush(Logfile);
#endif
        bComplete = false;
        m_bHomed = false;
        m_bParked = false;
        nErr = ERR_CMDFAILED;
    }

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isFindHomeComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
    fflush(Logfile);
#endif

    return nErr;
}


int CddwDome::isCalibratingComplete(bool &bComplete)
{
    int nErr = DDW_OK;
    double dDomeAz = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(!m_bCalibrating)
        return nErr;

    if(isDomeMoving()) {
        nErr = getDomeAz(dDomeAz);
        m_bHomed = false;
        bComplete = false;
        return nErr;
    }

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isCalibratingComplete]\n", timestamp);
    fflush(Logfile);
#endif

    nErr = getDomeAz(dDomeAz);
    if(nErr)
        return nErr;
    if (ceil(m_dHomeAz) != ceil(dDomeAz)) {
        // We need to resync the current position to the home position.
        m_dCurrentAzPosition = m_dHomeAz;
        syncDome(m_dCurrentAzPosition, m_dCurrentElPosition);
        m_bHomed = true;
        bComplete = true;
    }

    nErr = getDomeStepPerRev(m_nNbStepPerRev);
    m_bHomed = true;
    bComplete = true;
    m_bCalibrating = false;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isCalibratingComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
    fflush(Logfile);
#endif
    return nErr;
}


int CddwDome::abortCurrentCommand()
{
    int nErr;
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_bCalibrating = false;
    m_bIsHoming = false;
    m_bIsGoToing = false;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::abortCurrentCommand]\n", timestamp);
    fflush(Logfile);
#endif

    nErr = domeCommand("STOP", resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    parseGINF(resp);

    return nErr;
}

int CddwDome::getInfRecord()
{
    int nErr= DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(timer.GetElapsedSeconds() < m_dInfRefreshInterval)
        return nErr;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getInfRecord] Asking for INF record\n", timestamp);
    fflush(Logfile);
#endif

    nErr = domeCommand("GINF", resp, SERIAL_BUFFER_SIZE);
    if(nErr) {
        timer.Reset();
        return nErr;
    }
#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getInfRecord] got INF record : %s \n", timestamp, resp);
    fflush(Logfile);
#endif
    // parse INF packet
    parseGINF(resp);
    timer.Reset();
    return nErr;
}

#pragma mark - Getter / Setter

int CddwDome::getNbTicksPerRev()
{
    int nErr = DDW_OK;
    if(m_bIsConnected)
        nErr = getDomeStepPerRev(m_nNbStepPerRev);
    return m_nNbStepPerRev;
}


double CddwDome::getHomeAz()
{
    if(m_bIsConnected)
        getDomeHomeAz(m_dHomeAz);

    return m_dHomeAz;
}

double CddwDome::getCurrentAz()
{
    if(m_bIsConnected)
        getDomeAz(m_dCurrentAzPosition);
    
    return m_dCurrentAzPosition;
}

double CddwDome::getCurrentEl()
{
    if(m_bIsConnected)
        getDomeEl(m_dCurrentElPosition);
    
    return m_dCurrentElPosition;
}

int CddwDome::getCurrentShutterState()
{
    if(m_bIsConnected)
        getShutterState(m_nShutterState);

    return m_nShutterState;
}


// V4,701,527,4,526,0,1,1,0,522,532,0,128,255,255,255,255,255,255,255,999,5,0


int CddwDome::parseGINF(char *ginf)
{
    int nErr = DDW_OK;
    unsigned int n;
    std::vector<std::string> vFieldsData;

    nErr = parseFields(ginf, vFieldsData, ',');
    // do we have all the fields ?
    if (vFieldsData[gVersion] == "V1")
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


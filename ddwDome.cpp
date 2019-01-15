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
    memset(mLogBuffer,0,ND_LOG_BUFFER_SIZE);

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


int CddwDome::readResponse(char *respBuffer, int bufferLen)
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
            nErr = DDW_BAD_CMD_RESPONSE;
            break;
        }
        totalBytesRead += nBytesRead;
#if defined DDW_DEBUG && DDW_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::readResponse] nBytesRead = %lu\n", timestamp, nBytesRead);
        fflush(Logfile);
#endif
    } while (*bufPtr++ != 0x0D && totalBytesRead < bufferLen );


    if(totalBytesRead)
        *(bufPtr-1) = 0; //remove the \r

    return nErr;
}


int CddwDome::domeCommand(const char *cmd, char *result, int resultMaxLen)
{
    int nErr = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];
    unsigned long  nBytesWrite;

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
    if(nErr)
        return nErr;

    if(result)
        strncpy(result, &resp[1], resultMaxLen);

    return nErr;

}

int CddwDome::getDomeAz(double &domeAz)
{
    int nErr = DDW_OK;
    double domeAzticks;
    double domeTicksPerRev;
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = getInfRecord();
    if(nErr)
        return nErr;

    domeAzticks =  std::stof(m_svGinf[gADAZ]);
    domeTicksPerRev = std::stof(m_svGinf[gDticks]);

    domeAz = (359.0/domeTicksPerRev) * domeAzticks;
    m_dCurrentAzPosition = domeAz;

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
    double HomeAzticks;
    double domeTicksPerRev;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = getInfRecord();
    if(nErr)
        return nErr;

    HomeAzticks = std::stof(m_svGinf[gHomeAz]);
    domeTicksPerRev = std::stof(m_svGinf[gDticks]);

    Az = (359.0/domeTicksPerRev) * HomeAzticks;
    m_dHomeAz = Az;

    return nErr;
}



int CddwDome::getShutterState(int &state)
{
    int nErr = DDW_OK;
    int shutterState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = getInfRecord();
    if(nErr)
        return nErr;

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

    // state = atoi(resp);

    return nErr;
}


int CddwDome::getDomeStepPerRev(int &stepPerRev)
{
    int nErr = DDW_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getInfRecord();
    if(nErr)
        return nErr;

    stepPerRev =  std::stoi(m_svGinf[gDticks]);

    m_nNbStepPerRev = stepPerRev;
    return nErr;
}


bool CddwDome::isDomeMoving()
{
    bool isMoving;
    int nErr = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    isMoving = false;

    nErr = readResponse(resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    switch(resp[0]) {
        case 'L':
        case 'R':
        case 'T':
        case 'P':
            isMoving  = true;
            break;

        case 'V':   // GINF
            isMoving  = false;
            break;

        default :
            isMoving  = false;
            break;
    }
    return isMoving;
}

bool CddwDome::isDomeAtHome()
{
    int nErr = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(!m_bIsHoming)
        return false;

    nErr = readResponse(resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    switch(resp[0]) {
        case 'L':
        case 'R':
        case 'T':
        case 'P':
            m_bHomed  = true;
            break;

        case 'V':   // GINF
            m_bHomed  = false;
            break;

        default :
            m_bHomed  = false;
            break;
    }

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

    if(m_bCalibrating)
        return SB_OK;

    nErr = goHome();
    if(nErr)
        return nErr;

    m_bIsHoming = true;

    return nErr;
}

int CddwDome::unparkDome()
{
    int nErr = DDW_OK;

    m_bParked = false;
    nErr = goHome();
    if(nErr)
        return nErr;

    m_bIsHoming = true;
    return nErr;
}

int CddwDome::gotoAzimuth(double newAz)
{

    int nErr = DDW_OK;
    char buf[SERIAL_BUFFER_SIZE];
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(buf, SERIAL_BUFFER_SIZE, "G%03d", int(newAz));
    nErr = domeCommand(buf, resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    switch(resp[0]) {
        case 'L':
        case 'R':
        case 'T':
            nErr = DDW_OK;
            m_dGotoAz = newAz;
            break;

        case 'P':
            nErr = DDW_OK;
            m_dGotoAz = newAz;
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

    if(m_bCalibrating)
        return ERR_COMMANDINPROGRESS;

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

    if(m_bCalibrating)
        return ERR_COMMANDINPROGRESS;

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

    if(m_bCalibrating)
        return SB_OK;

    nErr = getInfRecord();
    if(nErr)
        return nErr;

    strcpy(version, m_svGinf[gVersion].c_str());
    return nErr;
}


int CddwDome::goHome()
{
    int nErr = DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

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

    nErr = getDomeAz(dDomeAz);
    if(nErr)
        return nErr;

    if(isDomeMoving()) {
        return nErr;
    }


    if (ceil(m_dGotoAz) == ceil(dDomeAz))
        bComplete = true;
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

    return nErr;
}

int CddwDome::isOpenComplete(bool &complete)
{
    int nErr = DDW_OK;
    int state;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getShutterState(state);
    if(nErr)
        return ERR_CMDFAILED;
    if(state == OPEN){
        m_bShutterOpened = true;
        complete = true;
        m_dCurrentElPosition = 90.0;
    }
    else {
        m_bShutterOpened = false;
        complete = false;
        m_dCurrentElPosition = 0.0;
    }

    return nErr;
}

int CddwDome::isCloseComplete(bool &complete)
{
    int nErr = DDW_OK;
    int state;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getShutterState(state);
    if(nErr)
        return ERR_CMDFAILED;
    if(state == CLOSED){
        m_bShutterOpened = false;
        complete = true;
        m_dCurrentElPosition = 0.0;
    }
    else {
        m_bShutterOpened = true;
        complete = false;
        m_dCurrentElPosition = 90.0;
    }

    return nErr;
}


int CddwDome::isParkComplete(bool &bComplete)
{
    int nErr = DDW_OK;
    nErr = isFindHomeComplete(bComplete);
    return nErr;
}

int CddwDome::isUnparkComplete(bool &bComplete)
{
    int nErr = DDW_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = isFindHomeComplete(bComplete);
    if(nErr)
        return nErr;

    if(bComplete) {
        m_bParked = false;
    }

    return nErr;
}

int CddwDome::isFindHomeComplete(bool &complete)
{
    int nErr = DDW_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(isDomeMoving()) {
        m_bHomed = false;
        complete = false;
        return nErr;
    }

    if(isDomeAtHome()){
        m_bHomed = true;
        complete = true;
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
        complete = false;
        m_bHomed = false;
        m_bParked = false;
        nErr = ERR_CMDFAILED;
    }

    return nErr;
}


int CddwDome::isCalibratingComplete(bool &complete)
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
        complete = false;
        return nErr;
    }

    
    nErr = getDomeAz(dDomeAz);
    if(nErr)
        return nErr;
    if (ceil(m_dHomeAz) != ceil(dDomeAz)) {
        // We need to resync the current position to the home position.
        m_dCurrentAzPosition = m_dHomeAz;
        syncDome(m_dCurrentAzPosition, m_dCurrentElPosition);
        m_bHomed = true;
        complete = true;
    }

    nErr = getDomeStepPerRev(m_nNbStepPerRev);
    m_bHomed = true;
    complete = true;
    m_bCalibrating = false;
    return nErr;
}


int CddwDome::abortCurrentCommand()
{
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_bCalibrating = false;

    return (domeCommand("STOP\r", NULL, SERIAL_BUFFER_SIZE));
}

int CddwDome::getInfRecord()
{
    int nErr= DDW_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(timer.GetElapsedSeconds() < m_dInfRefreshInterval)
        return nErr;

    timer.Reset();
    nErr = domeCommand("GINF", resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // parse INF packet
    parseGINF(resp);
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

int CddwDome::setHomeAz(double dAz)
{
    int nErr = DDW_OK;
    char buf[SERIAL_BUFFER_SIZE];
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(buf, SERIAL_BUFFER_SIZE, "HOME %3.1f\r", dAz);
    nErr = domeCommand(buf, resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    if(strncmp(resp,"A",1) == 0) {
        nErr = DDW_OK;
    }
    else {
        nErr = DDW_BAD_CMD_RESPONSE;
    }
    m_dHomeAz = dAz;
    return nErr;
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
    int n;
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


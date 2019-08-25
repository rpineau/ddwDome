//
//  ddwDome.cpp
//  DDW rotation drive unit for TI Dome X2 plugin
//
//  Created by Rodolphe Pineau on 2017-04-13.


#include "ddwDome.h"


CddwDome::CddwDome()
{
    // set some sane values
    m_pSerx = NULL;
    m_bIsConnected = false;

    m_nNbStepPerRev = 0;
    m_dShutterBatteryVolts = 0.0;
    
    m_dHomeAz = 180;

    m_dCurrentAzPosition = 0.0;
    m_dCurrentElPosition = 0.0;
    
    m_dDeadZoneDeg = 0.0;
    
    m_dCoastDeg = 0.0;
    
    m_bDomeIsMoving = false;
    m_bShutterIsMoving = false;
    m_bHasShutter = false;
    m_bShutterOpened = false;
    
    m_bParked = true;

    memset(m_szFirmwareVersion,0,SERIAL_BUFFER_SIZE);

    timer.Reset();
    dataReceivedTimer.Reset();
    m_dInfRefreshInterval = 2;
	
#ifdef DDW_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\X2_DDWLog.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/X2_DDWLog.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/X2_DDWLog.txt";
#endif
    Logfile = fopen(m_sLogfilePath.c_str(), "w");
#endif

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CddwDome::CddwDome] Version 2019_08_24_1810.\n", timestamp);
    fprintf(Logfile, "[%s] [CddwDome::CddwDome] Constructor Called.\n", timestamp);
    fflush(Logfile);
#endif

}

CddwDome::~CddwDome()
{

}

int CddwDome::Connect(const char *szPort, bool bHardwareFlowControl)
{
    int nErr;
    int nTimeout;
    bool bComplete;

    m_bIsConnected = true;
#if defined DDW_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::Connect] Connecting to %s with%s hardware control.\n", timestamp, szPort, bHardwareFlowControl?"":"out");
    fflush(Logfile);
#endif

    if(bHardwareFlowControl)
        nErr = m_pSerx->open(szPort, 9600, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1 -RTS_CONTROL 1");
    else
        nErr = m_pSerx->open(szPort, 9600, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1");

    if(nErr) {
        m_bIsConnected = false;
        return ERR_COMMNOLINK;
    }

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
        m_pSerx->close();
        m_pSleeper->sleep(int(m_dInfRefreshInterval*1000));
        return nErr;
    }

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::Connect] Got Firmware : %s\n", timestamp, m_szFirmwareVersion);
    fflush(Logfile);
#endif

    // get current state from DDW
    getDomeStepPerRev();
    getDomeHomeAz();
    getDomeAz(m_dCurrentAzPosition);
    getShutterState();
    getCoast();
    getDeadZone();
    
    // check if we're home but current Az != home Az
    if(isDomeAtHome()) {
#if defined DDW_DEBUG && DDW_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::Connect] m_dHomeAz : %3.2f\n", timestamp, m_dHomeAz);
        fprintf(Logfile, "[%s] [CddwDome::Connect] m_dCurrentAzPosition : %3.2f\n", timestamp, m_dCurrentAzPosition);
        fprintf(Logfile, "[%s] [CddwDome::Connect] dCoast : %3.2f\n", timestamp, m_dCoastDeg);
        fflush(Logfile);
#endif
        if( m_dCurrentAzPosition  < (m_dHomeAz - m_dCoastDeg) || m_dCurrentAzPosition  > ( m_dHomeAz + m_dCoastDeg) ) {
#if defined DDW_DEBUG && DDW_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CddwDome::Connect] neaed to resync on home sensor\n", timestamp);
            fprintf(Logfile, "[%s] [CddwDome::Connect] goto m_dCurrentAzPosition - m_dCoastDeg*1.5 : %3.2f\n", timestamp, m_dCurrentAzPosition - (m_dCoastDeg*1.5));
            fflush(Logfile);
#endif
            gotoAzimuth(m_dCurrentAzPosition - (m_dCoastDeg * 1.5));
            nTimeout = 0;
            bComplete = false;
            while(!bComplete && nTimeout<5) {
                m_pSleeper->sleep(2000);
                nTimeout++;
                isGoToComplete(bComplete);
            }
            if(nTimeout == 5)
                return ERR_CMDFAILED;
#if defined DDW_DEBUG && DDW_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CddwDome::Connect] now find home sensor\n", timestamp);
            fflush(Logfile);
#endif
            goHome();
            bComplete = false;
            while(!bComplete && nTimeout<5) {
                m_pSleeper->sleep(2000);
                nTimeout++;
                isFindHomeComplete(bComplete);
            }
            if(nTimeout == 5)
                return ERR_CMDFAILED;
            getDomeAz(m_dCurrentAzPosition);
        }
    }

    return SB_OK;
}


void CddwDome::Disconnect()
{
    if(m_bIsConnected) {
        m_pSerx->purgeTxRx();
        m_pSerx->close();
    }
    m_bIsConnected = false;
}

#pragma mark - DDW copmunications

int CddwDome::domeCommand(const char *cmd, char *result, unsigned int resultMaxLen, unsigned int nTimeout)
{
    int nErr = DDW_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned long  nBytesWrite;
    int nNbTimeout = 0;
    int nMaxNbTimeout = 3;

    do {
        m_pSerx->purgeTxRx();
    #if defined DDW_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::domeCommand] Sending %s\n", timestamp, cmd);
        fflush(Logfile);
    #endif

        nErr = m_pSerx->writeFile((void *)cmd, strlen(cmd), nBytesWrite);
        m_pSerx->flushTx();
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
        nErr = readResponse(szResp, SERIAL_BUFFER_SIZE, nTimeout);
        if (nErr == DDW_TIMEOUT) {
            if(nNbTimeout >= nMaxNbTimeout) // make sure we don't end up in an infinite loop
                return ERR_NORESPONSE;
            nNbTimeout++;
            m_pSleeper->sleep(1500);    // wait 1.5 second and resend command
        }
    } while (nErr == DDW_TIMEOUT);
	
#if defined DDW_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CddwDome::domeCommand] Response %s\n", timestamp, szResp);
	fflush(Logfile);
#endif
	
    if(result)
        strncpy(result, szResp, resultMaxLen);

    return nErr;

}

int CddwDome::readResponse(char *respBuffer, unsigned int bufferLen, unsigned int nTimeout)
{
    int nErr = DDW_OK;
    unsigned long nBytesRead = 0;
    unsigned long totalBytesRead = 0;
    char *bufPtr;

    memset(respBuffer, 0, (size_t) bufferLen);
    bufPtr = respBuffer;

    do {
        nErr = m_pSerx->readFile(bufPtr, 1, nBytesRead, nTimeout);
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

        if (nBytesRead !=1) {// timeout
#if defined DDW_DEBUG && DDW_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CddwDome::readResponse] readFile Timeout\n", timestamp);
            fflush(Logfile);
#endif
            if(totalBytesRead)    // some reponse do not end with \r\r
                nErr = DDW_OK;
            else
                nErr = DDW_TIMEOUT; // no response at all, we'll need to retry

            return nErr;
            break;
        }
        totalBytesRead += nBytesRead;
#if defined DDW_DEBUG && DDW_DEBUG >= 3
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::readResponse] totalBytesRead = %lu\n", timestamp, totalBytesRead);
        fprintf(Logfile, "[%s] [CddwDome::readResponse] respBuffer = '%s'\n", timestamp, respBuffer);
        fflush(Logfile);
#endif
    } while (*bufPtr++ != 0x0D && totalBytesRead < bufferLen );


    if(totalBytesRead && (*(bufPtr-1) == 0x0D))
        *(bufPtr-1) = 0; //remove the \r

    return nErr;
}

// read all the response, only keep the last one.
int CddwDome::readAllResponses(char *respBuffer, unsigned int bufferLen)
{
    int nErr = DDW_OK;
    int nbByteWaiting = 0;

	memset(respBuffer, 0, bufferLen);
    do {
        m_pSerx->bytesWaitingRx(nbByteWaiting);
		if(nbByteWaiting)
            nErr = readResponse(respBuffer, bufferLen, 250);
    } while(nbByteWaiting);

    return nErr;
}


int CddwDome::getInfRecord()
{
    int nErr= DDW_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(timer.GetElapsedSeconds() < m_dInfRefreshInterval)
        return nErr;

#if defined DDW_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CddwDome::getInfRecord] *********************** \n", timestamp);
	fflush(Logfile);
#endif

    if(m_bDomeIsMoving || m_bShutterIsMoving) {
#if defined DDW_DEBUG && DDW_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
		fprintf(Logfile, "[%s] [CddwDome::getInfRecord] Movement in progress m_bDomeIsMoving = %s, m_bShutterIsMoving = %s\n", timestamp, m_bDomeIsMoving?"True":"False", m_bShutterIsMoving?"True":"False");
        fflush(Logfile);
#endif
        return ERR_COMMANDINPROGRESS;
    }
    
    
#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getInfRecord] Asking for INF record\n", timestamp);
    fflush(Logfile);
#endif
    
    nErr = domeCommand("GINF", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
        timer.Reset();
        return nErr;
    }
#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getInfRecord] got INF record : %s \n", timestamp, szResp);
    fflush(Logfile);
#endif
    // parse INF packet
    if(strlen(szResp))  // no error, let's look at the response
        parseGINF(szResp);
    
    timer.Reset();
    return nErr;
}

#pragma mark - Private Getters

int CddwDome::getDomeAz(double &domeAz)
{
    int nErr = DDW_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined DDW_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CddwDome::getDomeAz] ***********************\n", timestamp);
	fflush(Logfile);
#endif


	if(m_bDomeIsMoving || m_bShutterIsMoving) {
#if defined DDW_DEBUG && DDW_DEBUG >= 2
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::getDomeAz] Movement in progress m_bDomeIsMoving = %s , m_bShutterIsMoving = %s  \n", timestamp, m_bDomeIsMoving?"True":"False", m_bShutterIsMoving?"True":"False");
		fflush(Logfile);
#endif
        domeAz = m_dCurrentAzPosition;  // should be updated when checking if dome is moving
        return nErr;
    }

    nErr = getInfRecord();
    if(nErr)
        return nErr;

    try {
        m_nNbStepPerRev = std::stoi(m_svGinf[gDticks]);
        m_dCurrentAzPosition = (360.0/m_nNbStepPerRev) * std::stof(m_svGinf[gADAZ]);
    } catch(const std::exception& e) {
#if defined DDW_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::getDomeAz] std::stof or std::stoi exception : %s\n", timestamp, e.what());
        fflush(Logfile);
#endif
        return ERR_DATAOUT;
    }
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

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    getShutterState();

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


int CddwDome::getDomeHomeAz()
{
    int nErr = DDW_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bDomeIsMoving || m_bShutterIsMoving)
        return nErr;

#if defined DDW_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getDomeHomeAz] ***********************\n", timestamp);
    fflush(Logfile);
#endif

    nErr = getInfRecord();
    if(nErr)
        return nErr;

    try {
        if(!m_nNbStepPerRev)
            m_nNbStepPerRev = std::stoi(m_svGinf[gDticks]);
        m_dHomeAz = (360.0/m_nNbStepPerRev) * std::stof(m_svGinf[gHomeAz]);
    } catch(const std::exception& e) {
#if defined DDW_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::getDomeHomeAz] std::stof or std::stoi exception : %s\n", timestamp, e.what());
        fflush(Logfile);
#endif
        return ERR_DATAOUT;
    }


#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getDomeHomeAz] m_dHomeAz = %3.2f\n", timestamp, m_dHomeAz);
    fflush(Logfile);
#endif

    return nErr;
}

int CddwDome::getCoast()
{
    int nErr = DDW_OK;
    int nNbStepCoast;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bDomeIsMoving | m_bShutterIsMoving)
        return nErr;

#if defined DDW_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getCoast] ***********************\n", timestamp);
    fflush(Logfile);
#endif

    nErr = getInfRecord();
    if(nErr)
        return nErr;

    try {
        if(!m_nNbStepPerRev)
            m_nNbStepPerRev = std::stoi(m_svGinf[gDticks]);
        nNbStepCoast = std::stoi(m_svGinf[gCoast]);
    } catch(const std::exception& e) {
#if defined DDW_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::getCoast] std::stoi exception : %s\n", timestamp, e.what());
        fflush(Logfile);
#endif
        return ERR_DATAOUT;
    }
    m_dCoastDeg = (360.0/m_nNbStepPerRev) * nNbStepCoast;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getCoast]Coast in degrees : %3.2f\n", timestamp, m_dCoastDeg);
    fflush(Logfile);
#endif

    return nErr;
}

int CddwDome::getDeadZone()
{
    int nErr = DDW_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bDomeIsMoving | m_bShutterIsMoving)
        return nErr;

#if defined DDW_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getDeadZone] ***********************\n", timestamp);
    fflush(Logfile);
#endif

    nErr = getInfRecord();
    if(nErr)
        return nErr;
    
    try {
        m_dDeadZoneDeg = std::stoi(m_svGinf[gINTDZ]);
    } catch(const std::exception& e) {
#if defined DDW_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::getDeadZone] std::stoi exception : %s\n", timestamp, e.what());
        fflush(Logfile);
#endif
        return ERR_DATAOUT;
    }

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getDeadZone] DeadZone in degrees : %3.2f\n", timestamp, m_dDeadZoneDeg);
    fflush(Logfile);
#endif
    
    return nErr;
}


int CddwDome::getShutterState()
{
    int nErr = DDW_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined DDW_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CddwDome::getShutterState] ***********************\n", timestamp);
	fflush(Logfile);
#endif


	if(m_bShutterIsMoving) {
#if defined DDW_DEBUG && DDW_DEBUG >= 2
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(Logfile, "[%s] [CddwDome::getShutterState] Movement in progress m_bShutterIsMoving = %s\n", timestamp, m_bShutterIsMoving?"True":"False");
		fflush(Logfile);
#endif
		return ERR_COMMANDINPROGRESS;
	}

    m_bShutterOpened = false;
    
	nErr = getInfRecord();
	if(nErr)
		return nErr;

    try {
        m_nShutterState = std::stoi(m_svGinf[gShutter]);
    } catch(const std::exception& e) {
#if defined DDW_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::getShutterState] std::stoi exception : %s\n", timestamp, e.what());
        fflush(Logfile);
#endif
        return ERR_DATAOUT;
    }

    switch(m_nShutterState) {
        case OPEN:
            m_bShutterOpened = true;
            break;

        case CLOSED:
		case UNKNOWN:
		default:
            m_bShutterOpened = false;
            break;
    }

#if defined DDW_DEBUG && DDW_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CddwDome::getShutterState] shutterState = %d\n", timestamp, m_nShutterState);
	fflush(Logfile);
#endif

	
    return nErr;
}


int CddwDome::getDomeStepPerRev()
{
    int nErr = DDW_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined DDW_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getDomeStepPerRev] ***********************\n", timestamp);
    fflush(Logfile);
#endif

    if(!m_bDomeIsMoving && !m_bShutterIsMoving)  {
        nErr = getInfRecord();
        if(nErr)
            return nErr;
    }

    try {
        m_nNbStepPerRev =  std::stoi(m_svGinf[gDticks]);
    } catch(const std::exception& e) {
#if defined DDW_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::getDomeStepPerRev] std::stoi exception : %s\n", timestamp, e.what());
        fflush(Logfile);
#endif
        return ERR_DATAOUT;
    }
    return nErr;
}

int CddwDome::getFirmwareVersion(char *version, int strMaxLen)
{
    int nErr = DDW_OK;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
#if defined DDW_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getFirmwareVersion] ***********************\n", timestamp);
    fflush(Logfile);
#endif
    
    if(strlen(m_szFirmwareVersion)){
#if defined DDW_DEBUG && DDW_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::getFirmwareVersion] m_szFirmwareVersion not empty, no need to ask again\n", timestamp);
        fflush(Logfile);
#endif
        strncpy(version, m_szFirmwareVersion, strMaxLen);
        return nErr;
    }
    
    if(m_bDomeIsMoving || m_bShutterIsMoving) {
        strncpy(version, "NA", strMaxLen);
        return nErr;
    }
    
#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getFirmwareVersion] calling getInfRecord();\n", timestamp);
    fflush(Logfile);
#endif
    
    nErr = getInfRecord();
    if(nErr)
        return nErr;
    
#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::getFirmwareVersion] back from getInfRecord();\n", timestamp);
    fflush(Logfile);
#endif
    
    if(!m_svGinf.size())
        return ERR_CMDFAILED;
    
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


#pragma mark - Movements

int CddwDome::gotoAzimuth(double dNewAz)
{

    int nErr = DDW_OK;
    char buf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    int nConvErr;
    std::vector<std::string> vFieldsData;
    double dDomeAz;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined DDW_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CddwDome::gotoAzimuth] ***********************\n", timestamp);
	fflush(Logfile);
#endif

	if(m_bDomeIsMoving || m_bShutterIsMoving) {
#if defined DDW_DEBUG && DDW_DEBUG >= 2
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::gotoAzimuth] Movement in progress m_bDomeIsMoving = %s , m_bShutterIsMoving = %s\n", timestamp, m_bDomeIsMoving?"True":"False", m_bShutterIsMoving?"True":"False");
		fflush(Logfile);
#endif
		return ERR_COMMANDINPROGRESS;
	}

#if defined DDW_DEBUG && DDW_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CddwDome::gotoAzimuth] GoTo %3.2f\n", timestamp, dNewAz);
	fflush(Logfile);
#endif

    m_bDomeIsMoving = false;    // let's not assume it's moving
	m_dGotoAz = dNewAz;
    snprintf(buf, SERIAL_BUFFER_SIZE, "G%03d", int(dNewAz));
    nErr = domeCommand(buf, szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
        return nErr;
    }

    if(strlen(szResp)) {  // no error, let's look at the response
        switch(szResp[0]) {
            case 'V':
                parseGINF(szResp);
                m_bDomeIsMoving = false;
                try {
                    m_nNbStepPerRev = std::stoi(m_svGinf[gDticks]);
                    m_dCurrentAzPosition = (360.0/m_nNbStepPerRev) * std::stof(m_svGinf[gADAZ]);
                } catch(const std::exception& e) {
#if defined DDW_DEBUG
                    ltime = time(NULL);
                    timestamp = asctime(localtime(&ltime));
                    timestamp[strlen(timestamp) - 1] = 0;
                    fprintf(Logfile, "[%s] [CddwDome::gotoAzimuth] std::stof or std::stoi exception : %s\n", timestamp, e.what());
                    fflush(Logfile);
#endif
                    return ERR_DATAOUT;
                }

    #if defined DDW_DEBUG && DDW_DEBUG >= 2
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CddwDome::gotoAzimuth] GINF response means the goto is too small to move the dome. So goto is done. m_bDomeIsMoving = %s\n", timestamp, m_bDomeIsMoving?"True":"False");
                fflush(Logfile);
    #endif
                break;
            case 'L':
            case 'R':
                m_bDomeIsMoving = true;
                nErr = DDW_OK;
                if(strlen(szResp)>1) {
                    // is there a P in there too ?
                    if(szResp[1] == 'P') {
                        nConvErr = parseFields(szResp, vFieldsData, 'P');
                        if(!nConvErr && m_nNbStepPerRev && vFieldsData.size()>1) {
                            try {
                                dDomeAz = (360.0/m_nNbStepPerRev) * std::stof(vFieldsData[1]);
                            } catch(const std::exception& e) {
#if defined DDW_DEBUG
                                ltime = time(NULL);
                                timestamp = asctime(localtime(&ltime));
                                timestamp[strlen(timestamp) - 1] = 0;
                                fprintf(Logfile, "[%s] [CddwDome::gotoAzimuth] std::stof exception : %s\n", timestamp, e.what());
                                fflush(Logfile);
#endif
                                return ERR_DATAOUT;
                            }
                            if ((ceil(m_dGotoAz) <= (ceil(dDomeAz) + m_dDeadZoneDeg) ) && (ceil(m_dGotoAz) >= (ceil(dDomeAz) - m_dDeadZoneDeg) )) {
                                m_bDomeIsMoving = false;
                            }
                        }
                    }
                }
            case 'T':
                m_bDomeIsMoving = true;
                nErr = DDW_OK;
                break;

            case 'P':
                m_bDomeIsMoving = true;
                nErr = DDW_OK;
                nConvErr = parseFields(szResp, vFieldsData, 'P');
                if(!nConvErr && m_nNbStepPerRev && vFieldsData.size()) {
                    try {
                        dDomeAz = (360.0/m_nNbStepPerRev) * std::stof(vFieldsData[0]);
                    } catch(const std::exception& e) {
#if defined DDW_DEBUG
                        ltime = time(NULL);
                        timestamp = asctime(localtime(&ltime));
                        timestamp[strlen(timestamp) - 1] = 0;
                        fprintf(Logfile, "[%s] [CddwDome::gotoAzimuth] std::stof exception : %s\n", timestamp, e.what());
                        fflush(Logfile);
#endif
                        return ERR_DATAOUT;
                    }
                    if ((ceil(m_dGotoAz) <= (ceil(dDomeAz) + m_dDeadZoneDeg) ) && (ceil(m_dGotoAz) >= (ceil(dDomeAz) - m_dDeadZoneDeg) )) {
                        m_bDomeIsMoving = false;
                    }
                }
                break;

            default :
                m_bDomeIsMoving = false;
                nErr = DDW_BAD_CMD_RESPONSE;
                break;
        }
    }
    dataReceivedTimer.Reset();

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::gotoAzimuth] m_dCurrentAzPosition = %3.2f, m_bDomeIsMoving = %s, m_bShutterIsMoving = %s\n", timestamp, m_dCurrentAzPosition, m_bDomeIsMoving?"True":"False", m_bShutterIsMoving?"True":"False");
    fflush(Logfile);
#endif

    return nErr;
}

int CddwDome::goHome()
{
    int nErr = DDW_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    int nTmpAz;
    int nTmphomeAz;
    bool bAtHome;
    bool bIsGotoDone;
    int nTimeout;
    int nTmp;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
#if defined DDW_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::goHome] ***********************\n", timestamp);
    fflush(Logfile);
#endif
    
    if(m_bDomeIsMoving || m_bShutterIsMoving) {
#if defined DDW_DEBUG && DDW_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::goHome] Movement in progress m_bDomeIsMoving = %s, m_bShutterIsMoving = %s\n", timestamp, m_bDomeIsMoving?"True":"False", m_bShutterIsMoving?"True":"False");
        fflush(Logfile);
#endif
        return ERR_COMMANDINPROGRESS;
    }
    
    m_bDomeIsMoving = false;
    nErr = domeCommand("GHOM", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
        return nErr;
    }
    
    if(strlen(szResp)) {  // no error, let's look at the response
        switch(szResp[0]) {
            case 'V':
                parseGINF(szResp);
                try {
                    nTmp = std::stoi(m_svGinf[gHome]);
                } catch(const std::exception& e) {
#if defined DDW_DEBUG
                    ltime = time(NULL);
                    timestamp = asctime(localtime(&ltime));
                    timestamp[strlen(timestamp) - 1] = 0;
                    fprintf(Logfile, "[%s] [CddwDome::goHome] std::stoi exception : %s\n", timestamp, e.what());
                    fflush(Logfile);
#endif
                    return ERR_CMDFAILED;
                }

                if( nTmp == AT_HOME) {  // we're already home ?
                    // check that the current position and the home position aggree
                    try {
                        nTmpAz = std::stoi(m_svGinf[gADAZ]);
                        nTmphomeAz = std::stoi(m_svGinf[gHomeAz]);
                    } catch(const std::exception& e) {
#if defined DDW_DEBUG
                        ltime = time(NULL);
                        timestamp = asctime(localtime(&ltime));
                        timestamp[strlen(timestamp) - 1] = 0;
                        fprintf(Logfile, "[%s] [CddwDome::goHome] std::stof or std::stoi exception : %s\n", timestamp, e.what());
                        fflush(Logfile);
#endif
                        return ERR_CMDFAILED;
                    }

                    if( nTmpAz < (nTmphomeAz - m_dCoastDeg) || nTmpAz > (nTmphomeAz + m_dCoastDeg)) {
                        // we're  home but the dome az is wrong, let's move off and back home, hopping the controller will correct the position
                        // when the sensor transition happens.
#if defined DDW_DEBUG && DDW_DEBUG >= 2
                        ltime = time(NULL);
                        timestamp = asctime(localtime(&ltime));
                        timestamp[strlen(timestamp) - 1] = 0;
                        fprintf(Logfile, "[%s] [CddwDome::goHome] not home, moving %3.2f degree off\n", timestamp, m_dCoastDeg + 1.0);
                        fflush(Logfile);
#endif
                        bIsGotoDone = false;
                        nTimeout = 0;
                        gotoAzimuth(m_dCurrentAzPosition + m_dDeadZoneDeg + 1.0); // move by INTDZ+1 degree off to make sure there is a movement
                        do {
                            m_pSleeper->sleep(1000);
                            isGoToComplete(bIsGotoDone);
                            nTimeout++;
                        } while (!bIsGotoDone && nTimeout<60);    // 60 seconds of timeout should be enough
#if defined DDW_DEBUG && DDW_DEBUG >= 2
                        ltime = time(NULL);
                        timestamp = asctime(localtime(&ltime));
                        timestamp[strlen(timestamp) - 1] = 0;
                        fprintf(Logfile, "[%s] [CddwDome::goHome] not home, moving back home\n", timestamp);
                        fflush(Logfile);
#endif
                        bAtHome = false;
                        nTimeout = 0;
                        nErr = domeCommand("GHOM", szResp, SERIAL_BUFFER_SIZE); // go back home
                        do {
                            m_pSleeper->sleep(1000);
                            isFindHomeComplete(bAtHome);
                            nTimeout++;
                        } while (!bAtHome && nTimeout<60); // the timeout is just here for safety (1 minute).
                    }
                    m_bDomeIsMoving = false;
                }
                break;
                
            case 'L':
            case 'R':
            case 'T':
                nErr = DDW_OK;
                m_bDomeIsMoving = true;
                break;
                
            case 'P':
                nErr = DDW_OK;
                m_bDomeIsMoving = true;
                break;
                
            default :
                nErr = DDW_BAD_CMD_RESPONSE;
                m_bDomeIsMoving = false;
                break;
        }
    }
    dataReceivedTimer.Reset();
    return nErr;
}

int CddwDome::openShutter()
{
    int nErr = DDW_OK;
    char szResp[SERIAL_BUFFER_SIZE];
	int shutterState;
	
    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined DDW_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CddwDome::openShutter] ***********************\n", timestamp);
	fflush(Logfile);
#endif


	if(m_bDomeIsMoving || m_bShutterIsMoving) {
#if defined DDW_DEBUG && DDW_DEBUG >= 2
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::openShutter] Movement in progress m_bDomeIsMoving = %s, m_bShutterIsMoving = %s\n", timestamp, m_bDomeIsMoving?"True":"False", m_bShutterIsMoving?"True":"False");
		fflush(Logfile);
#endif
		return ERR_COMMANDINPROGRESS;
	}


	nErr = domeCommand("GOPN", szResp, SERIAL_BUFFER_SIZE, 10000); // 10 second timeout
    if(nErr)
        return nErr;

    m_bShutterIsMoving = true;
	if(strlen(szResp) && szResp[0] == 'V') {
		//if we got an INF packet we're not moving
		m_bShutterIsMoving = false;
		parseGINF(szResp);
        try {
            shutterState = std::stoi(m_svGinf[gShutter]);
        } catch(const std::exception& e) {
#if defined DDW_DEBUG
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CddwDome::openShutter] std::stoi exception : %s\n", timestamp, e.what());
            fflush(Logfile);
#endif
            return ERR_CMDFAILED;
        }

        switch(shutterState) {
			case OPEN:
				m_bShutterOpened = true;
				break;
				
			case CLOSED:
			case UNKNOWN:
			default:
				m_bShutterOpened = false;
				break;
		}
	}

    dataReceivedTimer.Reset();
    return nErr;
}

int CddwDome::closeShutter()
{
    int nErr = DDW_OK;
    char szResp[SERIAL_BUFFER_SIZE];
	int shutterState;
	
    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined DDW_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CddwDome::closeShutter] ***********************\n", timestamp);
	fflush(Logfile);
#endif

    if(m_bDomeIsMoving || m_bShutterIsMoving) {
#if defined DDW_DEBUG && DDW_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::closeShutter] Movement in progress m_bDomeIsMoving = %s, m_bShutterIsMoving = %s\n", timestamp, m_bDomeIsMoving?"True":"False", m_bShutterIsMoving?"True":"False");
        fflush(Logfile);
#endif
        return ERR_COMMANDINPROGRESS;
    }

	nErr = domeCommand("GCLS", szResp, SERIAL_BUFFER_SIZE, 10000); // 10 second timeout
    if(nErr)
        return nErr;

    m_bShutterIsMoving = true;
	if(strlen(szResp) && szResp[0] == 'V') {
		//if we got an INF packet we're not moving
		m_bShutterIsMoving = false;
		parseGINF(szResp);
        try {
            shutterState = std::stoi(m_svGinf[gShutter]);
        } catch(const std::exception& e) {
#if defined DDW_DEBUG
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CddwDome::closeShutter] std::stoi exception : %s\n", timestamp, e.what());
            fflush(Logfile);
#endif
            return ERR_CMDFAILED;
        }

        switch(shutterState) {
			case OPEN:
				m_bShutterOpened = true;
				break;
				
			case CLOSED:
			case UNKNOWN:
			default:
				m_bShutterOpened = false;
				break;
		}
	}

    dataReceivedTimer.Reset();
    return nErr;
}




int CddwDome::parkDome()
{
    int nErr = DDW_OK;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined DDW_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CddwDome::parkDome] ***********************\n", timestamp);
	fflush(Logfile);
#endif

    if(m_bDomeIsMoving || m_bShutterIsMoving) {
#if defined DDW_DEBUG && DDW_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::parkDome]Movement in progress m_bDomeIsMoving = %s, m_bShutterIsMoving = %s\n", timestamp, m_bDomeIsMoving?"True":"False", m_bShutterIsMoving?"True":"False");
        fflush(Logfile);
#endif
        return ERR_COMMANDINPROGRESS;
    }
    

    nErr = goHome();
    return nErr;
}

int CddwDome::unparkDome()
{
    int nErr = DDW_OK;
    
#if defined DDW_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CddwDome::unparkDome] ***********************\n", timestamp);
	fflush(Logfile);
#endif

	if(m_bDomeIsMoving || m_bShutterIsMoving) {
#if defined DDW_DEBUG && DDW_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::unparkDome] Movement in progress m_bDomeIsMoving = %s, m_bShutterIsMoving = %s\n", timestamp, m_bDomeIsMoving?"True":"False", m_bShutterIsMoving?"True":"False");
        fflush(Logfile);
#endif
        return ERR_COMMANDINPROGRESS;
    }

    m_bParked = false;
    nErr = goHome();
    return nErr;
}


int CddwDome::calibrate()
{

    int nErr = DDW_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined DDW_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CddwDome::calibrate] ***********************\n", timestamp);
	fflush(Logfile);
#endif

	if(m_bDomeIsMoving || m_bShutterIsMoving) {
#if defined DDW_DEBUG && DDW_DEBUG >= 2
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::calibrate] Movement in progress m_bDomeIsMoving = %s, m_bShutterIsMoving = %s\n", timestamp, m_bDomeIsMoving?"True":"False", m_bShutterIsMoving?"True":"False");
		fflush(Logfile);
#endif
		return ERR_COMMANDINPROGRESS;
	}


	m_bDomeIsMoving = false;

    nErr = domeCommand("GTRN", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

	if(strlen(szResp)) {
		switch(szResp[0]) {
			case 'L':
			case 'R':
			case 'T':
			case 'P':
				nErr = DDW_OK;
				m_bDomeIsMoving = true;
				break;
			default :
				nErr = DDW_BAD_CMD_RESPONSE;
				break;
		}
	}
    dataReceivedTimer.Reset();
    return nErr;
}

int CddwDome::abortCurrentCommand()
{
    int nErr;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
    m_bDomeIsMoving = false;
    m_bShutterIsMoving = false;
    
#if defined DDW_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::abortCurrentCommand] ***********************\n", timestamp);
    fflush(Logfile);
#endif
    
    nErr = domeCommand("STOP\n", NULL, SERIAL_BUFFER_SIZE, 250);
    
    return nErr;
}

int CddwDome::syncDome(double dAz, double dEl)
{
    return ERR_COMMANDNOTSUPPORTED;
}

#pragma mark - End of movement checks

bool CddwDome::isDomeMoving()
{
    int nErr = DDW_OK;
    int nConvErr = DDW_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    std::vector<std::string> vFieldsData;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
#if defined DDW_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isDomeMoving] ***********************\n", timestamp);
    fflush(Logfile);
#endif
    
    if(!m_bDomeIsMoving) {
#if defined DDW_DEBUG && DDW_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::isDomeMoving] isMoving = %s, there was no movement initiated\n", timestamp, m_bDomeIsMoving?"True":"False");
        fflush(Logfile);
#endif
        return m_bDomeIsMoving;
    }
    
    // read as much as we can.
    nErr = readAllResponses(szResp, SERIAL_BUFFER_SIZE);
    
#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isDomeMoving] resp = %s\n", timestamp, szResp);
    fflush(Logfile);
#endif
    
    if(nErr) {
        if(nErr == DDW_TIMEOUT) {
            if(strlen(szResp)) {
                // is there a partial INF response in there.
                if(szResp[0] == 'V') {
                    m_bDomeIsMoving = false;
#if defined DDW_DEBUG && DDW_DEBUG >= 2
                    ltime = time(NULL);
                    timestamp = asctime(localtime(&ltime));
                    timestamp[strlen(timestamp) - 1] = 0;
                    fprintf(Logfile, "[%s] [CddwDome::isDomeMoving] [DDW_TIMEOUT] resp starts with 'V', we're done moving\n", timestamp);
                    fflush(Logfile);
#endif
                }
                else {
#if defined DDW_DEBUG && DDW_DEBUG >= 2
                    ltime = time(NULL);
                    timestamp = asctime(localtime(&ltime));
                    timestamp[strlen(timestamp) - 1] = 0;
                    fprintf(Logfile, "[%s] [CddwDome::isDomeMoving] [DDW_TIMEOUT] resp doesn't starts with 'V', still moving ?\n", timestamp);
                    fflush(Logfile);
#endif
                    m_bDomeIsMoving = true; // we're probably still moving but haven't got  L,R,T,C,O,S or Pxxx since last time we checked
                }
            }
            
            if((dataReceivedTimer.GetElapsedSeconds() >= 30.0f) && m_bDomeIsMoving) {
#if defined DDW_DEBUG && DDW_DEBUG >= 2
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CddwDome::isDomeMoving] [DDW_TIMEOUT] dataReceivedTimer.GetElapsedSeconds() = %3.2f\n", timestamp, dataReceivedTimer.GetElapsedSeconds());
                fflush(Logfile);
#endif
                // we might have missed the GINV response, send a GINV
                m_bDomeIsMoving = false;
                nErr = getInfRecord();
            }
        }
        else {
#if defined DDW_DEBUG && DDW_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CddwDome::isDomeMoving] [DDW_TIMEOUT] no response from dome, let's assume it stopped ?\n", timestamp);
            fflush(Logfile);
#endif
            m_bDomeIsMoving = false;   // there was an actuel error ?
        }
    }
    else if(strlen(szResp)) {  // no error, let's look at the response
        switch(szResp[0]) {
            case 'V':    // getting INF = we're done with the current opperation
#if defined DDW_DEBUG && DDW_DEBUG >= 2
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CddwDome::isDomeMoving] resp[0] is 'V', we're done moving\n", timestamp);
                fflush(Logfile);
#endif
                m_bDomeIsMoving = false;
                nErr = getInfRecord();
                dataReceivedTimer.Reset();
                break;
            case 'L':    // moving Left
            case 'R':    // moving Right
            case 'T':    // Az Tick
            case 'S':    // Manual ops
                m_bDomeIsMoving  = true;
                dataReceivedTimer.Reset();
#if defined DDW_DEBUG && DDW_DEBUG >= 2
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CddwDome::isDomeMoving] resp[0] is in [L,R,T,S], we're still moving\n", timestamp);
                fflush(Logfile);
#endif
                break;
            case 'P':    // moving and reporting position
#if defined DDW_DEBUG && DDW_DEBUG >= 2
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CddwDome::isDomeMoving] resp[0] is 'P' we're still moving and updating position\n", timestamp);
                fflush(Logfile);
#endif
                m_bDomeIsMoving  = true;
                nConvErr = parseFields(szResp, vFieldsData, 'P');
                if(!nConvErr && m_nNbStepPerRev && vFieldsData.size()) {
                    try {
                        m_dCurrentAzPosition = (360.0/m_nNbStepPerRev) * std::stof(vFieldsData[0]);
                    } catch(const std::exception& e) {
#if defined DDW_DEBUG
                        ltime = time(NULL);
                        timestamp = asctime(localtime(&ltime));
                        timestamp[strlen(timestamp) - 1] = 0;
                        fprintf(Logfile, "[%s] [CddwDome::isDomeMoving] std::stof exception : %s\n", timestamp, e.what());
                        fflush(Logfile);
#endif
                        return ERR_DATAOUT;
                    }
                }
                dataReceivedTimer.Reset();
                break;
            default :    // shouldn't happen !
                m_bDomeIsMoving  = false;
                break;
        }
    }
    
#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isDomeMoving] isMoving = %s\n", timestamp, m_bDomeIsMoving?"True":"False");
    fflush(Logfile);
#endif
    
    return m_bDomeIsMoving;
}

bool CddwDome::isShutterMoving()
{
    int nErr = DDW_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    std::vector<std::string> vFieldsData;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
#if defined DDW_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isShutterMoving] ***********************\n", timestamp);
    fflush(Logfile);
#endif
    
    if(!m_bShutterIsMoving) {
#if defined DDW_DEBUG && DDW_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::isShutterMoving] m_bShutterIsMoving = %s, there was no movement initiated\n", timestamp, m_bShutterIsMoving?"True":"False");
        fflush(Logfile);
#endif
        return m_bShutterIsMoving;
    }
    
    // read as much as we can.
    nErr = readAllResponses(szResp, SERIAL_BUFFER_SIZE);
    
#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isShutterMoving] resp = '%s'\n", timestamp, szResp);
    fflush(Logfile);
#endif

    if(nErr) {
        if(nErr == DDW_TIMEOUT) {
            if(strlen(szResp)) {
                // is there a partial INF response in there.
                if(szResp[0] == 'V') {
                    m_bShutterIsMoving = false;
                }
                else {
                    m_bShutterIsMoving = true; // we're probably still moving but haven't got C,O,S or V since last time we checked
                }
            }
            if(dataReceivedTimer.GetElapsedSeconds() > 30.0f && m_bShutterIsMoving) {
                // we might have missed the GINV response, send a GINV
                m_bShutterIsMoving = false;
                nErr = getInfRecord();
            }
        }
        else
            m_bShutterIsMoving = false;   // there was an actuel error ?
    }
    else if(strlen(szResp)) {  // no error, let's look at the response
        switch(szResp[0]) {
            case 'V':    // getting INF = we're done with the current opperation
                parseGINF(szResp);
                try {
                    if(std::stoi(m_svGinf[gShutter]) == UNKNOWN) // are we still opening but we got an INF record
                        m_bShutterIsMoving = true;
                    else
                        m_bShutterIsMoving = false;
                } catch(const std::exception& e) {
#if defined DDW_DEBUG
                    ltime = time(NULL);
                    timestamp = asctime(localtime(&ltime));
                    timestamp[strlen(timestamp) - 1] = 0;
                    fprintf(Logfile, "[%s] [CddwDome::isShutterMoving] std::stoi exception : %s\n", timestamp, e.what());
                    fflush(Logfile);
#endif
                    return m_bShutterIsMoving;
                }

                dataReceivedTimer.Reset();
                break;
            case 'C':    // Closing shutter
            case 'O':    // Opening shutter
            case 'S':    // Manual ops
                m_bShutterIsMoving  = true;
                dataReceivedTimer.Reset();
                break;
            default :    // shouldn't happen !
                m_bShutterIsMoving  = false;
                break;
        }
    }
    
#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isShutterMoving]m_bShutterIsMoving = %s\n", timestamp, m_bShutterIsMoving?"True":"False");
    if(nErr) {
        fprintf(Logfile, "[%s] [CddwDome::isShutterMoving] nErr = %s\n", timestamp, nErr == DDW_TIMEOUT?"DDW_TIMEOUT":"Other");
    }
    fflush(Logfile);
#endif
    
    return m_bShutterIsMoving;
}


bool CddwDome::isDomeAtHome()
{
    int nErr = DDW_OK;
    
    bool  bHomed = false;
    
    if(!m_bIsConnected)
        return bHomed;
    
#if defined DDW_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isDomeAtHome] ***********************\n", timestamp);
    fflush(Logfile);
#endif
    
    nErr = getInfRecord();
    if(nErr)
        return bHomed;
    
    try {
        if(std::stoi(m_svGinf[gHome]) == AT_HOME) {
            bHomed  = true;
            m_bDomeIsMoving = false;
        }
    } catch(const std::exception& e) {
#if defined DDW_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::isDomeAtHome] std::stof exception : %s\n", timestamp, e.what());
        fflush(Logfile);
#endif
        return bHomed;
    }

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isDomeAtHome] bHomed = %s\n", timestamp, bHomed?"True":"False");
    fflush(Logfile);
#endif
    
    return bHomed;
}



int CddwDome::isGoToComplete(bool &bComplete)
{
    int nErr = DDW_OK;
    double dDomeAz = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined DDW_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CddwDome::isGoToComplete] ***********************\n", timestamp);
	fflush(Logfile);
#endif

	bComplete = false;

    if(!m_bDomeIsMoving && !m_bShutterIsMoving) { // case of a goto to current position.
        bComplete = true;
        nErr = getDomeAz(dDomeAz);
#if defined DDW_DEBUG && DDW_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::isGoToComplete] dDomeAz = %3.2f, m_bDomeIsMoving = %s, m_bShutterIsMoving = %s, bComplete = %s\n", timestamp, dDomeAz, m_bDomeIsMoving?"True":"False", m_bShutterIsMoving?"True":"False", bComplete?"True":"False");
        fflush(Logfile);
#endif
        return nErr;
    }

    if(isDomeMoving()) {
        return nErr;
    }

    nErr = getDomeAz(dDomeAz);
    if(nErr)
        return nErr;

    if ((ceil(m_dGotoAz) <= (ceil(dDomeAz) + m_dCoastDeg) ) && (ceil(m_dGotoAz) >= (ceil(dDomeAz) - m_dCoastDeg) )) {
        bComplete = true;
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

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined DDW_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CddwDome::isOpenComplete] ***********************\n", timestamp);
	fflush(Logfile);
#endif


    bComplete = false;

    if(!m_bShutterIsMoving) { // case of an open when it's already openned
        bComplete = true;
#if defined DDW_DEBUG && DDW_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::isOpenComplete] m_bDomeIsMoving = %s, m_bShutterIsMoving = %s, bComplete = %s\n", timestamp, m_bDomeIsMoving?"True":"False", m_bShutterIsMoving?"True":"False", bComplete?"True":"False");
        fflush(Logfile);
#endif
        return nErr;
    }
    
    if(isShutterMoving()) {
        return nErr;
    }

    bComplete = true;

    nErr = getShutterState();
    if(!nErr) {
        if(m_bShutterOpened){
            m_dCurrentElPosition = 90.0;
            nErr =  ERR_CMDFAILED;  // we're done opening and yet it's not open !
        }
        else {
            m_dCurrentElPosition = 0.0;
        }
    }
#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isOpenComplete] bComplete = %s, nErr = %d\n", timestamp, bComplete?"True":"False", nErr);
    fflush(Logfile);
#endif

    return nErr;
}

int CddwDome::isCloseComplete(bool &bComplete)
{
    int nErr = DDW_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined DDW_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CddwDome::isCloseComplete] ***********************\n", timestamp);
	fflush(Logfile);
#endif

    bComplete = false;

    if(!m_bShutterIsMoving) { // case of an close when it's already closed
        bComplete = true;
#if defined DDW_DEBUG && DDW_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CddwDome::isCloseComplete] m_bDomeIsMoving = %s, m_bShutterIsMoving = %s, bComplete = %s\n", timestamp, m_bDomeIsMoving?"True":"False", m_bShutterIsMoving?"True":"False", bComplete?"True":"False");
        fflush(Logfile);
#endif
        return nErr;
    }

    if(isShutterMoving()) {
        return nErr;
    }
    // Shutter is not not moving anymore. so the close is done and now we check for errors.
    bComplete = true;

    nErr = getShutterState();
    if(!nErr) {
        if(!m_bShutterOpened){
            m_dCurrentElPosition = 0.0;
        }
        else {
            m_dCurrentElPosition = 90.0;
            nErr = ERR_CMDFAILED; // we're done closing and yet it's not closed !
        }
    }
#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isCloseComplete] bComplete = %s, nErr = %d\n", timestamp, bComplete?"True":"False", nErr);
    fflush(Logfile);
#endif

    return nErr;
}


int CddwDome::isParkComplete(bool &bComplete)
{
    int nErr = DDW_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined DDW_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isParkComplete] ***********************\n", timestamp);
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

#if defined DDW_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isUnparkComplete] ***********************\n", timestamp);
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

#if defined DDW_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isFindHomeComplete] ***********************\n", timestamp);
    fflush(Logfile);
#endif

    if(isDomeMoving()) {
        bComplete = false;
        return nErr;
    }

    if(isDomeAtHome()){
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

#if defined DDW_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CddwDome::isCalibratingComplete] ***********************\n", timestamp);
	fflush(Logfile);
#endif

	if(isDomeMoving()) {
        bComplete = false;
        return nErr;
    }


    nErr = getDomeAz(dDomeAz);
    if(nErr)
        return nErr;
    if (ceil(m_dHomeAz) != ceil(dDomeAz)) {
        // We need to resync the current position to the home position.
        m_dCurrentAzPosition = m_dHomeAz;
        bComplete = true;
    }

    nErr = getDomeStepPerRev();
    getCoast();
    getDeadZone();
    bComplete = true;
    m_bDomeIsMoving = false;

#if defined DDW_DEBUG && DDW_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CddwDome::isCalibratingComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
    fflush(Logfile);
#endif
    return nErr;
}



#pragma mark - Public Getters

int CddwDome::getNbTicksPerRev()
{
    int nErr = DDW_OK;
    if(m_bIsConnected)
        nErr = getDomeStepPerRev();
    return m_nNbStepPerRev;
}


double CddwDome::getHomeAz()
{
    if(m_bIsConnected)
        getDomeHomeAz();

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
        getShutterState();

    
    return m_nShutterState;
}


#pragma mark - Helper methods

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
		if(sSegment.size())
        	svFields.push_back(sSegment);
    }

    if(svFields.size()==0) {
        nErr = ERR_BADFORMAT;
    }
    return nErr;
}


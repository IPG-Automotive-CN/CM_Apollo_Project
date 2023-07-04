/*
******************************************************************************
**  CarMaker - Version 11.0
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
**
** Raw Signal Data Stream example client for IPGMovie 8.0.
**
** This example looks quite complex at first but is actually quite simple.
** - establish the RSDS connection: RSDS_Connect
** - get the RSDS data: RSDS_RecvHdr and RSDS_GetData
** everything else has to do with saving the data or actualising the statistics
**
** Have a look at rsds-client-camera-basics.c for a much simpler example.
**
** Compiling:
** Linux
**	gcc -Wall -Os -o rsds-client-camera-standalone rsds-client-camera-standalone.c
** MS Windows (MSYS/MinGW)
**	gcc -Wall -Os -o rsds-client-camera-standalone rsds-client-camera-standalone.c -lws2_32
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <signal.h>
#include <inttypes.h>
#include <unistd.h>

#ifdef WIN32
#  include <winsock2.h>
#else
#  include <sys/socket.h>
#  include <sys/types.h>
#  include <net/if.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <netdb.h>
#endif

typedef enum {
    SaveFormat_DataNotSaved = 0,
    SaveFormat_Raw,
    SaveFormat_PPM,
    SaveFormat_PGM_byte,
    SaveFormat_PGM_short,
    SaveFormat_PGM_float,
} tSaveFormat;

static struct {
    FILE *EmbeddedDataCollectionFile;
    char *MovieHost; /* pc on which IPGMovie runs          */
    int MoviePort; /* TCP/IP port for RSDS                */
    int sock; /* TCP/IP Socket                      */
    char sbuf[64]; /* Buffer for transmitted information */
    int RecvFlags; /* Receive Flags                      */
    int Verbose; /* Logging Output     			*/
    int ConnectionTries;
    tSaveFormat SaveFormat;
    int TerminationRequested;
} RSDScfg;

struct {
    double tFirstDataTime;
    double tStartSim;
    double tEndSim;
    double tLastSimTime;
    unsigned long long int nBytesTotal;
    unsigned long long int nBytesSim;
    unsigned long int nImagesTotal;
    unsigned long int nImagesSim;
    unsigned char nChannels;
} RSDSIF;

static void RSDS_PrintSimInfo();
// Helpers for RSDSIF : RSDS information ( stats about current status)
void RSDSIF_AddDataToStats(unsigned int len);
void RSDSIF_UpdateStats(unsigned int ImgLen, const char *ImgType, int Channel, int ImgWidth, int ImgHeight, float SimTime);
void RSDSIF_UpdateEndSimTime();
// misc helpers
void WriteImgDataToFile(const char* img, unsigned int ImgLen, const char *ImgType, int Channel, int ImgWidth, int ImgHeight, float SimTime);
void WriteEmbeddedDataToCSVFile(const char* data, unsigned int dataLen, int Channel, float SimTime, const char* AniMode);
void PrintEmbeddedData (const char* data, unsigned int dataLen);

/*
 ** RSDS_RecvHdr
 **
 ** Scan TCP/IP Socket and writes to buffer
 */
static int RSDS_RecvHdr(int sock, char *hdr)
{
    const int HdrSize = 64;
    int len = 0;
    int nSkipped = 0;
    int i;

    while (1) {
        if (RSDScfg.TerminationRequested)
            return -1;
        for (; len < HdrSize; len += i) {
            if ((i = recv(sock, hdr + len, HdrSize - len, RSDScfg.RecvFlags)) <= 0) {
            	if (!RSDScfg.TerminationRequested)
		    printf ("RSDS_RecvHdr Error during recv (received: '%s' (%d))\n", hdr, len);
                return -1;
	    }
        }
        if (hdr[0] == '*' && hdr[1] >= 'A' && hdr[1] <= 'Z') {
            /* remove white spaces at end of line */
            while (len > 0 && hdr[len - 1] <= ' ')
                len--;
            hdr[len] = 0;
            if (RSDScfg.Verbose == 1 && nSkipped > 0)
                printf("RSDS: HDR resync, %d bytes skipped\n", nSkipped);
            return 0;
        }
        for (i = 1; i < len && hdr[i] != '*'; i++)
            ;
        len -= i;
        nSkipped += i;
        memmove(hdr, hdr + i, len);
    }
}

/*
 ** RSDS_Connect
 **
 ** Connect over TCP/IP socket
 */
static int RSDS_Connect(void)
{
#ifdef WIN32
    WSADATA WSAdata;
    if (WSAStartup(MAKEWORD(2,2), &WSAdata) != 0) {
        fprintf (stderr, "RSDS: WSAStartup ((2,2),0) => %d\n", WSAGetLastError());
        return -1;
    }
#endif

    struct sockaddr_in DestAddr;
    struct hostent *he;
    int tries = RSDScfg.ConnectionTries;

    if ((he = gethostbyname(RSDScfg.MovieHost)) == NULL) {
        fprintf(stderr, "RSDS: unknown host: %s\n", RSDScfg.MovieHost);
        return -2;
    }
    DestAddr.sin_family = AF_INET;
    DestAddr.sin_port = htons((unsigned short) RSDScfg.MoviePort);
    DestAddr.sin_addr.s_addr = *(unsigned *) he->h_addr;
    RSDScfg.sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    while (connect(RSDScfg.sock, (struct sockaddr *) &DestAddr, sizeof(DestAddr)) < 0 && tries > 0) {
        fprintf(stderr, "RSDS: can't connect '%s:%d'\n", RSDScfg.MovieHost, RSDScfg.MoviePort);
        if (tries > 1) {
            fprintf(stderr, "\tretrying in 1 second... (%d)\n", --tries);
            fflush(stderr);
            sleep(1);
        } else {
            return -4;
        }
    }
    if (RSDS_RecvHdr(RSDScfg.sock, RSDScfg.sbuf) < 0)
        return -3;

    printf("RSDS: Connected: %s\n", RSDScfg.sbuf + 1);

    memset(RSDScfg.sbuf, 0, 64);

    return 0;
}

/*
 ** RSDS_GetData
 **
 ** data and image processing
 */
static int RSDS_GetData(void)
{
    unsigned int len = 0;
    int res = 0;

    /* Variables for Image Processing */
    char ImgType[32], AniMode[16];
    int ImgWidth, ImgHeight, Channel;
    float SimTime;
    unsigned int ImgLen, dataLen;

    if (sscanf(RSDScfg.sbuf, "*RSDS %d %s %f %dx%d %d", &Channel, ImgType, &SimTime, &ImgWidth, &ImgHeight, &ImgLen) == 6) {

        RSDSIF_UpdateStats(ImgLen, ImgType,Channel, ImgWidth, ImgHeight, SimTime);

        if (RSDScfg.Verbose == 1)
            printf("%-6.3f : %-2d : %-8s %dx%d %d\n", SimTime, Channel, ImgType, ImgWidth, ImgHeight, ImgLen);

        if (ImgLen > 0) {

            // this is how we get the data
            char *img = (char *) malloc(ImgLen);
            for (len = 0; len < ImgLen; len += res) {
                if ((res = recv(RSDScfg.sock, img + len, ImgLen - len, RSDScfg.RecvFlags)) < 0) {
                    printf("RSDS: Socket Reading Failure\n");
                    free(img);
                    break;
                }
            }

	    // save the data to disc
            WriteImgDataToFile(img, ImgLen, ImgType, Channel, ImgWidth, ImgHeight, SimTime);

            free(img);

            RSDSIF_AddDataToStats(len);
        }
        // needed for all channels, since we want the time until the last image
        RSDSIF_UpdateEndSimTime();
    } else if (sscanf(RSDScfg.sbuf, "*RSDSEmbeddedData %d %f %d %s", &Channel, &SimTime, &dataLen, AniMode) == 4) {

        if (RSDScfg.Verbose == 1)
            printf("Embedded Data: %d %f %d %s\n", Channel, SimTime, dataLen, AniMode);

        if (dataLen > 0) {
            char *data = (char *) malloc(dataLen);

            // get the data
            for (len = 0; len < dataLen; len += res) {
                if ((res = recv(RSDScfg.sock, data + len, dataLen - len, RSDScfg.RecvFlags)) < 0) {
                    printf("RSDS: Socket Reading Failure\n");
                    free(data);
                    break;
                }
            }

	    // save the data to disc
            WriteEmbeddedDataToCSVFile(data, dataLen, Channel, SimTime, AniMode);
            if (RSDScfg.Verbose == 1)
                PrintEmbeddedData(data, dataLen);

            free(data);
        }
    } else {
        printf("RSDS: not handled: %s\n", RSDScfg.sbuf);
    }

    return 0;
}

/*
 ** RSDS_Init
 **
 ** Initialize Data Struct
 */
void RSDS_Init(void)
{
    RSDScfg.MovieHost = "localhost";
    RSDScfg.MoviePort = 2210;
    RSDScfg.Verbose = 0;
    RSDScfg.SaveFormat = SaveFormat_DataNotSaved;
    RSDScfg.EmbeddedDataCollectionFile = NULL;
    RSDScfg.RecvFlags = 0;
    RSDScfg.ConnectionTries = 5;
    RSDScfg.TerminationRequested = 0;

    RSDSIF.tFirstDataTime = 0.0;
    RSDSIF.tStartSim = 0.0;
    RSDSIF.tEndSim = 0.0;
    RSDSIF.tLastSimTime = -1.0;
    RSDSIF.nImagesSim = 0;
    RSDSIF.nImagesTotal = 0;
    RSDSIF.nBytesTotal = 0;
    RSDSIF.nBytesSim = 0;
    RSDSIF.nChannels = 0;
}


static void RSDS_PrintSimInfo()
{
    double dtSimReal = RSDSIF.tEndSim - RSDSIF.tStartSim;
    // at least 1 sec of data is required
    if (dtSimReal > 1.0) {
        printf("\nLast Simulation------------------\n");
        double MiBytes = RSDSIF.nBytesSim / (1024.0 * 1024.0);
        printf("Duration: %.3f (real) %.3f (sim) -> x%.2f\n", dtSimReal, RSDSIF.tLastSimTime, RSDSIF.tLastSimTime / dtSimReal);
        printf("Channels: %d\n", RSDSIF.nChannels);
        printf("Images:   %ld (%.3f FPS)\n", RSDSIF.nImagesSim, RSDSIF.nImagesSim / dtSimReal);
        printf("Bytes:    %.3f MiB (%.3f MiB/s)\n\n", MiBytes, MiBytes / dtSimReal);
    }
    if (RSDScfg.EmbeddedDataCollectionFile != NULL)
        fflush(RSDScfg.EmbeddedDataCollectionFile);

}

static void RSDS_PrintClosingInfo()
{
    // from the very first image to the very last
    double dtSession = RSDSIF.tEndSim - RSDSIF.tFirstDataTime;
    printf("\n-> Closing RSDS-Client...\n");

    // at least 1 sec of data is required
    if (dtSession > 1.0) {
        RSDS_PrintSimInfo();
        printf("Session--------------------------\n");
        double MiBytes = RSDSIF.nBytesTotal / (1024.0 * 1024.0);
        printf("Duration: %g seconds\n", dtSession);
        printf("Images:   %ld (%.3f FPS)\n", RSDSIF.nImagesTotal, RSDSIF.nImagesTotal / dtSession);
        printf("Bytes:    %.3f MiB (%.3f MiB per second)\n", MiBytes, MiBytes / dtSession);
    }
    fflush(stdout);

    if (RSDScfg.EmbeddedDataCollectionFile != NULL)
        fclose(RSDScfg.EmbeddedDataCollectionFile);
}

// on a system with properly configured timers, calling this function should need less then 0.1us
static inline double GetTime()  // in seconds
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}

void RSDSIF_AddDataToStats(unsigned int len)
{
    RSDSIF.nImagesTotal++;
    RSDSIF.nBytesTotal += len;
    RSDSIF.nImagesSim++;
    RSDSIF.nBytesSim += len;
}

void RSDSIF_UpdateStats(unsigned int ImgLen, const char *ImgType, int Channel, int ImgWidth, int ImgHeight, float SimTime)
{
    if (RSDSIF.tFirstDataTime == 0.0)
        RSDSIF.tFirstDataTime = GetTime();

    if (SimTime < 0.005 || RSDSIF.tLastSimTime < 0) {
        if (Channel == 0) {
            if (RSDSIF.tLastSimTime > 0)
                RSDS_PrintSimInfo();
            printf("-> Simulation started... (@ %.3f)\n", SimTime);
            RSDSIF.tStartSim = GetTime();
            RSDSIF.nBytesSim = 0;
            RSDSIF.nImagesSim = 0;
            RSDSIF.nChannels = 1;
        }
        // this text will appear only for the first img of each channel
        if (RSDScfg.Verbose == 2)
            printf("%-6.3f : %-2d : %-8s %dx%d %d\n", SimTime, Channel, ImgType, ImgWidth, ImgHeight, ImgLen);
    }
    if (Channel == 0)
        RSDSIF.tLastSimTime = SimTime;

    if (Channel >= RSDSIF.nChannels)
        RSDSIF.nChannels = Channel + 1;
}

void RSDSIF_UpdateEndSimTime()
{
    RSDSIF.tEndSim = GetTime();
}

void WriteEmbeddedDataToCSVFile(const char* data, unsigned int dataLen, int Channel, float SimTime, const char* AniMode)
{
    if (RSDScfg.EmbeddedDataCollectionFile != NULL) {
        double * buf = (double *)data;
        unsigned int len =  dataLen/sizeof(double), i;

        fprintf(RSDScfg.EmbeddedDataCollectionFile, "%d,%f,%s", Channel, SimTime, AniMode);
        for (i = 0; i < len; i++ ) {
            fprintf(RSDScfg.EmbeddedDataCollectionFile, ",%f", buf[i]);
        }
        fprintf(RSDScfg.EmbeddedDataCollectionFile, "\n");
    }
}

void PrintEmbeddedData (const char* data, unsigned int dataLen)
{
    double * buf = (double *)data;
    unsigned int len =  dataLen/sizeof(double), i;
    for (i = 0; i < len; i++ ) {
        printf("(%d) %f ", i, buf[i]);
    }
    printf("\n");
}

void WriteImgDataToFile(const char* img, unsigned int ImgLen, const char *ImgType, int Channel, int ImgWidth, int ImgHeight, float SimTime)
{
    // Save the data
    char fname[64];
    char header[64];

    fname[0] = '\0';
    switch (RSDScfg.SaveFormat) {
        case SaveFormat_PGM_short: {
            sprintf(fname, "./rsds%d-%s-%0.2f.pgm", Channel, ImgType, SimTime);
            sprintf(header, "P5\n%d %d\n65535\n", ImgWidth, ImgHeight);
            break;
        }
        case SaveFormat_PGM_byte: {
            sprintf(fname, "./rsds%d-%s-%0.2f.pgm", Channel, ImgType, SimTime);
            sprintf(header, "P5\n%d %d\n255\n", ImgWidth, ImgHeight);
            break;
        }
        case SaveFormat_PGM_float: {
            sprintf(fname, "./rsds%d-%s-%0.2f.pgm", Channel, ImgType, SimTime);
            sprintf(header, "P5\n%d %d\nfloat\n", ImgWidth, ImgHeight);
            break;
        }
        case SaveFormat_PPM: {
            sprintf(fname, "./rsds%d-%s-%0.2f.ppm", Channel, ImgType, SimTime);
            sprintf(header, "P6\n%d %d\n255\n", ImgWidth, ImgHeight);
            break;
        }
        case SaveFormat_Raw: {
            sprintf(fname, "./rsds%d-%s-%0.2f.raw", Channel, ImgType, SimTime);
            break;
        }
        default: {
            // do not save the data
            break;
        }
    }
    if (fname[0] != '\0') {
        FILE *fp = fopen(fname, "wb");

        if (fp == NULL) {
            printf ("ERROR: could not write '%s'\n", fname);
            return;
        }

        // header
        if (RSDScfg.SaveFormat != SaveFormat_Raw)
            fwrite(header, strlen(header), 1, fp);

        fwrite(img, ImgLen, 1, fp);
        fclose(fp);
    }

}

void termination_handler(int signum)
{
    RSDScfg.TerminationRequested = 1;
    RSDS_PrintClosingInfo();
#ifdef WIN32
    WSACleanup();
#endif
    exit(0);
}

int main(int argc, char * const *argv)
{
    int i;
    char *appname = argv[0];

    RSDS_Init();

    // get the options
    while (*++argv != NULL) {
        if (strcmp(*argv, "-v") == 0) {
            RSDScfg.Verbose = 1;
        } else if (strcmp(*argv, "-V") == 0) {
            RSDScfg.Verbose = 2;
        } else if (strcmp(*argv, "-h") == 0) {
            printf("%s - IPG Automotive GmbH", appname);
            printf(" %s ?<option>? ?<host>?\n", appname);
            printf(" * options *\n");
            printf("    -t <n>  tries n times to connect to IPGMovie (default = 5)\n");
            printf("    -v		fully verbose output\n");
            printf("    -V		less verbose output\n");
            printf("    -p <port>	set RSDS port to listen to (default = 2210)\n");
            printf("    -s <format>	save picture data to:\n");
            printf("    	raw	.raw (as is, no header)\n");
            printf("    	g8	.pgm (greyscale) 8bpp\n");
            printf("    	g16	.pgm (greyscale) 16bpp\n");
            printf("    	ppm	.ppm (rgb) 24bpp\n");
            printf("    -e <file>	collect embedded data in CSV file\n");
            printf(" * host *\n");
            printf("    default = localhost\n");
	    fflush(stdout);
            goto end;
        } else if (strcmp(*argv, "-p") == 0 && argv[1]) {
            RSDScfg.MoviePort = atoi(*++argv);
        } else if (strcmp(*argv, "-t") == 0 && argv[1]) {
            RSDScfg.ConnectionTries = atoi(*++argv);
        } else if (strcmp(*argv, "-s") == 0 && argv[1]) {
            ++argv;
            if (strcmp(*argv, "raw") == 0)
                RSDScfg.SaveFormat = SaveFormat_Raw;
            else if (strcmp(*argv, "g8") == 0)
                RSDScfg.SaveFormat = SaveFormat_PGM_byte;
            else if (strcmp(*argv, "g16") == 0)
                RSDScfg.SaveFormat = SaveFormat_PGM_short;
            else if (strcmp(*argv, "gf") == 0)
                RSDScfg.SaveFormat = SaveFormat_PGM_float;
            else if (strcmp(*argv, "ppm") == 0)
                RSDScfg.SaveFormat = SaveFormat_PPM;
        } else if (strcmp(*argv, "-e") == 0 && argv[1]) {
            // all embedded data vectors are stored in *one* file
            FILE *fp = fopen(*++argv, "w");
            if (fp != NULL)
                RSDScfg.EmbeddedDataCollectionFile = fp;

        } else if (**argv != '-') {
            RSDScfg.MovieHost = *argv;
        }
    }

    // handle Ctrl-C
    if (signal(SIGINT, termination_handler) == SIG_IGN)
        signal(SIGINT, SIG_IGN);

    /* Connect to RSDS Server */
    if ((i = RSDS_Connect()) != 0) {
        fprintf(stderr, "Can't initialise RSDS Client (returns %d, %s)\n", i, i == -4 ? "No server": strerror(errno));
    }

    /* Read from TCP/IP-Port and process the image */
    while (RSDS_RecvHdr(RSDScfg.sock, RSDScfg.sbuf) == 0) {
    	if (RSDScfg.TerminationRequested)
    	    break;
        RSDS_GetData();
        fflush(stdout);
    }

    RSDS_PrintClosingInfo();

end:
#ifdef WIN32
    closesocket (RSDScfg.sock);
    WSACleanup();
#else
    close(RSDScfg.sock);
#endif

    return 0;
}

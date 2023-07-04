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
** Raw Signal Data Stream example client for IPGMovie 3.4 and later versions.
**
** This file was the standard example until CM8.0
**
** Compiling:
** Linux
**	gcc -Wall -Os -o rsds-client-camera-basics rsds-client-camera-basics.c
** MS Windows (MSYS/MinGW)
**	gcc -Wall -Os -o rsds-client-camera-basics rsds-client-camera-basics.c -lws2_32
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#ifdef WIN32
#  include <winsock2.h>
#else
#  include <unistd.h>
#  include <sys/socket.h>
#  include <sys/types.h>
#  include <net/if.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <netdb.h>
#endif


static struct {
    char *MovieHost;  /* pc on which IPGMovie runs          */
    int   MoviePort;  /* TCP/IP port for RSDS                */
    int   sock;       /* TCP/IP Socket                      */
    char  sbuf[64];   /* Buffer for transmitted information */
    int   RecvFlags;  /* Receive Flags                      */
    int   Verbose;    /* Logging Output                     */
    int   ConnectionTries;
} RSDScfg;

struct {
    int         nImages;
    int         nBytes;
    float       MinDepth;
} RSDSIF;



/*
** RSDS_RecvHdr
**
** Scan TCP/IP Socket and writes to buffer
*/
static int
RSDS_RecvHdr (int sock, char *hdr)
{
    const int HdrSize = 64;
    int len = 0;
    int nSkipped = 0;
    int i;

    while (1) {
        for ( ; len < HdrSize; len += i) {
	    if ((i=recv(sock, hdr+len, HdrSize-len, RSDScfg.RecvFlags)) <= 0)
		return -1;
	}
	if (hdr[0]=='*' && hdr[1]>='A' && hdr[1]<='Z') {
	    /* remove white spaces at end of line */
	    while (len>0 && hdr[len-1]<=' ')
		len--;
	    hdr[len] = 0;
	    if (RSDScfg.Verbose && nSkipped>0)
	        printf ("RSDS: HDR resync, %d bytes skipped\n", nSkipped);
	    return 0;
	}
	for (i=1; i<len && hdr[i]!='*'; i++) ;
	len -= i;
	nSkipped += i;
	memmove (hdr, hdr+i, len);
    }
}



/*
** RSDS_Connect
**
** Connect over TCP/IP socket
*/
static int
RSDS_Connect (void)
{
#ifdef WIN32
    WSADATA WSAdata;
    if (WSAStartup(MAKEWORD(2,0), &WSAdata) != 0) {
        fprintf (stderr, "RSDS: WSAStartup ((2,0),0) => %d\n", WSAGetLastError());
        return -1;
    }
#endif

    struct sockaddr_in	DestAddr;
    struct hostent  	*he;

    int tries = RSDScfg.ConnectionTries;

    if ((he=gethostbyname(RSDScfg.MovieHost)) == NULL) {
        fprintf (stderr, "RSDS: unknown host: %s\n", RSDScfg.MovieHost);
        return -2;
    }
    DestAddr.sin_family      = AF_INET;
    DestAddr.sin_port        = htons((unsigned short)RSDScfg.MoviePort);
    DestAddr.sin_addr.s_addr = *(unsigned *)he->h_addr;
    RSDScfg.sock = socket(AF_INET, SOCK_STREAM, 0);

    while (connect(RSDScfg.sock, (struct sockaddr *) &DestAddr, sizeof(DestAddr)) < 0 && tries > 0){
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

    printf ("RSDS: Connected: %s\n", RSDScfg.sbuf+1);

    return 0;
}



/*
** RSDS_GetData
**
** data and image processing
*/
static int
RSDS_GetData(void)
{
    int len = 0;
    int res = 0;

    /* Variables for Image Processing */
    char    ImgType[64];
    int     ImgWidth, ImgHeight, Channel;
    float   SimTime;
    int     ImgLen;
    int     nPixel;
    int     Pixel;
    int     MinDepthPixel;
    float  *f_img;

    if (sscanf(RSDScfg.sbuf, "*RSDS %d %s %f %dx%d %d", &Channel,
               ImgType, &SimTime, &ImgWidth, &ImgHeight, &ImgLen) == 6) {
        if (0)
            printf ("%6.3f %d: %8s %dx%d %d\n", SimTime, Channel, ImgType, ImgWidth, ImgHeight, ImgLen);
        if (ImgLen > 0) {
            char *img = (char *)malloc(ImgLen);
            for (len=0; len<ImgLen; len+=res) {
                if ((res=recv(RSDScfg.sock, img+len, ImgLen-len, RSDScfg.RecvFlags)) < 0) {
                    printf("RSDS: Socket Reading Failure\n");
                    break;
                }
            }
	    /* This example only works with the export format "depth"
	       To use for other formats, please refer to the manual of IPGMovie,
	       where the byte format of each export format is explained.
	       For instance, for the export formt "rgb" you would be dealing
	       with:
	       		char *rgb = img;
	       where rgb[pixelNr+0] is the red component of the current pixel,
	       	     rgb[pixelNr+1] is the green component,
	       	     rgb[pixelNr+2] is the blue component,
	       and 0 <= pixelNr < (width*height), linewise, from left to right.
	    */

            if (len == ImgLen && strcmp(ImgType, "depth") == 0) {
                /* process your image here
                ** This part is just a Demo!!! ###############################
                */

                /* Starting search parameter */
                MinDepthPixel = 0;

                /* Find the pixel with the smallest distance to any object.
                ** Every pixel has a length of 4 byte float.
                */
                nPixel = ImgLen/sizeof(float);
                f_img = (float *)img;

                for (Pixel = 0; Pixel < nPixel; Pixel++) {
                    if(f_img[Pixel] < f_img[MinDepthPixel])
                        MinDepthPixel = Pixel;
                }
                RSDSIF.MinDepth = f_img[MinDepthPixel];

                if (RSDScfg.Verbose) {
                    /* Print general image information */
                    printf ("> %s\n", RSDScfg.sbuf);

                    /* Print minimal distance and position of the pixel
                    ** x-position from left to right
                    ** y-position from top down
                    */
                    printf ("Minimal distance: %6.3f m\n", RSDSIF.MinDepth);
                    printf ("Pixel position: x = %u, y = %u\n", MinDepthPixel%ImgWidth, MinDepthPixel/ImgWidth);
                    printf ("\n");
                }
                /* Demo End!!! ################################################ */
            }
            free (img);
        }
        RSDSIF.nImages++;
        RSDSIF.nBytes += len;
    } else {
        printf ("RSDS: not handled: %s\n",RSDScfg.sbuf);
    }

    return 0;
}



/*
** RSDS_Init
**
** Initialize Data Struct
*/
void
RSDS_Init(void)
{
    RSDScfg.MovieHost = "localhost";
    RSDScfg.MoviePort = 2210;
    RSDScfg.Verbose   = 0;
    RSDScfg.RecvFlags = 0;
    RSDScfg.ConnectionTries = 5;

    RSDSIF.nImages  = 0;
    RSDSIF.nBytes   = 0;
    RSDSIF.MinDepth = 0;
}


int
main (int argc, char *const*argv)
{
    int i;
    char *appname = argv[0];

    RSDS_Init();

    while (*++argv != NULL) {
        if (strcmp(*argv,"-v") == 0) {
	    RSDScfg.Verbose++;
	} else if (strcmp(*argv,"-p")==0 && argv[1]) {
	    RSDScfg.MoviePort = atoi(*++argv);
	} else if (**argv!='-') {
	    RSDScfg.MovieHost = *argv;
	} else if (strcmp(*argv, "-t") == 0 && argv[1]) {
            RSDScfg.ConnectionTries = atoi(*++argv);
        } else if (strcmp(*argv, "-h") == 0) {
	    printf("%s - IPG Automotive GmbH", appname);
            printf(" %s ?<option>? ?<host>?\n", appname);
            printf(" * options *\n");
            printf("    -t <n>  tries n times to connect to IPGMovie (default = 5)\n");
            printf("    -v		fully verbose output\n");
            printf("    -p <port>	set RSDS port to listen to (default = 2210)\n");
	    printf(" * host *\n");
            printf("    default = localhost\n");
	    fflush(stdout);
            goto end;
	}
    }

    /* Connect to RSDS Server */
    if ((i = RSDS_Connect ()) != 0) {
        fprintf(stderr, "Can't initialise RSDS Client (returns %d, %s)\n",
                i, i == -4 ? "No server": strerror(errno));
    }

     /* Read from TCP/IP-Port and process the image */
     while (RSDS_RecvHdr(RSDScfg.sock,RSDScfg.sbuf) == 0) {
         RSDS_GetData();
     }

end:
    printf("Closing RSDS-Client...\n");
#ifdef WIN32
    closesocket (RSDScfg.sock);
#else
    close (RSDScfg.sock);
#endif

    return 0;
}

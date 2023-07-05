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
*/

#ifndef __RSDS_CLIENT_H__
#define __RSDS_CLIENT_H__

#ifdef __cplusplus
extern "C" {
#endif

static struct {
    char *MovieHost;  /* pc on which IPGMovie runs          */
    int   MoviePort;  /* TCP/IP port for RSDS                */
    int   sock;       /* TCP/IP Socket                      */
    char  sbuf[64];   /* Buffer for transmitted information */
    int   RecvFlags;  /* Receive Flags                      */
    int   Verbose;    /* Logging Output                     */
    
} RSDScfg;

struct {
    int         nImages;
    int         nBytes;
    float       MinDepth;
    char*        image;
    int         imgWidth;
    int         imgHeight;
    int         imgLen;

} RSDSIF;

void RSDS_Init  (void);
void RSDS_Start (void);
void RSDS_Exit  (void);


#ifdef __cplusplus
}
#endif

#endif /* __RSDS_CLIENT_H__ */

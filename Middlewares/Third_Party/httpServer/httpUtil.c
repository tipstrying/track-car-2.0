/**
 * @file	httpUtil.c
 * @brief	HTTP Server Utilities
 * @version 1.0
 * @date	2014/07/15
 * @par Revision
 *			2014/07/15 - 1.0 Release
 * @author
 * \n\n @par Copyright (C) 1998 - 2014 WIZnet. All rights reserved.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "httpUtil.h"
#include "types.h"
#include "socket.h"
#include "httpapi.h"
#include "listRunTask.h"

#ifdef _USE_FLASH_
#include "dataflash.h"
#endif
//#include "motor.h"
extern RunTaskDef runTaskHeader;

uint8_t http_get_cgi_handler(uint8_t *uri_name, uint8_t *buf, uint32_t *file_len)
{
    uint8_t ret = HTTP_OK;
    uint16_t len = 0;
    if (strcmp((const char *)uri_name, "status.cgi") == 0)
    {
        float sp, pos;
        GetSpeedHttpApi( &sp );
        GetPositionHttpApi( &pos );
        sprintf( (char *)buf, "{\"sp\":%0.2f, \"pos\":%0.2f}", sp, pos );
        len = strlen( (const char*)buf );
    }
    else if( strcmp( (const char *)uri_name, "getruntasklist.cgi" ) == 0 )
    {
        listConverToJson( &runTaskHeader, (char *)buf );
        len = strlen( (char *)buf );
    }
    else if( strcmp( (const char *)uri_name, "getSwitchStatus.cgi" ) == 0 )
    {
        sprintf( (char *)buf, "{\"status\":%d}", InOutStatus() );
        len = strlen( (char *)buf );
    }
    else if( strcmp( (const char *)uri_name, "position.cgi" ) == 0 )
    {
        float positon = 0;
        GetPositionHttpApi( &positon );
        sprintf( (char *)buf, "{\"position\":%0.2f}", positon );
        len = strlen( (char *)buf );
    }
    else if( strcmp( (const char*) uri_name, "taskstatus.cgi" ) == 0 )
    {
        getTaskStatus( buf );
        len = strlen( (const char*)buf );
    }
    else if( strcmp( (const char*) uri_name, "taskStack.cgi" ) == 0 )
    {
        getTaskStack( buf );
        len = strlen( (const char*)buf );
    }
    else if( strcmp( (const char*) uri_name, "ramFree.cgi" ) == 0 )
    {
        getRamStatus( buf );
        len = strlen( (const char*)buf );
    }
    else
    {
        // CGI file not found
        ret = HTTP_FAILED;
    }

    if (ret)
        *file_len = len;
    return ret;
}
void setOut(int i, int status);
uint8_t http_post_cgi_handler(uint8_t *uri_name, st_http_request *p_http_request, uint8_t *buf, uint32_t *file_len)
{
    uint8_t ret = HTTP_OK;
    uint16_t len = 0;
    uint8_t *device_ip;
    uint8_t val;

    if (strcmp((const char *)uri_name, "post.cgi") == 0)
    {
        uint8_t *posXNumber = get_http_param_value((char *)p_http_request->URI, "posX");
        len = 0;
    }
    else if( strcmp( (const char *)uri_name, "setpos.cgi" ) == 0 )
    {
        uint8_t *posXNumber = get_http_param_value((char *)p_http_request->URI, "position");
        float position;
        if( posXNumber )
        {
            position = ATOI( posXNumber, 10 );
            sprintf( (char *)buf, "Set Pos Ok" );
            if( SetPositionHttpApi( position ) )
            {
                sprintf( (char *)buf + strlen( (char *)buf ), "\tsend to motor ok\r\n" );
            }

            len = strlen( (char *)buf );
        }
        else
            len = 0;
    }
    else if( strcmp( (const char *)uri_name, "setop.cgi" ) == 0 )
    {
        float speed, position;
        int cmd;
        uint8_t *posNumber = get_http_param_value((char *)p_http_request->URI, "position");
        if( posNumber )
        {
            position = ATOI( posNumber, 10 );
        }
        uint8_t *cmdString = get_http_param_value((char *)p_http_request->URI, "cmd");
        if( cmdString )
            cmd = ATOI( cmdString, 10 );

        uint8_t *speedString = get_http_param_value((char *)p_http_request->URI, "speed");
        if( speedString )
            speed = ATOI( speedString, 10 );

        if( posNumber && cmdString && speedString )
        {
            if( speed )
            {
                SetOpHttpApi( position, 1, speed );
            }
            if( cmd > 1 )
            {
                cmd++;
                SetOpHttpApi(position, cmd, 0 );
            }
            sprintf( (char *)buf, "set op ok\r\n" );
            len = strlen( (char *)buf );
        }
        else
            len = 0;
    }
    else if( strcmp( (const char *)uri_name, "beltTest.cgi" ) == 0 )
    {
        uint8_t *cmdString = get_http_param_value((char *)p_http_request->URI, "cmd");
        if( cmdString )
            beltOpHttpApi( ATOI( cmdString, 10 ) );
        sprintf( (char *)buf, "set op ok\r\n" );
        len = strlen( (char *)buf );
    }
    else
    {
        // CGI file not found
        ret = HTTP_FAILED;
    }

    if (ret)
        *file_len = len;
    return ret;
}

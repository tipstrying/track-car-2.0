#include "listRunTask.h"
#include "string.h"
#include "cmsis_os.h"

#define itemSize  sizeof( RunTaskDef )
void listAddCallBack( RunTaskDef data );
void listDelCallBack( RunTaskDef data );

int listCreate( RunTaskDef * header )
{
    header = NULL;
    return pdTRUE;
}

void deleteList( RunTaskDef * header )
{
    if( !header )
        return ;

    RunTaskDef * next = header->next;
    header->next = NULL;
    header = next;
    while( header != NULL )
    {
        next = header->next;
        vPortFree( header );
        header = next;
    }
    header->next = 0;
}

int listAdd( RunTaskDef * header, RunTaskDef dataIn )
{
    RunTaskDef * next;
    next = header;
    /* 空链表 */
    if( !header )
    {
        return pdFALSE;
    }
    /* 非空链表，找到最后一个节点 */
    next = header;
    while( next->next != NULL )
        next = next->next;

    next->next = pvPortMalloc( itemSize );
    if( next->next )
    {
        *(next->next) = dataIn;
        next->next->next = NULL;
        listAddCallBack( dataIn );
        return pdTRUE;
    }
    else
    {
        return pdFALSE;
    }
}
int listGetItemByID( RunTaskDef * header, int ID, RunTaskDef * dataOut)
{
    if( header )
    {
        RunTaskDef * next = header->next;
        while( next )
        {
            if( next->ID == ID )
            {
                if( dataOut != NULL )
                    *dataOut = *next;
                return pdTRUE;
            }
            next = next->next;
        }
        return pdFALSE;
    }
    else
        return pdFALSE;
}

int listGetItemByCMD( RunTaskDef *header, int cmd, RunTaskDef *dataOut )
{
    if( header )
    {
        RunTaskDef * next = header->next;
        while( next )
        {
            if( next->cmd == cmd )
            {
                if( dataOut != NULL )
                    *dataOut = *next;
                return pdTRUE;
            }
            next = next->next;
        }
        return pdFALSE;
    }
    else
        return pdFALSE;
}
static int listItemcmp( RunTaskDef data1, RunTaskDef data2 )
{
    return pdFALSE;
}

int listUpdateItemByID( RunTaskDef *header, int ID, RunTaskDef dataIn )
{
    if( header )
    {
        RunTaskDef * next = header ->next;
        while( next )
        {
            if( next->ID == ID )
            {
                dataIn.next = next->next;
                *next = dataIn;
                return pdTRUE;
            }
            next = next->next;
        }
        if( listAdd( header, dataIn ) )
            return pdTRUE;
        else
            return pdFALSE;
    }
    else
        return pdFALSE;
}
int listDeleteItemByID( RunTaskDef *header, int ID )
{
    if( header )
    {
        RunTaskDef * next = header;
        RunTaskDef *tmp;
        while( next->next )
        {
            if( next->next->ID == ID )
            {
                tmp = next->next;
                listDelCallBack( *tmp);
                next->next = next->next->next;
                vPortFree( tmp );
                return pdTRUE;
            }
            next = next->next;
        }
        return pdTRUE;
    }
    else
        return pdFALSE;
}
int listDeleteItemByIndex( RunTaskDef *header, int Index )
{
    int index = 0;
    if( Index == 0 )
        return pdFALSE;
    if( header )
    {
        RunTaskDef * next = header;
        RunTaskDef *tmp;
        while( next->next )
        {
            index ++;
            if( index == Index )
            {
                tmp = next->next;
								if(tmp->next == NULL)
								{
										next->next = NULL;
								}
								else
								{
										next->next = next->next->next;
								}
                listDelCallBack( *tmp );
                //next->next = next->next->next;
                vPortFree( tmp );
                return pdTRUE;
            }
            next = next->next;

        }
        return pdTRUE;
    }
    else
        return pdFALSE;
}
int listDeleteItemByCMD( RunTaskDef *header, int ID )
{
    if( header )
    {
        RunTaskDef * next = header;
        RunTaskDef *tmp;
        while( next->next )
        {
            if( next->next->cmd == ID )
            {
                tmp = next->next;
                listDelCallBack( *tmp);
                next->next = next->next->next;
                vPortFree( tmp );
                return pdTRUE;
            }
            next = next->next;
        }
        return pdTRUE;
    }
    else
        return pdFALSE;
}
int listConverToJson( RunTaskDef *header, char *buff )
{
    if( !header )
        return pdFALSE;
    if( !header->next )
    {
        sprintf( buff, "{}" );
        return pdFALSE;
    }
    uint32_t timeNow = osKernelSysTick();
    RunTaskDef *next = header->next;
    sprintf( buff, "{" );
    int j = 0;
    while( next )
    {
        sprintf( buff+strlen(buff), "\"%d\":{\"ID\":%d,\"position\":%0.2f,\"cmd\":%d,\"iData\":%d,\"uData\":%u,\"fData\":%f},", j, next->ID, next->position, next->cmd,next->data.iData, next->data.uData, next->data.fData);
        j++;
        next = next->next;
    }
    sprintf( buff+strlen(buff) - 1, "}" );
    return pdTRUE;
}


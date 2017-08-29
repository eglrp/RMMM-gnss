/*
 * gnss_data.h
 *
 *  Created on: 8 juin 2017
 *      Author: jsanchez
 */

#ifndef SOURCES_APPS_POW_APP_GNSS_DATA_H_
#define SOURCES_APPS_POW_APP_GNSS_DATA_H_

#include "gnss_api.h"

typedef struct Gnss_data
{
    position_t pos;
    char timestamp[27];

}Gnss_data;

typedef struct Gnss_nvm
{
    Gnss_data positions[100];
    tUInt n_positions;

}Gnss_nvm;

#endif /* SOURCES_APPS_POW_APP_GNSS_DATA_H_ */

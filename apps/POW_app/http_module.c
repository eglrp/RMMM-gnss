/*
 * http_module.c
 *
 *  Created on: 24 juil. 2017
 *      Author: jsanchez
 */

#include "gnss_defs.h"
#include "libs/cJSON.h"
#include "libs/picohttpparser.h"
#include "POW_app.h"
#include "svc_uart.h"
#include "clibs.h"
#include "gnssapp.h"      /* to configure power services  */
#include "gnss_debug.h"
#include "gnss_api.h"     /* to use GNSS api              */

char * HTTP_POST_PACKAGE[] = {
    "Host: cloud-logger-159118.appspot.com",
    "Accept: */*",
    "User-Agent: runscope/0.1",
    "Accept-Encoding: gzip, deflate, sdch"

};


void analyse_response(char * response){

  int i;
  size_t num_headers=0;
  for(i=0;i<mystrlen(response)-7; i++){
    if(response[i]=='\r' && response[i+1]=='\n' && response[i+2]!='\r'){      //TO-DO Could be done better (count just before the payload to avoid fake positives)
      num_headers++;
    }
  }

  int minor_version;
  int status;
  const char *msg;
  size_t msg_len;
  struct phr_header headers[num_headers];



  int pret;
  while (1) {

    // parse the reponse
    pret = phr_parse_response(response, strlen(response), &minor_version, &status, &msg, &msg_len, headers, &num_headers, 0);
    if (pret > 0)
      break;  //successfully parsed the request
    else if (pret == -1){
      GPS_DEBUG_MSG(( "[CLOE_demo] PARSING ERROR\r\n"));
      break;
    }
    // request is incomplete, continue the loop
  }

  //GPS_DEBUG_MSG(( "[CLOE_demo] response %s\r\n", response));

  int payload_size = mystrlen(response)-pret+1;
  char payload[1500];
  switch (status){
    case 201:         // CREATED -> new config
      GPS_DEBUG_MSG(( "[CLOE_demo] Code 201 detected, searching the payload... \r\n"));
      _clibs_memcpy(payload, response + pret, payload_size);
      payload[payload_size] = '\0';
      GPS_DEBUG_MSG(( "[CLOE_demo] payload = %s\r\n", payload));
      set_new_config(payload);
      break;
    case 204:         // NO CONTENT -> nothing special
      GPS_DEBUG_MSG(( "[CLOE_demo] Positions well received by the cloud server\r\n"));
      break;
    default:          // NOT EXPECTED / ERROR
      GPS_DEBUG_MSG(( "[CLOE_demo] ERROR: The cloud server answered with a code %d\r\n", status));
      break;
  }
  return;

}


void send_packet_activate(){
  char post_init[50] = "PATCH /api/trackers/000000000000000 HTTP/1.1";         // Base pointer to store the real imei
  _clibs_sprintf(post_init, "PATCH /api/trackers/%s HTTP/1.1", config.imei);
  char * http_post_init_tracker[] = {
      post_init,
      "Host: cloud-logger-159118.appspot.com",
      "Accept: */*",
      "User-Agent: runscope/0.1",
      "Accept-Encoding: gzip, deflate, sdch"

  };

  cJSON *root;
  cJSON *data;
  cJSON *attributes;
  root = cJSON_CreateObject();
  cJSON_AddItemToObject(root, "data", data = cJSON_CreateObject());
  cJSON_AddStringToObject(data, "type", "trackers");
  char * e;
  errno = 0;
  long long int n = strtoll(config.imei, &e, 0);
  cJSON_AddNumberToObject(data, "id", n);
  cJSON_AddItemToObject(data, "attributes", attributes = cJSON_CreateObject());
  cJSON_AddBoolToObject(attributes, "activated", true);
  cJSON_AddBoolToObject(attributes, "remote", true);

  char *rendered = cJSON_Print(root);
  remove_all_chars(rendered, '\n');
  remove_all_chars(rendered, '\t');
  remove_all_chars(rendered, '\r');

  //Sending header

  //tU8 buff[] = POST_COMMAND;

  send_at(post_init);
  send_at("\r\n");

  int i;
  for(i=1; i<(sizeof(http_post_init_tracker)/sizeof(*http_post_init_tracker)); i++){
    send_at(http_post_init_tracker[i]);
    send_at("\r\n");
  }

  //Content-Length
  int length_payload = mystrlen(rendered);
  char str[18];
  _clibs_sprintf(str, "Content-Length: %d\0", length_payload);
  send_at(str);

  send_at("\r\n");
  send_at("\r\n");

  // HEADER FINISHED

  send_at(rendered);

  cJSON_Delete(root);

  send_at("\r\n");
  send_at("\r\n");
}

void send_http_package(){
  cJSON *root;
  cJSON *data;
  root = cJSON_CreateObject();
  cJSON_AddItemToObject(root, "data", data = cJSON_CreateArray());

  int i;
  for(i=0; i<n_positions; i++){
    cJSON *position;
    cJSON *geopoint;
    cJSON *created_on;
    cJSON *attributes;
    cJSON_AddItemToArray(data, position = cJSON_CreateObject());
    cJSON_AddStringToObject(position, "type", "positions");
    cJSON_AddItemToObject(position, "attributes", attributes = cJSON_CreateObject());
    //char buffer[17];
    //at_get_network_time(buffer);
    //GPS_DEBUG_MSG(( "[CLOE_demo] Buffer %s \r\n", buffer));
    GPS_DEBUG_MSG(( "[CLOE_demo] The latitude, longitude and timestamp are %.8f  ,  %.8f  , %s \r\n", positions[i].pos.latitude, positions[i].pos.longitude,positions[i].timestamp));
    char str[20];
    _clibs_sprintf(str, "%.8f,%.8f\0", positions[i].pos.latitude, positions[i].pos.longitude);
    cJSON_AddStringToObject(attributes, "geo_point", str);
    cJSON_AddStringToObject(attributes, "created_on", positions[i].timestamp);

  }


  char *rendered = cJSON_Print(root);
  remove_all_chars(rendered, '\n');
  remove_all_chars(rendered, '\t');
  remove_all_chars(rendered, '\r');

  //Sending header

  tU8 buff[] = "POST /api/fleets/0000000000000000/trackers/000000000000000/positions HTTP/1.1";
  _clibs_sprintf(buff, "POST /api/fleets/%s/trackers/%s/positions HTTP/1.1", config.fleet_id,config.imei);
  send_at(buff);
  send_at("\r\n");

  for(i=0; i<(sizeof(HTTP_POST_PACKAGE)/sizeof(*HTTP_POST_PACKAGE)); i++){
    send_at(HTTP_POST_PACKAGE[i]);
    send_at("\r\n");
  }

  //Content-Length
  int length_payload_per_position = mystrlen(rendered);
  char str2[18];
  _clibs_sprintf(str2, "Content-Length: %d\0", length_payload_per_position);
  send_at(str2);

  send_at("\r\n");
  send_at("\r\n");

  // HEADER FINISHED

  send_at(rendered);


  free(rendered);
  cJSON_Delete(root);

  send_at("\r\n");
  send_at("\r\n");
}

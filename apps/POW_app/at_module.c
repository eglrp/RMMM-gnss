/*
 * at_module.c
 *
 *  Created on: 24 mai 2017
 *      Author: jsanchez
 */

#include "POW_app.h"
#include "svc_uart.h"
#include "clibs.h"
#include "gnss_debug.h"

void report_error(char *name) {
  GPS_DEBUG_MSG(("[CLOE_demo] error %s\r\n", name ));
}

void send_at(const char *atc){
  svc_uart_write(COM_PORT, (tU8 *)atc, strlen(atc), gpOS_TIMEOUT_INFINITY);
  GPS_DEBUG_MSG(( "[CLOE_demo] send=%s\r\n", atc));
}

int read_server_reponse(char * output, int size){
  tU8 ch;
  tU32 nb, index = 0;
  gpOS_clock_t timeout = 5000000;

  output[index] = '\0';

  while(strstr(output, "\r\n\r\nOK\r\n") == NULL){
    nb = svc_uart_read(COM_PORT, &ch, 1, &timeout);

    if(nb == 0){

    }
    else {
      output[index] = ch;
      index++;
    }
  }

  output[index] = '\0';

  //GPS_DEBUG_MSG(( "[CLOE_demo] RECV: Expected size=%d\r\n",size));
  //GPS_DEBUG_MSG(( "[CLOE_demo] RECV: index=%d\r\n",index));


  return mystrlen(output);

}

boolean_t send_at_wait_response_server(char *atc, int max_bytes){
  char buffer[max_bytes];
  boolean_t status_t = true;
  int retries = 0;

  _clibs_memset(buffer, 0, MAX_READ_BUFFER_SIZE);

  send_at(atc);
  int size = read_server_reponse(buffer, max_bytes);

  //Get the index of the first \n to delete the content until the first line break (+SQNSRECV:1, xxxx)
  int i;
  for(i=0;i<strlen(buffer);i++){
    if(buffer[i]=='\n'){
      if(i>5) break; // Avoid the initial line break
    }
  }

  //GPS_DEBUG_MSG(( "[CLOE_demo] no filtered=%s\r\n", buffer));

  buffer[size]='\0';

  buffer[mystrlen(buffer)-4] = '\0'; // Delete the OK

  for (i=0;i<strlen(buffer) - 7;i++) { //Detecting the first line break give us troubles...
    if (buffer[i] == 'H' && buffer[i+1] == 'T' && buffer[i+2] == 'T' && buffer[i+3] == 'P' && buffer[i+4] == '/' && buffer[i+5] == '1' && buffer[i+6] == '.' && buffer[i+7] == '1') { // Super cool code here...
      memcpy(buffer, &buffer[i], mystrlen(buffer)-i); // Delete the first line
    }
  }

  //GPS_DEBUG_MSG(( "[CLOE_demo] filtered=%s\r\n", buffer));


  // Deleting every doubled couple \r\n
  int end = 0;
  for(i=0;i<mystrlen(buffer)-8; i++){
    if(buffer[i]=='\r' && buffer[i+1]=='\n' && buffer[i+2]=='\r' && buffer[i+3]=='\n' && buffer[i+4]!='\r' && end==0){
      buffer[i]=' ';
      buffer[i+1]=' ';
    }
    if(buffer[i]=='\r' && buffer[i+1]=='\n' && buffer[i+2]=='\r' && buffer[i+3]=='\n' && buffer[i+4]=='\r' && buffer[i+5]=='\n' && buffer[i+6]=='\r' && buffer[i+7]=='\n' && buffer[i+8]!='\r'){
      buffer[i]=' ';
      buffer[i+1]=' ';
      buffer[i+2]=' ';
      buffer[i+3]=' ';
      end=1;
    }
  }

  buffer[mystrlen(buffer)]='\0';
  //GPS_DEBUG_MSG(( "[CLOE_demo] RECV: Real size=%d\r\n", mystrlen(buffer)));
  GPS_DEBUG_MSG(( "[CLOE_demo] Analysing the answer...\r\n"));
  analyse_response(buffer);

  return status_t;
}

boolean_t at_recv_data(int port, int max_bytes);

void read_at(char * output, int size){
  tU8 ch;
  tU32 nb, index = 0;
  gpOS_clock_t timeout = 5000000;

  output[index] = '\0';

  while(1){
    nb = svc_uart_read(COM_PORT, &ch, 1, &timeout);

    if(nb == 0){
      break;
    }
    else {
      output[index] = ch;
      index++;
    }

    if((index>2) && !strcmp(&output[index-2],"\r\n")){              // Line break detected
      if((index>12) && !strcmp(&output[index-11], "+SYSSTART\r\n")){
        GPS_DEBUG_MSG(( "[CLOE_demo] SYSSTART Found\r\n"));
        break;
      }
      if((index>3) && !strcmp(&output[index-4], "OK\r\n")){
        break;
      }
      if(strstr(output, "+SQNSRING:") != NULL){
        int data_size = get_size(output);
        GPS_DEBUG_MSG(( "[CLOE_demo] RING found, size: %d\r\n", data_size));
        at_recv_data(SOCKET_ID,1500);
        break;
      }
      if((index>6) && !strcmp(&output[index-7], "ERROR\r\n")){
        break;
      }
    }
  }

  output[index] = '\0';

  if (index!=0){
    //to see contents in the logs
    remove_all_chars(output, '\n');
    remove_all_chars(output, '\r');
    GPS_DEBUG_MSG(( "[CLOE_demo] read=%s\r\n", output));
    //GPS_DEBUG_MSG(( "\r\n[CLOE_demo] Received %d characters\r\n", index));
  }

}

void send_at_get_response(char *atc, gpOS_clock_t *timeout_ms){
  char buffer[MAX_READ_BUFFER_SIZE];

  _clibs_memset(buffer, 0, sizeof(buffer));

  send_at(atc);

  if(timeout_ms != gpOS_TIMEOUT_IMMEDIATE)
  {
    gpOS_task_delay((*timeout_ms)*1000*gpOS_timer_ticks_per_usec());
  }

  read_at(buffer, sizeof(buffer));
}

boolean_t send_at_check_response(char *atc, char * resp){
  char buffer[MAX_READ_BUFFER_SIZE];
  _clibs_memset(buffer, 0, sizeof(buffer));

  send_at(atc);
  read_at(buffer, sizeof(buffer));
  if(strstr(buffer, resp)){
    return true;
  }else{
    return false;
  }
}

boolean_t send_at_until_response(char *atc, char * resp, int max_retries){
  char buffer[MAX_READ_BUFFER_SIZE];
  boolean_t not_timedout = true;
  int retries = 0;

  _clibs_memset(buffer, 0, sizeof(buffer));

  send_at(atc);
  read_at(buffer, sizeof(buffer));
  while(!strstr(buffer, resp)){
    gpOS_task_delay(100*1000*gpOS_timer_ticks_per_usec());
    if(retries++ >= max_retries){
      not_timedout = false;
      break;
    }
    send_at(atc);
    _clibs_memset(buffer, 0, sizeof(buffer));
    read_at(buffer, sizeof(buffer));
  }
  return not_timedout;
}

boolean_t send_at_wait_response(char *atc, char * resp, int max_retries){
  char buffer[MAX_READ_BUFFER_SIZE];
  boolean_t status = true;
  int retries = 0;

  _clibs_memset(buffer, 0, sizeof(buffer));

  send_at(atc);
  read_at(buffer, sizeof(buffer));
  while(!strstr(buffer, resp)){
    gpOS_task_delay(10*1000*gpOS_timer_ticks_per_usec());
    if(retries++ >= max_retries*10){
      status = false;
      break;
    }
    if(strstr(buffer, "ERROR")){
      status = false;
      break;
    }
    _clibs_memset(buffer, 0, sizeof(buffer));
    read_at(buffer, sizeof(buffer));
  }

  if( !status )
    GPS_DEBUG_MSG(( "[CLOE_demo] \"%s\" not found\r\n", resp));

  return status;
}

boolean_t send_at_read_response(char *atc, char * resp, int max_retries, char * output){
  output[0]='\0';
  char buffer[MAX_READ_BUFFER_SIZE];
  boolean_t status = true;
  int retries = 0;

  _clibs_memset(buffer, 0, sizeof(buffer));

  send_at(atc);
  read_at(buffer, sizeof(buffer));
  while(!strstr(buffer, resp)){
    gpOS_task_delay(100*1000*gpOS_timer_ticks_per_usec());
    if(retries++ >= max_retries){
      status = false;
      break;
    }
    if(strstr(buffer, "ERROR")){
      status = false;
      break;
    }
    _clibs_memset(buffer, 0, sizeof(buffer));
    read_at(buffer, sizeof(buffer));
  }

  _clibs_strcpy(output,buffer);

  return status;
}

boolean_t wait_urc(char * urc, int max_retries){
  char buffer[2*MAX_READ_BUFFER_SIZE];
  boolean_t not_timedout = true;
  int retries = 0;

  _clibs_memset(buffer, 0, sizeof(buffer));

  read_at(buffer, sizeof(buffer));

  while(!strstr(buffer, urc)){
    gpOS_task_delay(100*1000*gpOS_timer_ticks_per_usec());
    if(retries++ >= max_retries){
      not_timedout = false;
      break;
    }
    _clibs_memset(buffer, 0, sizeof(buffer));
    read_at(buffer, sizeof(buffer));
  }
  return not_timedout;
}

boolean_t at_disable_echo(){
  return send_at_wait_response(_AT("ATE0"), "OK", 10);
}

boolean_t at_check(){
  return send_at_wait_response(_AT("AT"), "OK", 10);
}

boolean_t at_disable_PSM(){
  return send_at_wait_response(_AT("AT+CPSMS=0"), "OK", 1);
}

/*
   Mobile Equipment Errors

    0 disable result code and use ERROR instead
    1 enable result code and use numeric <err> values
    2 enable result code and use verbose <err> values
 */

boolean_t at_configure_errors(int i){
  char str[15];
  _clibs_sprintf(str, "AT+CMEE=%d\r\n", i);
  return send_at_wait_response(str, "OK", 10);
}

boolean_t at_activate_modem(){
  return send_at_wait_response(_AT("AT+CFUN=1"), "OK", 10);
}

boolean_t at_configure_socket(int socket_id, int pdp_id, int packet_size, int exchange_timeout, int connection_timeout, int data_sending_timeout){
  char str[30];
  _clibs_sprintf(str, "at+sqnscfg=%d,%d,%d,%d,%d,%d\r\n", socket_id, pdp_id, packet_size, exchange_timeout, connection_timeout, data_sending_timeout);
  return send_at_wait_response(str, "OK", 10);
}

boolean_t at_extended_configure_socket(int socket_id, int sring_mode, int rcv_data_mode, int keep_alive){
  char str[30];
  _clibs_sprintf(str, "at+sqnscfgext=%d,%d,%d,%d\r\n", socket_id, sring_mode, rcv_data_mode, keep_alive);
  return send_at_wait_response(str, "OK", 10);
}

boolean_t at_extended_configure_socket_2(int socket_id, int sring_mode, int rcv_data_mode, int keep_alive, int listen_auto_reponse){
  char str[33];
  _clibs_sprintf(str, "at+sqnscfgext=%d,%d,%d,%d,%d\r\n", socket_id, sring_mode, rcv_data_mode, keep_alive, listen_auto_reponse);
  return send_at_wait_response(str, "OK", 10);
}

boolean_t at_extended_configure_socket_3(int socket_id, int sring_mode, int rcv_data_mode, int keep_alive, int listen_auto_reponse, int send_data_mode){
  char str[33];
  _clibs_sprintf(str, "at+sqnscfgext=%d,%d,%d,%d,%d,%d\r\n", socket_id, sring_mode, rcv_data_mode, keep_alive, listen_auto_reponse, send_data_mode);
  return send_at_wait_response(str, "OK", 10);
}

void at_get_imei(char *output){
  send_at_read_response(_AT("AT+CGSN"),"OK",10, output);
  int i = 0;
  while(output[i] != '\0')
  {
    i++;
  }
  output[i-2] = '\0'; // Delete the OK
}

void at_get_network_time(char *output){
  flush_rx();
  send_at_read_response(_AT("AT+CCLK?"),"OK",50, output);
}

boolean_t at_socket_shutdown(int socket_id){
  char str[30];
  _clibs_sprintf(str, "at+sqnsh=%d\r\n", socket_id);
  return send_at_wait_response(str, "OK", 50);
}

boolean_t at_socket_dial(int socket_id, int protocol, int port, char * ip, int closure_behaviour, int udp_port, int conn_mode){
  char str[100];
  _clibs_sprintf(str, "at+sqnsd=%d,%d,%d,%s,%d,%d,%d\r\n", socket_id, protocol, port, ip, closure_behaviour, udp_port, conn_mode);
  return send_at_wait_response(str, "OK", 100);
}

boolean_t at_check_signal_quality(){
  return send_at_wait_response(_AT("AT+CSQ"), "OK", 10);
}

boolean_t at_check_pin_status(){
  return send_at_wait_response(_AT("AT+CPIN?"), "OK", 10);
}

boolean_t at_check_eps_registration_status(){
  return send_at_wait_response(_AT("AT+CEREG?"),"+CEREG: 2,1", 5);
}

boolean_t at_check_pdp_context(tUInt pdn){
  char str[15];
  _clibs_sprintf(str, "+CGACT: %d,1", pdn);
  return send_at_wait_response(_AT("at+cgact?"),str, 5);
}

void at_activate_pdp(tUInt pdn){
  gpOS_clock_t std_at_timeout_ms = 100;
  char str[15];
    _clibs_sprintf(str, "at+cgact=1,%d\r\n", pdn);
  send_at_get_response(str, &std_at_timeout_ms);
}

boolean_t at_is_socket_closed(int socket){
  char str[15];
  _clibs_sprintf(str, "+SQNSS: %d,0", socket);
  return send_at_wait_response(_AT("AT+SQNSS"),str, 5);
}

boolean_t at_recv_data(int port, int max_bytes){
  char str[25];
  _clibs_sprintf(str, "at+sqnsrecv=%d,%d\r\n",port,max_bytes);
  return send_at_wait_response_server(str,max_bytes);
}

boolean_t at_send_data(int port){
  char str[20];
  _clibs_sprintf(str, "at+sqnssend=%d\r\n",port);
  return send_at_wait_response(str,"OK", 30);
}

boolean_t at_listen_server(){
  return wait_urc("+SQNSRING", 100);
}

boolean_t at_listen_sysstart(){
  return wait_urc("+SYSSTART", 50);
}

/***************************
 * Debug purpose functions *
 ***************************/

boolean_t at_showphystat(){
  GPS_DEBUG_MSG(( "[CLOE_demo] Expecting around 1172 characters\r\n"));
  return send_at_wait_response("AT!=\"showphystat\"\r\n","OK", 10);

}
boolean_t at_showpdn(){
  GPS_DEBUG_MSG(( "[CLOE_demo] Expecting around 1439 characters\r\n"));
  return send_at_wait_response("AT!=\"showpdn\"\r\n","OK", 10);
}

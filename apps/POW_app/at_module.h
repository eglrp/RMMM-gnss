/*
 * at_module.h
 *
 *  Created on: 20 juil. 2017
 *      Author: jsanchez
 */

#ifndef APPS_POW_APP_AT_MODULE_H_
#define APPS_POW_APP_AT_MODULE_H_

#include "gnss_defs.h"

void report_error(char *name);
boolean_t at_disable_echo();
boolean_t at_check();
boolean_t at_disable_PSM();
boolean_t at_configure_errors(int i);
boolean_t at_activate_modem();
boolean_t at_configure_socket(int socket_id, int pdp_id, int packet_size, int exchange_timeout, int connection_timeout, int data_sending_timeout);
boolean_t at_extended_configure_socket(int socket_id, int sring_mode, int rcv_data_mode, int keep_alive);
boolean_t at_extended_configure_socket_2(int socket_id, int sring_mode, int rcv_data_mode, int keep_alive, int listen_auto_reponse);
boolean_t at_extended_configure_socket_3(int socket_id, int sring_mode, int rcv_data_mode, int keep_alive, int listen_auto_reponse, int send_data_mode);
void at_get_imei(char *output);
void at_get_network_time(char *output);
boolean_t at_socket_shutdown(int socket_id);
boolean_t at_socket_dial(int socket_id, int protocol, int port, char * ip, int closure_behaviour, int udp_port, int conn_mode);
boolean_t at_check_signal_quality();
boolean_t at_check_pin_status();
boolean_t at_check_eps_registration_status();
boolean_t at_check_pdp_context(tUInt pdn);
void at_activate_pdp(tUInt pdn);
boolean_t at_is_socket_closed(int socket);
boolean_t at_recv_data(int port, int max_bytes);
boolean_t at_send_data(int port);
boolean_t at_listen_server();
boolean_t at_listen_sysstart();

/***************************
 * Debug purpose functions *
 ***************************/

boolean_t at_showphystat();
boolean_t at_showpdn();


#endif /* APPS_POW_APP_AT_MODULE_H_ */

/*
 * tools.c
 *
 *  Created on: 24 mai 2017
 *      Author: jsanchez
 */

#include "POW_app.h"
#include "clibs.h"
#include "gnss_defs.h"
#include "gnss_debug.h"
#include "gnss_events.h"
#include "gnss_api.h"
#include "lld_gpio.h"
#include "svc_uart.h"


void remove_all_chars(char* str, char c) {
  char *pr = str, *pw = str;
  while (*pr) {
    *pw = *pr++;
    pw += (*pw != c);
  }
  *pw = '\0';
}

int mystrlen(char *s)
{
  int i =0;
  while (*s++) i++;
  return i;
}

char* concat(int count, ...)
{
  va_list ap;
  int i;

  // Find required length to store merged string
  int len = 1; // room for NULL
  va_start(ap, count);
  for(i=0 ; i<count ; i++)
    len += strlen(va_arg(ap, char*));

  va_end(ap);

  // Allocate memory to concat strings
  char *merged = calloc(sizeof(char),len);
  int null_pos = 0;

  // Actually concatenate strings
  va_start(ap, count);
  for(i=0 ; i<count ; i++)
  {
    char *s = va_arg(ap, char*);
    strcpy(merged+null_pos, s);
    null_pos += strlen(s);
  }
  va_end(ap);

  return merged;
}


int split (char *str, char c, char ***arr)
{
  int count = 1;
  int token_len = 1;
  int i = 0;
  char *p;
  char *t;

  p = str;
  while (*p != '\0')
  {
    if (*p == c)
      count++;
    p++;
  }

  *arr = (char**) malloc(sizeof(char*) * count);
  if (*arr == NULL)
    exit(1);

  p = str;
  while (*p != '\0')
  {
    if (*p == c)
    {
      (*arr)[i] = (char*) malloc( sizeof(char) * token_len );
      if ((*arr)[i] == NULL)
        exit(1);

      token_len = 0;
      i++;
    }
    p++;
    token_len++;
  }
  (*arr)[i] = (char*) malloc( sizeof(char) * token_len );
  if ((*arr)[i] == NULL)
    exit(1);

  i = 0;
  p = str;
  t = ((*arr)[i]);
  while (*p != '\0')
  {
    if (*p != c && *p != '\0')
    {
      *t = *p;
      t++;
    }
    else
    {
      *t = '\0';
      i++;
      t = ((*arr)[i]);
    }
    p++;
  }

  return count;
}

int count_digits( int value )
{
  int result = 0;
  while( value != 0 ) {
    value /= 10;
    result++;
  }
  return result;
}

void parse_date_response(char * response){
  char num[23];
  char date[11];
  char hour[12];
  char *ptr;

  char joined[27];

  ptr = strchr(response, '\"'); // Detect the first quote mark

  num[0] = '2';
  num[1] = '0';
  memcpy(num+2, ptr+1, 20); // String length it's always 20
  int i;
  for(i=0; i<mystrlen(num); i++){
    if(num[i]=='/') num[i]='-';
  }
  num[22]='\0';
  memcpy(date, num, 10); // Retrieving the first 10 characters (date)
  date[10]='\0';
  memcpy(hour, num+11, 11); // Retrieving the second 11 characters (time)
  hour[11]='\0';

  memcpy(joined, date, 10); // Joining
  joined[10]='T';
  memcpy(joined+11, hour, 11);
  joined[19]='.';
  for(i=20;i<26;i++){ // Adding the nano seconds
    joined[i]='0';
  }
  joined[26]='\0';

  _clibs_strcpy(response, joined);
}

/*
 * Function that works with the AT commands  SQNSRECV and SQNSRING to get the data size
 */

int get_size(char* output){
  int c = 0;
  char **arr = NULL;
  c = split(output, ',', &arr);
  int data_size = atoi(arr[1]);
  free(arr);
  return data_size;
}
void flush_rx(){

  int size = MAX_READ_BUFFER_SIZE *2; //doubled it here since we were getting a TOO BIG response from server (>200 bytes)
  char buffer[size];
  _clibs_memset(buffer, 0, sizeof(buffer));

  tU8 ch;
  tU32 nb, index = 0;
  gpOS_clock_t timeout = 5000000;

  buffer[index] = '\0';

  while(1){
    nb = svc_uart_read(COM_PORT, &ch, 1, &timeout);
    if(nb == 0){
      break;
    }
    else {
      buffer[index] = ch;
      index++;
    }
    if(index == (size - 1)){
      GPS_DEBUG_MSG(( "[CLOE_demo] TOO BIG RESPONSE \r\n", index, ch));
      break;
    }
  }
  buffer[index] = '\0';
  if (index!=0){
    //to see contents in the logs
    remove_all_chars(buffer, '\n');
    remove_all_chars(buffer, '\r');
    GPS_DEBUG_MSG(( "[CLOE_demo] flushed=%s\r\n", buffer));
  }
}



#pragma once

#ifndef HTTPD_H_

#define HTTPD_H_

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "w5500.h"

#endif /* HTTPD_H_ */

typedef struct http_sock_prop {
	volatile uint8_t data_stat;			   // data transmit status
	volatile uint32_t data_size;		   // data to transmit length
	volatile uint16_t first_data_part_size; //size of first data part for transmit
	volatile uint16_t last_data_part_size; //size of last data part for transmit
	volatile uint16_t cnt_data_part; //total number of data parts to transfer			
	volatile uint16_t cnt_rem_data_part; //remain number of data parts to transfer				data parts is equal to data_size / (w5500_tx_size = 2kB)
	volatile uint8_t http_doc; // file variant to transmit
	char pageName[20];		   // page name
	uint8_t pageNum;
} http_sock_prop_ptr;

// HTTP file variants
#define EXISTING_HTML 0
#define E404_HTML 1

void http_request();
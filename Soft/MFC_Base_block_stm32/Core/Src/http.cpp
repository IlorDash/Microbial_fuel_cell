#include "http.h"
#include "web_pages.h"
#include <map>
#include <string>

#define PAGES_NUM 2
#define SECT_SIZE 512
#define SERVICE_BYTES_NUM 3
#define SOCK_TX_BUFF_SIZE 2048
#define EEPROM_PAGE_SIZE 128 // in Bytes

extern char str1[60];
extern char tmpbuf[30];
extern uint8_t sect[SECT_SIZE + SERVICE_BYTES_NUM];
extern tcp_prop_ptr tcpProp;
extern uint16_t *p_curPageNum;
extern uint16_t *p_curDataFrameNum;

http_sock_prop_ptr httpSockProp[2];

char *pages_names[] = {"hello_world.html"};
std::map<std::string, char *> pages_map = {{pages_names[0], hello_world_page}};

char table_page[SOCK_TX_BUFF_SIZE] = "";
const char tablePageName[] = "table";

const char http_header[] = {"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"};
const char error_header[] = {"HTTP/1.0 404 File not found\r\nServer: nginx\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"};
char *header;

bool isTablePage = false;
uint16_t txTableRowsNum = 0;
uint16_t totalTableRowsNum = 0;

bool in_array(char *value, char *array[]) {
	int size = PAGES_NUM;
	for (int i = 0; i < size; i++) {
		if (!strcmp(value, array[i])) {
			return true;
		}
	}

	return false;
}

void tcp_send_http_data(char *dataBuff, uint16_t data_len) {

	if (data_len > SOCK_TX_BUFF_SIZE) {
		HAL_GPIO_WritePin(ERROR_GPIO_Port, ERROR_Pin, GPIO_PIN_SET);
		while (1) {
		}
	}

	uint16_t end_point;
	uint8_t num_sect = 0;
	uint16_t len_sect;

	end_point = GetWritePointer(tcpProp.cur_sock);
	end_point += data_len;
	// Fill with data packet transmit buff
	SetWritePointer(tcpProp.cur_sock, end_point);
	end_point = GetWritePointer(tcpProp.cur_sock);

	num_sect = data_len / SECT_SIZE;

	for (uint16_t i = 0; i <= num_sect; i++) {
		// not last sector
		if (i < num_sect) {
			len_sect = SECT_SIZE;
		} else {
			len_sect = data_len;
		}

		memcpy(sect + SERVICE_BYTES_NUM, dataBuff + i * SECT_SIZE,
			   len_sect); // reading 512 byte sectors from map and writing them to tx

		w5500_writeSockBuf(tcpProp.cur_sock, end_point, (uint8_t *)sect, len_sect);
		end_point += len_sect;
		data_len -= len_sect;
	}

	RecvSocket(tcpProp.cur_sock);
	SendSocket(tcpProp.cur_sock); // sending http header
}

void tcp_send_http_data() {
	uint16_t data_len = 0;
	uint16_t header_len = 0;

	if (httpSockProp[tcpProp.cur_sock].http_doc == EXISTING_HTML) {

		header_len = strlen(http_header);
		tcp_send_http_data((char *)http_header, header_len);
		data_len = strlen(table_page_start);
		tcp_send_http_data(table_page_start, data_len);

		for (uint16_t i = 0; i < totalTableRowsNum; i++) { // starting from 1, because 0 we send with header

			memset(table_page, 0, strlen(table_page));
			getMeasTablePageFromEEPROM(table_page, i, 1);
			data_len = strlen(table_page);

			tcp_send_http_data(table_page, data_len);
		}

		data_len = strlen(table_page_end);
		tcp_send_http_data(table_page_end, data_len);
	} else { // error page case
		header_len = strlen(error_header);
		tcp_send_http_data((char *)error_header, header_len);

		data_len = strlen(e404_html);
		tcp_send_http_data(e404_html, data_len);
	}

	httpSockProp[tcpProp.cur_sock].data_stat = DATA_COMPLETED;
}

void http_request() {
	uint16_t point;
	uint8_t RXbyte;
	uint16_t i = 0;
	char *ss1;
	int ch1 = '.';

	// finding first "/" symbol in HTTP head
	point = GetReadPointer(tcpProp.cur_sock);
	i = 0;
	while (RXbyte != (uint8_t)'/') {
		RXbyte = w5500_readSockBufByte(tcpProp.cur_sock, point + i);
		i++;
	}
	point += i;
	RXbyte = w5500_readSockBufByte(tcpProp.cur_sock, point);

	if (RXbyte == (uint8_t)' ') {
		char tempTablePageName[16] = "";
		strcpy(tempTablePageName, (char *)tablePageName);
		strcpy(httpSockProp[tcpProp.cur_sock].pageName, strcat(tempTablePageName, "_1"));
		httpSockProp[tcpProp.cur_sock].http_doc = EXISTING_HTML;
	} else {
		// finding next space (" ") in HTTP head, reading file name from request
		i = 0;
		while (tmpbuf[i - 1] != (uint8_t)' ') {
			tmpbuf[i] = w5500_readSockBufByte(tcpProp.cur_sock, point + i);
			i++;
		}
		i--;		   // set cntr to end of file name
		tmpbuf[i] = 0; //закончим строку
		strcpy(httpSockProp[tcpProp.cur_sock].pageName, tmpbuf);
	}

	isTablePage = !(strncmp(httpSockProp[tcpProp.cur_sock].pageName, tablePageName, strlen(tablePageName)));
	if (isTablePage) {
		httpSockProp[tcpProp.cur_sock].http_doc = EXISTING_HTML;
		totalTableRowsNum = (*p_curPageNum) * EEPROM_PAGE_SIZE / DATA_FRAME_LENGTH + *p_curDataFrameNum;
		// } else if (in_array(httpSockProp[tcpProp.cur_sock].pageName, pages_names)) {
		// 	httpSockProp[tcpProp.cur_sock].http_doc = EXISTING_HTML;
		// 	httpSockProp[tcpProp.cur_sock].data_size = strlen(http_header);
		// 	httpSockProp[tcpProp.cur_sock].data_size += strlen(pages_map[httpSockProp[tcpProp.cur_sock].pageName]);
	} else {
		httpSockProp[tcpProp.cur_sock].http_doc = E404_HTML;
		// first include header size
		httpSockProp[tcpProp.cur_sock].data_size = strlen(error_header);
		// then size of document itself
		httpSockProp[tcpProp.cur_sock].data_size += strlen(e404_html);
	}

	// httpSockProp[tcpProp.cur_sock].cnt_rem_data_part = httpSockProp[tcpProp.cur_sock].data_size / SOCK_TX_BUFF_SIZE + 1;
	// httpSockProp[tcpProp.cur_sock].last_data_part_size = httpSockProp[tcpProp.cur_sock].data_size % SOCK_TX_BUFF_SIZE;
	// if (httpSockProp[tcpProp.cur_sock].last_data_part_size == 0) {
	// 	httpSockProp[tcpProp.cur_sock].last_data_part_size = SOCK_TX_BUFF_SIZE; // calc number of data parts
	// 	httpSockProp[tcpProp.cur_sock].cnt_rem_data_part--;
	// }
	// httpSockProp[tcpProp.cur_sock].cnt_data_part = httpSockProp[tcpProp.cur_sock].cnt_rem_data_part;
	// httpSockProp[tcpProp.cur_sock].first_data_part_size = SOCK_TX_BUFF_SIZE;
	// if (httpSockProp[tcpProp.cur_sock].cnt_data_part == 1) {
	// 	httpSockProp[tcpProp.cur_sock].first_data_part_size = httpSockProp[tcpProp.cur_sock].last_data_part_size;
	// }

	tcp_send_http_data();

	DisconnectSocket(tcpProp.cur_sock); // Disconnecting
	SocketClosedWait(tcpProp.cur_sock);

	OpenSocket(tcpProp.cur_sock, Mode_TCP);
	// Waiting init socket (status SOCK_INIT)
	SocketInitWait(tcpProp.cur_sock);

	// Continue listering socket
	ListenSocket(tcpProp.cur_sock);
	SocketListenWait(tcpProp.cur_sock);
}

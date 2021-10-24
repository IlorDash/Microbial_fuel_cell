#include "http.h"
#include "web_pages.h"
#include <map>
#include <string>

#define PAGES_NUM 2
#define SECT_SIZE 512
#define SERVICE_BYTES_NUM 3
#define SOCK_TX_BUFF_SIZE 2048

extern char str1[60];
extern char tmpbuf[30];
extern uint8_t sect[SECT_SIZE + SERVICE_BYTES_NUM];
http_sock_prop_ptr httpSockProp[2];
extern tcp_prop_ptr tcpProp;

volatile uint16_t tcp_size_wnd = 2048;

char *pages_names[] = {"hello_world.html"};
std::map<std::string, char *> pages_map = {{pages_names[0], hello_world_page}};

char table_page[SOCK_TX_BUFF_SIZE] = "";
const char tablePageName[] = "table";

const char http_header[] = {"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"};
const char error_header[] = {"HTTP/1.0 404 File not found\r\nServer: nginx\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"};
char *header;

bool in_array(char *value, char *array[]) {
	int size = PAGES_NUM;
	for (int i = 0; i < size; i++) {
		if (!strcmp(value, array[i])) {
			return true;
		}
	}
	return false;
}

void tcp_send_http_one_data() {
	uint16_t i = 0;
	uint16_t data_len = 0;
	uint16_t header_len = 0;
	uint16_t end_point;
	uint8_t num_sect = 0;
	uint16_t len_sect;

	if (httpSockProp[tcpProp.cur_sock].http_doc == EXISTING_HTML) {

		header = (char *)http_header;
		header_len = strlen(header);
		bool isTablePage = !strncmp(httpSockProp[tcpProp.cur_sock].pageName, tablePageName, strlen(tablePageName));
		if (isTablePage) {
			memset(table_page, 0, strlen(table_page));
			strcat(table_page, table_page_start);
			getMeasTablePageFromEEPROM(table_page, httpSockProp[tcpProp.cur_sock].pageNum);
			strcat(table_page, table_page_end);
                        data_len = strlen(table_page);
		} else {
			data_len = strlen(pages_map[httpSockProp[tcpProp.cur_sock].pageName]);
		}

		end_point = GetWritePointer(tcpProp.cur_sock);
		end_point += header_len + data_len;
		// Fill with data packet transmit buff
		SetWritePointer(tcpProp.cur_sock, end_point);
		end_point = GetWritePointer(tcpProp.cur_sock);
		memcpy(sect + SERVICE_BYTES_NUM, header, header_len); // 3 service bytes
		w5500_writeSockBuf(tcpProp.cur_sock, end_point, (uint8_t *)sect, header_len);
		end_point += header_len;

		num_sect = data_len / SECT_SIZE;

		for (i = 0; i <= num_sect; i++) {
			// not last sector
			if (i < num_sect) {
				len_sect = 512;
			} else {
				len_sect = data_len;
			}

			if (isTablePage) {
				memcpy(sect + SERVICE_BYTES_NUM, table_page + i * SECT_SIZE,
					   SECT_SIZE); // reading 512 byte sectors from map and writing them to tx
			} else {
				memcpy(sect + SERVICE_BYTES_NUM, pages_map[httpSockProp[tcpProp.cur_sock].pageName] + i * SECT_SIZE,
					   SECT_SIZE); // reading 512 byte sectors from map and writing them to tx
			}

			w5500_writeSockBuf(tcpProp.cur_sock, end_point, (uint8_t *)sect, len_sect);
			end_point += len_sect;
			data_len -= len_sect;
		}
	} else { // error page case
		header_len = strlen(error_header);
		data_len = strlen(e404_html);
		end_point = GetWritePointer(tcpProp.cur_sock);
		end_point += header_len + data_len;
		SetWritePointer(tcpProp.cur_sock, end_point);
		end_point = GetWritePointer(tcpProp.cur_sock);

		// Fill with data packet transmit buff
		memcpy(sect + 3, error_header, header_len);
		w5500_writeSockBuf(tcpProp.cur_sock, end_point, (uint8_t *)sect, header_len);
		end_point += header_len;
		memcpy(sect + 3, e404_html, data_len);
		w5500_writeSockBuf(tcpProp.cur_sock, end_point, (uint8_t *)sect, data_len);
		end_point += data_len;
	}

	RecvSocket(tcpProp.cur_sock);
	SendSocket(tcpProp.cur_sock); // sending http answer
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

	bool is_page_table = !(strncmp(httpSockProp[tcpProp.cur_sock].pageName, tablePageName, strlen(tablePageName)));
	if (is_page_table) {
		httpSockProp[tcpProp.cur_sock].http_doc = EXISTING_HTML;
		// first include header size
		httpSockProp[tcpProp.cur_sock].data_size = strlen(http_header);
		// then size of document itself

		httpSockProp[tcpProp.cur_sock].pageNum = atoi(httpSockProp[tcpProp.cur_sock].pageName + strlen(tablePageName) + 1);
	} else if (in_array(httpSockProp[tcpProp.cur_sock].pageName, pages_names)) {
		httpSockProp[tcpProp.cur_sock].http_doc = EXISTING_HTML;
		// first include header size
		httpSockProp[tcpProp.cur_sock].data_size = strlen(http_header);
	} else {
		httpSockProp[tcpProp.cur_sock].http_doc = E404_HTML;
		// first include header size
		httpSockProp[tcpProp.cur_sock].data_size = strlen(error_header);
		// then size of document itself
		httpSockProp[tcpProp.cur_sock].data_size += strlen(e404_html);
	}

	tcp_send_http_one_data();

	DisconnectSocket(tcpProp.cur_sock); // Disconnecting
	SocketClosedWait(tcpProp.cur_sock);

	OpenSocket(tcpProp.cur_sock, Mode_TCP);
	// Waiting init socket (status SOCK_INIT)
	SocketInitWait(tcpProp.cur_sock);

	// Continue listering socket
	ListenSocket(tcpProp.cur_sock);
	SocketListenWait(tcpProp.cur_sock);
}

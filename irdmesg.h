#pragma once

#include <string>

void send_string(const std::string& str);
void send_message(const std::string& str);

void log_kmesg_loop();
void log_logcat_loop();

void start_log_server();

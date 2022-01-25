#include "irdmesg.h"

#include <algorithm>
#include <cstdint>
#include <vector>
#include <thread>
#include <chrono>
#include <regex>
#include <array>
#include <mutex>

#include <queue>
#include <condition_variable>

#include <fcntl.h>
#include <unistd.h>
#include <byteswap.h>

#define REMOTE_DEV "/dev/remote_ctrl_dev"
#define KMSG_FILE "/proc/kmsg"
#define MSG_OFFSET 22

static constexpr size_t CMD_START_SIZE = 8;
static constexpr uint8_t CMD_START[] = {
    0x00,
    0x06,
    0x69,
    0x69,
    0x96,
    0x96,
    0x02,
    0x04
};

namespace {

using std::regex;

static const std::vector<regex> MSG_FILTER = {
    regex{"Remote_ctl"},
    regex{"swapper"},
    regex{"remote_ldo"},
    regex{"remote_ctrl"},
    regex{"remote_gpio"},
    regex{"irdmesg"},
    regex{"i2c"},
    regex{"I2C"},
};

}

class LogQueue {
public:
    void putMessage(const std::string& msg) {
        std::lock_guard<std::mutex> lock{mMutex};

        mMessageQueue.emplace(msg);

        mCondVar.notify_all();
    }

    std::string getMessage() {
        std::unique_lock<std::mutex> lock{mMutex};

        mCondVar.wait(lock, [this] { return !mMessageQueue.empty(); });

        const auto msg = mMessageQueue.front();
        mMessageQueue.pop();
        lock.unlock();

        return msg;
    }
private:
    std::mutex mMutex;
    std::condition_variable mCondVar;

    std::queue<std::string> mMessageQueue;
};

static LogQueue logQueue;

static void send_chars(const char* chars, int size);

static void write_dev(const void* data, size_t count);
static void send_bytes(const void* data, size_t count);

static uint8_t convert_ticks(uint8_t ticks);
static std::vector<uint16_t> char_to_pattern(char c);

static bool filter_string(const std::string& str);
static std::string format_string(const std::string& str);

void start_log_server() {
    auto t = std::thread{[] {
        while (true) {
            const auto msg = logQueue.getMessage();
            send_string(msg);
        }
    }};

    t.detach();
}

void log_kmesg_loop() {
    constexpr size_t read_size = 4096;

    std::array<char, read_size> read_buffer;

    int kmesg_fd = open(KMSG_FILE, O_RDONLY);
    ssize_t bytes_read = 0;

    const auto read_buffer_begin = std::cbegin(read_buffer);
    while (true) {
        bytes_read = read(kmesg_fd, read_buffer.data(), read_size);
        for (auto it = read_buffer_begin; ; ) {
            const auto newline_it = std::find(it, std::cend(read_buffer), '\n');
            std::string msg_str{it, newline_it};
//            printf("string_to_send: '%s'\n", msg_str.c_str());
            if (filter_string(msg_str)) {
                send_message(msg_str);
            }
            it = newline_it + 1;
            if (std::distance(read_buffer_begin, it) > bytes_read) {
                break;
            }
        }
    }

    close(kmesg_fd);
}

void log_logcat_loop() {
    constexpr size_t read_size = 4096;
    std::array<char, read_size> read_buffer;

    FILE* fstream = nullptr;
    char* res = nullptr;

    // wait for logcat
    while(true) {
        if (fstream != nullptr) {
            pclose(fstream);
        }
        fstream = popen("/system/bin/logcat -v monotonic", "r");
        res = fgets(read_buffer.data(), read_size, fstream);

        if (res == nullptr) {
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(10ms);
        } else {
            break;
        }
    }

    while (true) {
        read_buffer.fill(0);
        (void)fgets(read_buffer.data(), read_size, fstream);

        auto first_char = std::begin(read_buffer);

        while (*first_char == ' ') {
            first_char++;
        }

        const auto newline_it = std::find(first_char, std::end(read_buffer), '\n');
        std::string msg_str{first_char, newline_it};
        send_message(msg_str);
    }
}

void send_message(const std::string& str) {
    logQueue.putMessage(format_string(str));
}

void send_string(const std::string& str) {
    constexpr int portion = 18;
    const auto str_size = str.length();
    size_t i = 0;
    const auto msg_c_str = str.c_str();
    while (true) {
        if (i + portion > str_size) {
//            printf("i = %d, i + portion = %d, str_size  = %lu, str_size -i  = %d\n", i, i+portion, str_size, str_size - i);
            send_chars(msg_c_str + i,  str_size - i);
            break;
        }

        send_chars(msg_c_str + i, portion);
        i += portion;
    }
}

static bool filter_string(const std::string& str) {
//    if (str.length() < MSG_OFFSET) {
//        return false;
//    }

    for (const auto& rx : MSG_FILTER) {
        if (std::regex_search(str, rx)) {
            return false;
        }
    }

    return true;
}

static std::string format_string(const std::string& str) {
    return std::string{std::cbegin(str), std::cend(str)} + "\r\n";
}

static void send_chars(const char* chars, int size) {
    std::vector<uint16_t> pattern;

    for (int i = 0; i < size; i++) {
        const auto char_pattern = char_to_pattern(chars[i]);
        pattern.insert(std::cend(pattern), std::cbegin(char_pattern), std::cend(char_pattern));
    }

    std::vector<uint16_t> packet;
    packet.reserve(pattern.size() + 8);

    packet.emplace_back(pattern.size() * 2 + 6);
    packet.emplace_back(0x0000);
    // Carrier frequency 38 KHz
    packet.emplace_back(0x9470);

    packet.insert(std::cend(packet), std::cbegin(pattern), std::cend(pattern));

    uint16_t checksum = 0;
    uint8_t* packet_bytes = reinterpret_cast<uint8_t*>(packet.data());
    const auto packet_bytes_size = packet.size() * 2;

    for(size_t i = 0; i < packet_bytes_size; i++) {
        checksum += packet_bytes[i];
    }

    packet.emplace_back(checksum);

    for(auto& f : packet) {
        f = bswap_16(f);
    }

    send_bytes(packet.data(), packet.size() * 2);
}

static void write_dev(const void* data, size_t count) {
    int dev_fd = open(REMOTE_DEV, O_RDWR);
    (void)write(dev_fd, data, count);
    close(dev_fd);
}

static void send_bytes(const void* data, size_t count) {
    write_dev(CMD_START, CMD_START_SIZE);
    write_dev(data, count);
}

static uint8_t convert_ticks(uint8_t ticks) {
    if (ticks < 6) {
        return 0x0f + (16 * (ticks - 1));
    } else {
        return 0x0e + (16 * (ticks - 1));
    }
}

static std::vector<uint16_t> char_to_pattern(char c) {
    // pattern with start bit
    std::vector<uint16_t> pattern = {1};

    int level = 0;
    int len = 0;
    bool firstBit = true;

    for (unsigned char i = 0; i < 8; ++i)
    {
        unsigned char bit = (c >> i) & 1;
        if(bit == level)
        {
            len += 1;
        }
        else
        {
            if (firstBit) {
                pattern[0] += len;
                firstBit = false;
            } else {
                pattern.emplace_back(len + 1);
            }

            level = bit;
            len = 0;
        }
    }

    pattern.emplace_back(len + 1);

    // add stop bit
    if (level == 1) {
        pattern[pattern.size() - 1] += 1;
    } else {
        pattern.emplace_back(1);
    }

    for (uint16_t& l : pattern) {
        l = convert_ticks(l);
    }

    return pattern;
}

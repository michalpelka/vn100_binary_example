#pragma once
#include "thread"
#include <string>
#include <mutex>
#include <atomic>
#include <chrono>
#include <vector>
#include <functional>

class vn100_client {
public:
    const std::vector<uint8_t> header_expected1{0xFA, 0x01}; // Heder with hello and 1st group activated;
	virtual ~vn100_client();
    using DataHandlerType=std::function<void(double,std::array<float,4>,std::array<float,3>,std::array<float,3>)>;
    vn100_client(const std::string& portname, int baudrate=230400);
    std::string getReport() const;
    int getRate() {
        return msgs_count_1sec;
    }

    void setDone(){
        done.store(true);
    }

    bool getDone(){
        return done;
    }
    void setHandler_pps_change(const std::function<void(double, double)> &_handler_pps_change) {
        handler_pps_middle = _handler_pps_change;
    }

    void setHandler_data(const DataHandlerType &_data_handler) {
        data_handler = _data_handler;
    }
private:
    void vn100_client_listener_thread_worker();
    void vn100_client_monitor_thread_worker();

    std::string portname;
    int baudrate;
    std::string data;
    std::thread listner_thread;
    std::thread monitor_thread;

    mutable std::mutex mtx;
    std::atomic<bool> done;
    std::atomic<int> msgs;
    std::atomic<int> msgs_count_1sec;

    int count_since_turnover = 0;
    int last_timestamp = 0;
    const float expected_framerate=101;
    double middle_handler_fired_at=0;
    double last_timestamp_sent = 0;
    std::function<void(double, double)> handler_pps_middle;
    DataHandlerType data_handler;

};

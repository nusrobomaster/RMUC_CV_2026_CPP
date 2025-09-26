#include <iostream>
#include <thread>
#include "calibur/log.h"
#include "calibur/util.h"

int main(int argc, char** argv) {
    calibur::Logger::ptr logger(new calibur::Logger);
    logger->addAppender(calibur::LogAppender::ptr(new calibur::StdoutLogAppender));

    calibur::FileLogAppender::ptr file_appender(new calibur::FileLogAppender("./log.txt"));

    calibur::LogFormatter::ptr fmt(new calibur::LogFormatter("%d%T%p%T%m%n"));
    file_appender->setFormatter(fmt);

    file_appender->setLevel(calibur::LogLevel::ERROR);
    logger->addAppender(file_appender);

    // calibur::LogEvent::ptr event(new calibur::LogEvent(logger, calibur::LogLevel::INFO, __FILE__, __LINE__, 0, calibur::GetThreadId(), calibur::GetFiberId(), time(0)));
    // event->getSS() << "Hello Sylar Log";
    
    // logger->log(calibur::LogLevel::DEBUG, event);
    std::cout << "hello calibur log" << std::endl;

    CALIBUR_LOG_INFO(logger) << "test macro";
    CALIBUR_LOG_ERROR(logger) << "test macro error";

    CALIBUR_LOG_FMT_ERROR(logger, "test macro fmt error %s", "aa");

    auto l = calibur::LoggerMgr::GetInstance()->getLogger("xx");
    CALIBUR_LOG_INFO(l) << "xxx";

    return 0;
}

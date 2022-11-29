#include <septentrio_gnss_driver/communication/communication_core.hpp>

#define BUFFER_SAFE 2000
#define BUFFER_SIZE 2048

#define ERROR_CPU_LOAD 100
#define WARN_CPU_LOAD  95

void io_comm_rx::Comm_IO::check_software_error(
    diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    uint8_t level = 0;
    std::string msg = "OK";

    if ((last_receiverstatus_.rx_error >> 3) & 0x01)
    {
        level = 2;
        msg = "Error";
    }

    stat.summary(level, msg);
};

void io_comm_rx::Comm_IO::check_watchdog_error(
    diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    uint8_t level = 0;
    std::string msg = "OK";

    if ((last_receiverstatus_.rx_error >> 4) & 0x01)
    {
        level = 2;
        msg = "Error";
    }

    stat.summary(level, msg);
}

void io_comm_rx::Comm_IO::check_antenna_error(
    diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    uint8_t level = 0;
    std::string msg = "OK";

    if ((last_receiverstatus_.rx_error >> 5) & 0x01)
    {
        level = 2;
        msg = "Error";
    }

    stat.summary(level, msg);
}

void io_comm_rx::Comm_IO::check_congestion_error(
    diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    uint8_t level = 0;
    std::string msg = "OK";

    if ((last_receiverstatus_.rx_error >> 6) & 0x01)
    {
        level = 2;
        msg = "Error";
    }

    stat.summary(level, msg);
}

void io_comm_rx::Comm_IO::check_missedevent_error(
    diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    uint8_t level = 0;
    std::string msg = "OK";

    if ((last_receiverstatus_.rx_error >> 8) & 0x01)
    {
        level = 2;
        msg = "Error";
    }

    stat.summary(level, msg);
}

void io_comm_rx::Comm_IO::check_invalidconfig_error(
    diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    uint8_t level = 0;
    std::string msg = "OK";

    if ((last_receiverstatus_.rx_error >> 10) & 0x01)
    {
        level = 2;
        msg = "Error";
    }

    stat.summary(level, msg);
}

void io_comm_rx::Comm_IO::check_outofgeofence_error(
    diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    uint8_t level = 0;
    std::string msg = "OK";

    if ((last_receiverstatus_.rx_error >> 11) & 0x01)
    {
        level = 2;
        msg = "Error";
    }

    stat.summary(level, msg);
}

void io_comm_rx::Comm_IO::check_antenna_state(
    diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    uint8_t level = 0;
    std::string msg = "OK";

    if (!((last_receiverstatus_.rx_status >> 1) & 0x01))
    {
        level = 2;
        msg = "Error";
    }

    stat.summary(level, msg);
}

void io_comm_rx::Comm_IO::check_cpu_load_state(
    diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    uint8_t level = 0;
    std::string msg = "OK";

    if (last_receiverstatus_.cpu_load >= WARN_CPU_LOAD &&
        last_receiverstatus_.cpu_load < ERROR_CPU_LOAD)
    {
        level = 1; // WARN
        msg = "Warning";
    }

    stat.addf("CPU Load [%]", "%d", last_receiverstatus_.cpu_load);
    stat.summary(level, msg);
}

void io_comm_rx::Comm_IO::check_connection_state(
    diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    uint8_t level = 0;
    std::string msg = "OK";

    if (!is_connect_)
    {
        level = 2;
        msg = "Error";
    }

    stat.summary(level, msg);
}

void io_comm_rx::Comm_IO::diagnostic_update()
{
    if (is_connect_)
        last_receiverstatus_ = handlers_.getRxMessage().getReceiverStatus();
    
    diagnostic_updater_.force_update();
}

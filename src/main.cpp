/*
 * Solar Scintillation Monitor Logger
 * Copyright (c) 2017-2020 Filip Szczerek <ga.software@yahoo.com>
 *
 * This project is licensed under the terms of the MIT license
 * (see the LICENSE file for details).
 *
 */

#include <boost/asio.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <ctime>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <ncurses/ncurses.h>
#include <optional>
#include <stdexcept>
#include <thread>

#ifdef __unix__
#include <termios.h>
#endif

// If set to 1, serial port communications with SSM are simulated
// without the need to connect an actual device
#define SSM_IO_SIMULATION 0

namespace p_opt = boost::program_options;
using namespace std::chrono_literals;

constexpr auto LOG_FLUSH_INTERVAL = 30s;

WINDOW *screen; ///< NCurses window representing the screen
std::chrono::high_resolution_clock::time_point logStart;

struct Timestamps
{
    std::string local;
    std::string utc;
};

class NCursesScope
{
    bool ended = false;
public:
    NCursesScope()  { screen = initscr(); curs_set(0); }
    ~NCursesScope() { if (!ended) EndNCurses(); }
    void EndNCurses() {  noraw(); endwin(); ended = true; }
};

namespace Display
{
    struct Pos { int row, col; };

    constexpr Pos Message = { 7, 0 };
    constexpr Pos Seeing  = { 3, 0 };
    constexpr Pos Input   = { 4, 0 };
    constexpr Pos Samples = { 5, 0 };
    constexpr Pos Timestamp = { 8, 0 };
    constexpr Pos LogFile   = { 9, 0 };
}

namespace CommProtocol
{
    // Standard identifiers
    const char *Input  = "A0";
    const char *Seeing = "A1";

    // Additional identifiers
    const char *Samples = "D0";
}

namespace CmdLineOptions
{
    const char *Help    = "help";
    const char *Port    = "port";
    const char *LogFile = "log-file";
}

#define POS_XY(PositionStruct)  PositionStruct.row, PositionStruct.col

void ClearToEoL(int x, int y)
{
    move(x, y);
    clrtoeol();
}

void ShowMessage(const char *format, ...)
{
    ClearToEoL(POS_XY(Display::Message));

    va_list args;
    va_start(args, format);
    vw_printw(screen, format, args);
    va_end(args);

    refresh();
}

Timestamps GetTimestamps(const std::chrono::high_resolution_clock::time_point &t)
{
    char buf[128] = { 0 };
    Timestamps result{};

    const unsigned milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch()).count() % 1000;
    char msbuf[5] = { 0 };
    snprintf(msbuf, sizeof(msbuf), ".%03u", milliseconds);
    const auto tt = std::chrono::high_resolution_clock::to_time_t(t);

    const auto tUtc = std::gmtime(&tt);
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", tUtc);
    result.utc = std::string(buf) + msbuf;

    const auto tLocal = std::localtime(&tt);
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", tLocal);
    result.local = std::string(buf) + msbuf;

    return result;
}

void PrintInfo()
{
    std::cout << "Solar Scintillation Monitor Logger\n\n";
}

void PrintUsage()
{
    std::cout << "Usage: \n"
                 "        ssm_logger [--" << CmdLineOptions::Port << "] <serial_port> [[--" << CmdLineOptions::LogFile << "] <log_file_path>]\n\n";
}

void PrintValue(const Display::Pos &pos, const char *label, const std::string &valueStr, bool valueInBold = true)
{
    ClearToEoL(POS_XY(pos));
    mvprintw(POS_XY(pos), "%-10s ", (std::string(label) + ":").c_str());
    if (valueInBold) attron(A_BOLD);
    printw("%s", valueStr.c_str());
    if (valueInBold) attroff(A_BOLD);
}

int main(int argc, char *argv[])
{
    PrintInfo();

    std::string serialPortName;
    std::string logFilePath;

    p_opt::options_description optDescr("Allowed options");
    optDescr.add_options()(CmdLineOptions::Help, "show help message")
                          (CmdLineOptions::Port,    p_opt::value<std::string>(&serialPortName), "specify serial port")
                          (CmdLineOptions::LogFile, p_opt::value<std::string>(&logFilePath), "specify log file");

    p_opt::positional_options_description posOptDescr;
    posOptDescr.add(CmdLineOptions::Port, 1)
               .add(CmdLineOptions::LogFile, 1);

    p_opt::variables_map progOptVars;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(optDescr).positional(posOptDescr).run(), progOptVars);
    p_opt::notify(progOptVars);

    if (progOptVars.count(CmdLineOptions::Help))
    {
        PrintUsage();
        std::cout << optDescr << std::endl;
        return EXIT_SUCCESS;
    }

    if (progOptVars.count(CmdLineOptions::Port) == 0)
    {
        PrintUsage();
        std::cout << optDescr << std::endl;
        return EXIT_FAILURE;
    }

    std::optional<std::ofstream> logFile;
    bool logFileExisted = false;
    if (progOptVars.count(CmdLineOptions::LogFile))
    {
        logFileExisted = std::filesystem::exists(logFilePath);
        logFile = std::ofstream(logFilePath, std::ios_base::out | std::ios_base::app); // append to previous contents
        if (!logFile->is_open())
        {
            std::cout << "Could not open " << logFilePath << " for writing." << std::endl;
            return EXIT_FAILURE;
        }
    }

#if __unix__ && !SSM_IO_SIMULATION
    // Uploading a sketch to Arduino may leave the terminal in an undesired state;
    // we want to disable all input/output processing (i.e. enable "raw" operation).

    termios term;
    int fd = open(serialPortName.c_str(), O_RDWR);
    if (fd == -1)
    {
        std::cout << "Could not open serial port " << serialPortName << "." << std::endl;
        return EXIT_FAILURE;
    }

    if (tcgetattr(fd, &term) < 0)
    {
        std::cout << "Call to tcgetattr() failed. " << std::endl;
        return EXIT_FAILURE;
    }

    cfmakeraw(&term);

    if (tcsetattr(fd, TCSANOW, &term) < 0)
    {
        std::cout << "Call to tcsetattr() failed. " << std::endl;
        return EXIT_FAILURE;
    }

    close(fd);
#endif // if __unix__ && !SSM_IO_SIMULATION

    try
    {
#if !SSM_IO_SIMULATION

        boost::asio::io_service io;
        boost::asio::serial_port serialPort(io, serialPortName);
        serialPort.set_option(boost::asio::serial_port::baud_rate(115200));

        if (!serialPort.is_open())
        {
            std::cout << "Could not open serial port " << serialPortName << "." << std::endl;
            return EXIT_FAILURE;
        }

        boost::asio::deadline_timer timer(io);

#endif

        bool timedOut = false;

        NCursesScope ncursesScope;

        std::string title = "Solar Scintillation Monitor";
#if SSM_IO_SIMULATION
        title += " (simulation)";
#endif
        mvprintw(0, 0, title.c_str());
        mvprintw(1, 0, "Press Q to quit");
        refresh();

        logStart = std::chrono::high_resolution_clock::now();
        auto lastLogFlush = logStart;
        if (logFile.has_value())
        {
            *logFile << "# -------- SSM Logger --------\n"
                        "# Start time (UTC): " << GetTimestamps(logStart).utc << "\n";

            if (!logFileExisted)
            {
                // CSV columns description:
                *logFile << "Timestamp;Timestamp (UTC);Milliseconds since start;Seeing (arc sec);Input value (V)\n";
            }

            mvprintw(POS_XY(Display::LogFile), "Log: %s", logFilePath.c_str());
        }
        else
            mvprintw(POS_XY(Display::LogFile), "Logging disabled");

        nodelay(stdscr, TRUE); // enable asynchronous mode of getch()
        noecho();

        while (true)
        {
            int ch;
            if (toupper(ch = getch()) == 'Q')
                break;

            boost::asio::streambuf inputBuf;

#if !SSM_IO_SIMULATION

            boost::asio::async_read_until(serialPort, inputBuf, '\n',
                                    [&](const boost::system::error_code &error, size_t numBytesXferred)
                                    {
                                        if (numBytesXferred > 0)
                                            timer.cancel();
                                        else
                                            ShowMessage("async_read() returned, but nothing was received");
                                    });

            timer.expires_from_now(boost::posix_time::milliseconds(5000));
            timer.async_wait([&](const boost::system::error_code& error)
                             {
                                 if (!error) // timer not canceled?
                                 {
                                    serialPort.cancel();
                                    timedOut = true;
                                 }
                             });

            io.run();
            io.reset();

#else
            // Simulate input

            std::this_thread::sleep_for(500ms);

            static const std::string inputStr = std::string(CommProtocol::Input)  + ": 0.95\n" +
                                                            CommProtocol::Seeing  + ": 1.23\n" +
                                                            CommProtocol::Samples + ": 2345\n";
            inputBuf.sputn(inputStr.c_str(), inputStr.length());

#endif

            if (!timedOut)
            {
                const auto tReceived = std::chrono::high_resolution_clock::now();

                std::istream is(&inputBuf);
                std::string line, valInput, valSeeing;

                while (!is.eof())
                {
                    std::getline(is, line);

                    if (line.substr(0, 2) == CommProtocol::Input)
                    {
                        valInput = line.substr(4);
                        PrintValue(Display::Input, "input", valInput + " V");
                    }
                    else if (line.substr(0, 2) == CommProtocol::Seeing)
                    {
                        valSeeing = line.substr(4);
                        PrintValue(Display::Seeing, "seeing", valSeeing + "\"");
                    }
                    else if (line.substr(0, 2) == CommProtocol::Samples)
                    {
                        PrintValue(Display::Samples, "samples", line.substr(4));
                    }
                }

                const auto timestamps = GetTimestamps(tReceived);

                if (logFile.has_value())
                {

                    if (!valInput.empty() || !valSeeing.empty())
                    {
                        *logFile << timestamps.local << ";"
                                 << timestamps.utc << ";"
                                 << std::chrono::duration_cast<std::chrono::milliseconds>(tReceived - logStart).count() << ";"
                                 << valSeeing << ";"
                                 << valInput  << "\n";
                    }

                    if (tReceived - lastLogFlush >= LOG_FLUSH_INTERVAL)
                    {
                        logFile->flush();
                        lastLogFlush = tReceived;
                    }
                }

                PrintValue(Display::Timestamp, "Timestamp (UTC)", timestamps.utc);

                refresh();
            }
            else
            {
                ncursesScope.EndNCurses();
                std::cout << "Timed out." << std::endl;
                return EXIT_SUCCESS;
            }
        }
    }
    catch (boost::system::system_error &ex)
    {

        std::cout << "Error: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

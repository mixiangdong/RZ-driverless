/******************************************************************************
 * Copyright 2020 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#pragma once
/*Common*/
#include <cstdint>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <math.h>
#include <memory>
#include <array>
#include <algorithm>
#include <functional>
#include <iterator>
#include <chrono>
#include <queue>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <future>
#include <stdexcept>
#include <mutex>
#include <type_traits>
#include <numeric>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <rs_driver/macro/version.h>
/*Linux*/
#ifdef __GNUC__
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif

/*Pcap*/
#include <pcap.h>

/*Camera*/
typedef std::pair<std::string, double> CameraTrigger;

/*Output style*/
#define RESET "\033[0m"
#define GREEN "\033[32m"               ///< Green
#define YELLOW "\033[33m"              ///< Yellow
#define BLUE "\033[34m"                ///< Blue
#define MAGENTA "\033[35m"             ///< Magenta
#define CYAN "\033[36m"                ///< Cyan
#define BOLDBLACK "\033[1m\033[30m"    ///< Bold Black
#define BOLDRED "\033[1m\033[31m"      ///< Bold Red
#define BOLDGREEN "\033[1m\033[32m"    ///< Bold Green
#define BOLDYELLOW "\033[1m\033[33m"   ///< Bold Yellow
#define BOLDBLUE "\033[1m\033[34m"     ///< Bold Blue
#define BOLDMAGENTA "\033[1m\033[35m"  ///< Bold Magenta
#define BOLDCYAN "\033[1m\033[36m"     ///< Bold Cyan
#define BOLDWHITE "\033[1m\033[37m"    ///< Bold White

#define INFOL (std::cout << GREEN)
#define INFO (std::cout << BOLDGREEN)
#define WARNING (std::cout << BOLDYELLOW)
#define ERROR (std::cout << BOLDRED)
#define DEBUG (std::cout << BOLDCYAN)
#define TITLE (std::cout << BOLDMAGENTA)
#define MSG (std::cout << BOLDWHITE)
#define END (std::endl)
#define REND "\033[0m" << std::endl

/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <fstream>
#include <vector>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "stringtoolbox.hpp"

#include "line.hpp"
#include "sensor.hpp"

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") 
      || 0 == commandlineArguments.count("frame-id") 
      || 0 == commandlineArguments.count("freq") 
      || 0 == commandlineArguments.count("map-file") 
      || 0 == commandlineArguments.count("x") 
      || 0 == commandlineArguments.count("y") 
      || 0 == commandlineArguments.count("yaw")) {
    std::cerr << argv[0] << " simulates an SRF02 ultrasonic distance sensor." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --frame-id=<The frame to use for position> --freq=<Simulation frequency> --map-file=<File where walls are defined as rows according to x1,y1,x2,y2;> --x=<X offset of the sensor> --y=<Y offset of the sensor> --yaw=<yaw angle of the sensor> --cid=<OpenDaVINCI session> [--id=<ID if more than one sensor>] [--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --map-file=/opt/arena.map --x=0.1 --y=0.0 --yaw=0.0 --freq=10 --cid=111" << std::endl;
    retCode = 1;
  } else {
    uint32_t const ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};

    std::vector<Line> walls;
    std::ifstream input(commandlineArguments["map-file"]);
    for (std::string str; getline(input, str);) {
      std::vector<std::string> coordinates = stringtoolbox::split(
          stringtoolbox::split(stringtoolbox::trim(str), ';')[0], ',');
      if (coordinates.size() == 4) {
        double x1{std::stof(coordinates[0])};
        double y1{std::stof(coordinates[1])};
        double x2{std::stof(coordinates[2])};
        double y2{std::stof(coordinates[3])};
        Line line{x1, y1, x2, y2};
        walls.push_back(line);
      }
    }

    uint32_t const FRAME_ID{static_cast<uint32_t>(std::stoi(commandlineArguments["frame-id"]))};
    double const X{static_cast<double>(std::stof(commandlineArguments["x"]))};
    double const Y{static_cast<double>(std::stof(commandlineArguments["y"]))};
    double const YAW{static_cast<double>(std::stof(commandlineArguments["yaw"]))};

    Sensor sensor{walls, X, Y, YAW};

    auto onEnvelope{[&FRAME_ID, &sensor](cluon::data::Envelope &&envelope)
      {
        if (envelope.dataType() == opendlv::sim::Frame::ID() 
            && envelope.senderStamp() == FRAME_ID) {
          auto frame = cluon::extractMessage<opendlv::sim::Frame>(std::move(envelope));
          sensor.setFrame(frame);
        }
      }};

    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"])), onEnvelope};

    double dt = 1.0 / std::stoi(commandlineArguments["freq"]);
    while (od4.isRunning()) {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));

      auto voltageReading = sensor.step();

      cluon::data::TimeStamp sampleTime;
      od4.send(voltageReading, sampleTime, ID);
      if (VERBOSE) {
        std::cout << "Sensor reading is " << voltageReading.voltage() << std::endl;
      }
    }
  }
  return retCode;
}

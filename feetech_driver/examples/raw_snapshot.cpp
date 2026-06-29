#include <feetech_driver/SMS_STS.h>
#include <feetech_driver/common.hpp>
#include <feetech_driver/communication_protocol.hpp>
#include <feetech_driver/serial_port.hpp>
#include <fmt/format.h>

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace feetech_driver;

namespace {

std::vector<uint8_t> parse_ids(const std::string& csv) {
  std::vector<uint8_t> ids;
  std::stringstream stream(csv);
  std::string token;
  while (std::getline(stream, token, ',')) {
    if (token.empty()) {
      continue;
    }
    const int value = std::stoi(token);
    if (value < 0 || value > static_cast<int>(kMaxServoId)) {
      throw std::out_of_range(fmt::format("servo id {} out of range", value));
    }
    ids.push_back(static_cast<uint8_t>(value));
  }
  if (ids.empty()) {
    throw std::invalid_argument("no servo ids provided");
  }
  return ids;
}

std::string json_escape(const std::string& input) {
  std::string out;
  out.reserve(input.size());
  for (char c : input) {
    switch (c) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      default:
        out += c;
        break;
    }
  }
  return out;
}

struct Sample {
  double monotonic_time;
  std::vector<std::array<uint8_t, 4>> data;
};

std::vector<Sample> collect_samples(CommunicationProtocol& protocol,
                                    const std::vector<uint8_t>& ids,
                                    const std::optional<double> duration_sec,
                                    const double period_sec) {
  std::vector<Sample> samples;
  if (!duration_sec.has_value()) {
    Sample sample{};
    sample.monotonic_time = 0.0;
    auto read_result = protocol.sync_read(ids, SMS_STS_PRESENT_POSITION_L, &sample.data);
    if (!read_result) {
      throw std::runtime_error(read_result.error());
    }
    samples.push_back(std::move(sample));
    return samples;
  }

  const auto start = std::chrono::steady_clock::now();
  const auto deadline = start + std::chrono::duration<double>(*duration_sec);
  std::size_t sample_index = 0;
  while (true) {
    const auto now = std::chrono::steady_clock::now();
    if (now > deadline && sample_index > 0) {
      break;
    }

    Sample sample{};
    sample.monotonic_time = std::chrono::duration<double>(now - start).count();
    auto read_result = protocol.sync_read(ids, SMS_STS_PRESENT_POSITION_L, &sample.data);
    if (!read_result) {
      throw std::runtime_error(read_result.error());
    }
    samples.push_back(std::move(sample));
    sample_index++;

    const auto after_read = std::chrono::steady_clock::now();
    if (after_read >= deadline) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(period_sec));
  }

  return samples;
}

void write_present_block(std::ofstream& output, const std::vector<uint8_t>& ids, const std::vector<std::array<uint8_t, 4>>& data) {
  output << "    \"present\": {\n";
  for (std::size_t i = 0; i < ids.size(); ++i) {
    const int raw_position = from_sts(WordBytes{.low = data[i][0], .high = data[i][1]});
    const int raw_speed = from_sts(WordBytes{.low = data[i][2], .high = data[i][3]});
    const int centered_ticks = raw_position - kStsMidpoint;
    output << fmt::format(
        "      \"{}\": {{\"raw_position\": {}, \"centered_ticks\": {}, \"radians_from_midpoint\": {:.17g}, "
        "\"raw_speed\": {}, \"decoded_speed_ticks\": {}}}",
        static_cast<int>(ids[i]),
        raw_position,
        centered_ticks,
        to_radians(centered_ticks),
        raw_speed,
        decode_sign_magnitude(raw_speed, SMS_STS_SIGN_BIT_VELOCITY));
    if (i + 1 < ids.size()) {
      output << ",";
    }
    output << "\n";
  }
  output << "    }";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4 && argc != 6) {
    std::cerr << "Usage: " << argv[0]
              << " <port> <comma-separated-ids> <output-json> [<duration-sec> <period-sec>]\n";
    return EXIT_FAILURE;
  }

  const std::string port = argv[1];
  const std::vector<uint8_t> ids = parse_ids(argv[2]);
  const std::string output_path = argv[3];
  std::optional<double> duration_sec;
  double period_sec = 0.0;
  if (argc == 6) {
    duration_sec = std::stod(argv[4]);
    period_sec = std::stod(argv[5]);
    if (*duration_sec <= 0.0) {
      throw std::invalid_argument("duration must be > 0");
    }
    if (period_sec <= 0.0) {
      throw std::invalid_argument("period must be > 0");
    }
  }

  auto serial_port = std::make_unique<SerialPort>(port);
  auto configure_result = serial_port->configure().and_then([&] { return serial_port->open(); });
  if (!configure_result) {
    throw std::runtime_error(configure_result.error());
  }

  CommunicationProtocol protocol(std::move(serial_port));
  const std::vector<Sample> samples = collect_samples(protocol, ids, duration_sec, period_sec);

  std::ofstream output(output_path);
  if (!output.is_open()) {
    throw std::runtime_error(fmt::format("failed to open {}", output_path));
  }

  output << "{\n";
  output << "  \"port\": \"" << json_escape(port) << "\",\n";
  output << "  \"ids\": [";
  for (std::size_t i = 0; i < ids.size(); ++i) {
    if (i > 0) {
      output << ", ";
    }
    output << static_cast<int>(ids[i]);
  }
  output << "]";
  if (!duration_sec.has_value()) {
    output << ",\n";
    write_present_block(output, ids, samples.front().data);
    output << "\n";
  } else {
    output << ",\n";
    output << fmt::format("  \"duration_sec\": {},\n", *duration_sec);
    output << fmt::format("  \"period_sec\": {},\n", period_sec);
    output << "  \"samples\": [\n";
    for (std::size_t i = 0; i < samples.size(); ++i) {
      output << "    {\n";
      output << fmt::format("      \"sample_index\": {},\n", i);
      output << fmt::format("      \"monotonic_time\": {:.17g},\n", samples[i].monotonic_time);
      write_present_block(output, ids, samples[i].data);
      output << "\n    }";
      if (i + 1 < samples.size()) {
        output << ",";
      }
      output << "\n";
    }
    output << "  ]\n";
  }
  output << "}\n";

  return EXIT_SUCCESS;
}

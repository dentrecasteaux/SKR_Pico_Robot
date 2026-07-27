#pragma once

#include <Arduino.h>

class SerialTransport
{
public:
  enum class ReadResult
  {
    None,
    Line,
    LineTooLong
  };

  explicit SerialTransport(Stream& stream);

  ReadResult readLine(const char*& line);
  Print& output();

private:
  static constexpr size_t LINE_BUFFER_SIZE = 160;
  static constexpr size_t MAX_BYTES_PER_READ = 64;

  Stream& stream_;
  char lineBuffer_[LINE_BUFFER_SIZE] = {};
  size_t lineLength_ = 0;
  bool discardingLongLine_ = false;
};

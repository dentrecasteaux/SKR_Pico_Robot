#include "SerialTransport.h"

SerialTransport::SerialTransport(Stream& stream) : stream_(stream) {}

SerialTransport::ReadResult SerialTransport::readLine(const char*& line)
{
  line = nullptr;

  size_t bytesRead = 0;
  while (stream_.available() > 0 && bytesRead < MAX_BYTES_PER_READ) {
    const char received = static_cast<char>(stream_.read());
    ++bytesRead;

    if (received == '\r') {
      continue;
    }

    if (received == '\n') {
      if (discardingLongLine_) {
        discardingLongLine_ = false;
        lineLength_ = 0;
        return ReadResult::LineTooLong;
      }

      lineBuffer_[lineLength_] = '\0';
      lineLength_ = 0;
      line = lineBuffer_;
      return ReadResult::Line;
    }

    if (discardingLongLine_) {
      continue;
    }

    if (lineLength_ < LINE_BUFFER_SIZE - 1) {
      lineBuffer_[lineLength_++] = received;
    } else {
      discardingLongLine_ = true;
    }
  }

  return ReadResult::None;
}

Print& SerialTransport::output() { return stream_; }

//
// Created by Rafael Günther on 23.10.25.
//

#include "bus_interface.h"

#include <algorithm>

Bus_error Bus_interface::read_words(const Bus_target& target, uint16_t* data,
                                    std::size_t length) const
{
  if (!data || length == 0)
    return Bus_error::invalid_arg;

  // Big-endian on the wire: [MSB][LSB]
  constexpr std::size_t BYTES_PER_WORD = 2;
  constexpr std::size_t CHUNK_WORDS = 16;

  uint8_t buf[CHUNK_WORDS*BYTES_PER_WORD];

  std::size_t done = 0;
  while (done < length)
  {
    const std::size_t w = std::min(length - done, CHUNK_WORDS);
    const std::size_t num_bytes = w * BYTES_PER_WORD;

    const Bus_error e = read_bytes(target, buf, num_bytes);

    if (e != Bus_error::ok)
      return e;

    for (std::size_t i=0; i<w; i++)
      data[done + i] = buf[i*2]<<8 | buf[i*2+1];

    done += w;
  }

  return Bus_error::ok; // TODO: Implement error handling
}

Bus_error Bus_interface::write_words(const Bus_target& target, const uint16_t* data,
                                     std::size_t len) const
{
  if (!data || len == 0)
    return Bus_error::invalid_arg;

  // Big-endian on the wire: [MSB][LSB]
  constexpr std::size_t BYTES_PER_WORD = 2;
  constexpr std::size_t CHUNK_WORDS = 16;

  uint8_t buf[CHUNK_WORDS*BYTES_PER_WORD];

  std::size_t done = 0;
  while (done < len)
  {
    const std::size_t w = std::min(len - done, CHUNK_WORDS);
    const std::size_t num_bytes = w * BYTES_PER_WORD;

    for (std::size_t i=0; i<w; i++)
    {
      buf[i*2] = data[done+i] >> 8;
      buf[i*2+1] = data[done+i] & 0xFF;
    }

    const Bus_error e = write_bytes(target, buf, num_bytes);

    if (e != Bus_error::ok)
      return e;

    done += w;
  }

  return Bus_error::ok; // TODO: Implement error handling
}


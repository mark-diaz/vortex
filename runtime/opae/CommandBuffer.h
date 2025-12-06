// Copyright © 2019-2023
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __COMMAND_BUFFER_H__
#define __COMMAND_BUFFER_H__

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

class CommandBuffer {
public:

  struct CmdHeader {
    uint32_t cmd_type;
  };

  CommandBuffer(uint8_t* base, size_t capacity);

  bool push_command(uint32_t cmd_type, const void* payload, size_t payload_size);

  size_t get_bytes_written() const;
  size_t get_cmd_count() const;
  size_t get_block_count() const;
  size_t get_flush_count() const;

  size_t get_flush_base_addr() const;

  void set_prev_flush_cmd_count();
  size_t get_prev_flush_cmd_count() const;

  void reset_block_count();
  void increment_flush_count();

private:
  bool write_bytes(const void* src, size_t len);
  size_t get_free_space() const;

private:
  uint8_t* base_addr_;
  size_t   capacity_;
  size_t   tail_;
  size_t   bytes_written_;

  size_t   cmd_count_;
  size_t   block_count_;
  size_t   block_offset_;

  size_t   flush_count_;
  size_t   prev_flush_cmd_count_;
};

#endif // __COMMAND_BUFFER_H__
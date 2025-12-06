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

#include "CommandBuffer.h"
#include <common.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

// TODO:
// - add logic to make sure that SW doesn't overwrite not processed yet data
// - add logic to make sure flush_count is never greater than 16
CommandBuffer::CommandBuffer(uint8_t* base, size_t capacity)
  : base_addr_(base)
  , capacity_(capacity)
  , tail_(0)
  , bytes_written_(0)
  , cmd_count_(0)
  , block_count_(0)
  , block_offset_(0)
  , flush_count_(0)
  , prev_flush_cmd_count_(0)
{}


bool CommandBuffer::push_command(uint32_t cmd_type, const void* payload, size_t payload_size) {
  CmdHeader hdr = { cmd_type };
  size_t total = sizeof(CmdHeader) + payload_size;

  fprintf(stdout,
      "[CMD BUFFER] push_command:\n"
      "  cmd_type     = 0x%x\n"
      "  payload_size = %zu\n"
      "  total_size   = %zu\n"
      "  block_offset = %zu (before)\n"
      "  block_count  = %zu (before)\n"
      "  tail         = %zu\n",
      cmd_type, payload_size, total, block_offset_, block_count_, tail_
  );

  // If this command won't fit in the current block, pad + finalize block
  if (block_offset_ + total > CACHE_BLOCK_SIZE) {
    size_t pad = CACHE_BLOCK_SIZE - block_offset_;
    if (!write_bytes(nullptr, pad))
        return false;

    block_count_++;         // one full block completed
    block_offset_ = 0;      // start a new block
  }

  // Write header
  if (!write_bytes(&hdr, sizeof(CmdHeader)))
    return false;

  // Write payload
  if (!write_bytes(payload, payload_size))
    return false;

  block_offset_ += total;
  cmd_count_++;

  // If fully filled an entire block
  if (block_offset_ == CACHE_BLOCK_SIZE) {
    block_count_++;
    block_offset_ = 0;
  }

  return true;
}

// Getters ////////////////////////////////////////////////

size_t CommandBuffer::get_bytes_written() const {
  return bytes_written_;
}

size_t CommandBuffer::get_cmd_count() const {
  return cmd_count_;
}

size_t CommandBuffer::get_block_count() const {
  return block_count_;
}

size_t CommandBuffer::get_flush_count() const {
  return flush_count_;
}

size_t CommandBuffer::get_flush_base_addr() const {
  size_t bytes_since_flush = block_count_ * CACHE_BLOCK_SIZE + block_offset_;
  return (tail_ + capacity_ - bytes_since_flush) % capacity_;
}

void CommandBuffer::set_prev_flush_cmd_count() {
  prev_flush_cmd_count_ = cmd_count_;
}

size_t CommandBuffer::get_prev_flush_cmd_count() const {
  return prev_flush_cmd_count_;
}

void CommandBuffer::reset_block_count() {
    block_count_ = 0;
}

void CommandBuffer::increment_flush_count() {
  flush_count_++;
}

// Private Helpers ///////////////////////////////////////////

bool CommandBuffer::write_bytes(const void* src, size_t len) {
  if (len > get_free_space())
    return false;

  const uint8_t* src_ptr = reinterpret_cast<const uint8_t*>(src);

  for (size_t i = 0; i < len; ++i) {
    uint8_t byte = src_ptr ? src_ptr[i] : 0;
    base_addr_[(tail_ + i) % capacity_] = byte;
  }

  tail_ = (tail_ + len) % capacity_;
  bytes_written_ += len;
  return true;
}

size_t CommandBuffer::get_free_space() const {
  return capacity_ - bytes_written_;
}
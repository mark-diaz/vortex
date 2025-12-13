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

CommandBuffer::CommandBuffer(uint8_t* base, size_t capacity)
  : base_addr_(base)
  , capacity_(capacity)
  , tail_(0)
  , used_bytes_(0)
  , cmd_count_(0)
  , block_count_(0)
  , block_offset_(0)
  , flush_count_(0)
  , prev_flush_cmd_count_(0)
{}

bool CommandBuffer::enqueue(uint8_t cmd_type, const void* payload, size_t payload_size) {
  CmdHeader hdr = { cmd_type };
  size_t total = sizeof(CmdHeader) + payload_size;

  // if command won't fit in the current block, pad + finalize block
  if (block_offset_ + total > CACHE_BLOCK_SIZE) {
    size_t pad = CACHE_BLOCK_SIZE - block_offset_;
    if (!write_bytes(nullptr, pad))
        return false;

    block_count_++;
    block_offset_ = 0;
  }

  // write header
  if (!write_bytes(&hdr, sizeof(CmdHeader)))
    return false;

  // write payload
  if (!write_bytes(payload, payload_size))
    return false;

  block_offset_ += total;
  cmd_count_++;

  // fully filled an entire block
  if (block_offset_ == CACHE_BLOCK_SIZE) {
    block_count_++;
    block_offset_ = 0;
  }

  return true;
}

bool CommandBuffer::clear() {

  // reset command buffer state:
  tail_ = 0;
  used_bytes_ = 0;
  block_offset_ = 0;
  block_count_ = 0;
  prev_flush_cmd_count_ = 0;
  cmd_count_ = 0;  

  return true;
}

// getters ////////////////////////////////////////////////

size_t CommandBuffer::get_used_bytes() const {
  return used_bytes_;
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

size_t CommandBuffer::get_flush_start_offset() const {
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

// private helpers ///////////////////////////////////////////

bool CommandBuffer::write_bytes(const void* src, size_t len) {
  if (len > get_free_space())
    return false;

  const uint8_t* src_ptr = reinterpret_cast<const uint8_t*>(src);

  for (size_t i = 0; i < len; ++i) {
    uint8_t byte = src_ptr ? src_ptr[i] : 0;
    base_addr_[(tail_ + i) % capacity_] = byte;
  }

  tail_ = (tail_ + len) % capacity_;
  used_bytes_ += len;
  return true;
}

size_t CommandBuffer::get_free_space() const {
  return capacity_ - used_bytes_;
}
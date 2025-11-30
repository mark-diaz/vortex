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

#include <common.h>

#include "driver.h"

#include <vortex_afu.h>

#ifdef SCOPE
#include "scope.h"
#endif

#include <algorithm>
#include <assert.h>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <list>
#include <memory>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <unordered_map>
#include <uuid/uuid.h>

using namespace vortex;

#define CMD_MEM_READ            AFU_IMAGE_CMD_MEM_READ
#define CMD_MEM_WRITE           AFU_IMAGE_CMD_MEM_WRITE
#define CMD_RUN                 AFU_IMAGE_CMD_RUN
#define CMD_DCR_WRITE           AFU_IMAGE_CMD_DCR_WRITE
// COMMAND BUFFER: TODO: proposed extension - embedded write
// #define CMD_MEM_INLINE_WRITE    AFU_IMAGE_CMD_MEM_INLINE_WRITE

#define MMIO_CMD_TYPE    (AFU_IMAGE_MMIO_CMD_TYPE * 4)
#define MMIO_CMD_ARG0    (AFU_IMAGE_MMIO_CMD_ARG0 * 4)
#define MMIO_CMD_ARG1    (AFU_IMAGE_MMIO_CMD_ARG1 * 4)
#define MMIO_CMD_ARG2    (AFU_IMAGE_MMIO_CMD_ARG2 * 4)

// COMMAND BUFFER
#define MMIO_FLUSH  (AFU_IMAGE_MMIO_FLUSH * 4)
#define MMIO_HOST_RING_BUFFER_BASE_ADDR  (AFU_IMAGE_MMIO_HOST_RING_BUFFER_BASE_ADDR * 4)
#define MMIO_RING_BUFFER_WPTR (AFU_IMAGE_MMIO_RING_BUFFER_WPTR * 4)
#define MMIO_RING_BUFFER_RPTR (AFU_IMAGE_MMIO_RING_BUFFER_RPTR * 4)
#define MMIO_RING_BUFFER_NUM_CMD_REMAINING (AFU_IMAGE_MMIO_RING_BUFFER_NUM_CMD_REMAINING * 4)

#define MMIO_STATUS      (AFU_IMAGE_MMIO_STATUS * 4)
#define MMIO_DEV_CAPS    (AFU_IMAGE_MMIO_DEV_CAPS * 4)
#define MMIO_ISA_CAPS    (AFU_IMAGE_MMIO_ISA_CAPS * 4)
#define MMIO_SCOPE_READ  (AFU_IMAGE_MMIO_SCOPE_READ * 4)
#define MMIO_SCOPE_WRITE (AFU_IMAGE_MMIO_SCOPE_WRITE * 4)

#define STATUS_STATE_BITS 8

#define CHECK_HANDLE(handle, _expr, _cleanup)                                  \
  auto handle = _expr;                                                         \
  if (handle == nullptr) {                                                     \
    printf("[VXDRV] Error: '%s' returned NULL!\n", #_expr);                    \
    _cleanup                                                                   \
  }

#define CHECK_FPGA_ERR(_expr, _cleanup)                                        \
  do {                                                                         \
    auto err = _expr;                                                          \
    if (err == 0)                                                              \
      break;                                                                   \
    printf("[VXDRV] Error: '%s' returned %d, %s!\n", #_expr, (int)err,         \
           api_.fpgaErrStr(err));                                              \
    _cleanup                                                                   \
  } while (false)

///////////////////////////////////////////////////////////////////////////////

// COMMAND BUFFER:

// Per-command staging buffer record
struct StagingBuffer {
  uint64_t wsid;
  uint64_t ioaddr;
  uint8_t* ptr;
  uint64_t size;
};

// note: vortex.h: CACHE_BLOCK_SIZE = 64
class CommandBuffer {
public:

  struct CmdHeader {
    uint32_t cmd_type;  // 3-bit opcode, rest reserved
  };

  CommandBuffer(uint8_t* base, size_t capacity, size_t cache_block_size)
    : base_addr_(base)
    , capacity_(capacity)
    , cache_block_size_(cache_block_size)
    , head_(0)
    , tail_(0)
    , curr_offset_(0)
    , size_(0)
  {}

  bool push_command(uint32_t cmd_type, const void* payload, size_t payload_size) {
    CmdHeader hdr = { cmd_type };
    size_t total = sizeof(CmdHeader) + payload_size;

    // enforce "one command per cache block" rule
    if (curr_offset_ + total > cache_block_size_) {
      size_t pad = cache_block_size_ - curr_offset_;
      if (!write_bytes(nullptr, pad))  // zero pad
        return false;
      curr_offset_ = 0;
    }

    if (!write_bytes(&hdr, sizeof(CmdHeader)))
      return false;

    if (!write_bytes(payload, payload_size))
      return false;

    curr_offset_ += total;
    return true;
  }

  size_t used_space() const {
    return size_;
  }

  uint8_t* data() {
    return base_addr_;
  }

private:

  bool write_bytes(const void* src, size_t len) {
    if (len > free_space())
      return false;

    const uint8_t* p = reinterpret_cast<const uint8_t*>(src);

    for (size_t i = 0; i < len; ++i) {
      uint8_t v = p ? p[i] : 0;
      base_addr_[(tail_ + i) % capacity_] = v;
    }

    tail_ = (tail_ + len) % capacity_;
    size_ += len;
    return true;
  }

  size_t free_space() const {
    return capacity_ - size_;
  }

  uint8_t* base_addr_;   // Pinned Memory
  size_t capacity_;
  size_t cache_block_size_;
  size_t head_, tail_;
  size_t curr_offset_;
  size_t size_;
};

class vx_device {
public:
  vx_device()
    : fpga_(nullptr)
    , global_mem_(ALLOC_BASE_ADDR,
                  GLOBAL_MEM_SIZE - ALLOC_BASE_ADDR,
                  RAM_PAGE_SIZE,
                  CACHE_BLOCK_SIZE)
    // COMMAND BUFFER: initial testing
    , cmd_buffer_wsid_(0)
    , cmd_buffer_ptr_(nullptr)
    , cmd_buffer_ioaddr_(0)
    , cmd_buffer_(nullptr, 0, CACHE_BLOCK_SIZE)
  {}

  ~vx_device() {
  #ifdef SCOPE
    vx_scope_stop(this);
  #endif
    if (fpga_ != nullptr) {
      // Release all per-command staging buffers
      for (auto& sb : staging_buffers_) {
        api_.fpgaReleaseBuffer(fpga_, sb.wsid);
      }
      staging_buffers_.clear();
      // Deallocate Pinned Command Buffer
      if (cmd_buffer_wsid_ != 0) {
        api_.fpgaReleaseBuffer(fpga_, cmd_buffer_wsid_);
        cmd_buffer_wsid_ = 0;
      }
      api_.fpgaClose(fpga_);
    }
    drv_close();
  }

  int init() {
    fpga_token accel_token;
    fpga_properties filter;
    fpga_guid guid;
    uint32_t num_matches;

    memset(&api_, 0, sizeof(opae_drv_api_t));
    if (drv_init(&api_) != 0) {
      return -1;
    }

    // Set up a filter that will search for an accelerator
    CHECK_FPGA_ERR(api_.fpgaGetProperties(nullptr, &filter), {
      return -1;
    });

    CHECK_FPGA_ERR(api_.fpgaPropertiesSetObjectType(filter, FPGA_ACCELERATOR), {
      api_.fpgaDestroyProperties(&filter);
      return -1;
    });

    // Add the desired UUID to the filter
    std::string s_uuid(AFU_ACCEL_UUID);
    std::replace(s_uuid.begin(), s_uuid.end(), '_', '-');
    uuid_parse(s_uuid.c_str(), guid);
    CHECK_FPGA_ERR(api_.fpgaPropertiesSetGUID(filter, guid), {
      api_.fpgaDestroyProperties(&filter);
      return -1;
    });

    // Do the search across the available FPGA contexts
    CHECK_FPGA_ERR(api_.fpgaEnumerate(&filter, 1, &accel_token, 1, &num_matches), {
      api_.fpgaDestroyProperties(&filter);
      return -1;
    });

    // Not needed anymore
    CHECK_FPGA_ERR(api_.fpgaDestroyProperties(&filter), {
      api_.fpgaDestroyToken(&accel_token);
      return -1;
    });

    if (num_matches < 1) {
      fprintf(stderr, "[VXDRV] Error: accelerator %s not found!\n", AFU_ACCEL_UUID);
      api_.fpgaDestroyToken(&accel_token);
      return -1;
    }

    // Open accelerator
    CHECK_FPGA_ERR(api_.fpgaOpen(accel_token, &fpga_, 0), {
      api_.fpgaDestroyToken(&accel_token);
      return -1;
    });

    // Done with token
    CHECK_FPGA_ERR(api_.fpgaDestroyToken(&accel_token), {
      api_.fpgaClose(fpga_);
      return -1;
    });
    
    // COMMAND BUFFER:
    CHECK_FPGA_ERR(api_.fpgaPrepareBuffer(fpga_, CMD_BUFFER_CAPACITY, &cmd_buffer_ptr_, &cmd_buffer_wsid_, 0), {
      return -1;});

    // Get IO address that AFU uses for DMA
    CHECK_FPGA_ERR(api_.fpgaGetIOAddress(fpga_, cmd_buffer_wsid_, &cmd_buffer_ioaddr_), {
      api_.fpgaReleaseBuffer(fpga_, cmd_buffer_wsid_);
      cmd_buffer_wsid_ = 0;
      return -1;});

    // Inform AFU where host command buffer lives
    CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_HOST_RING_BUFFER_BASE_ADDR, cmd_buffer_ioaddr_), {
      return -1;});

    // Construct command buffer interface over pinned memory
    cmd_buffer_ = CommandBuffer(
      reinterpret_cast<uint8_t*>(cmd_buffer_ptr_),
      CMD_BUFFER_CAPACITY,
      CACHE_BLOCK_SIZE
    );

    // Debug message
    fprintf(stdout, "[VXDRV] Command Buffer allocated: CPU ptr=%p, IO=0x%lx, size=%lu bytes\n", cmd_buffer_ptr_, cmd_buffer_ioaddr_, CMD_BUFFER_CAPACITY);

    {
      // Load ISA CAPS
      CHECK_FPGA_ERR(api_.fpgaReadMMIO64(fpga_, 0, MMIO_ISA_CAPS, &isa_caps_), {
        api_.fpgaClose(fpga_);
        return -1;
      });

      // Load device CAPS
      CHECK_FPGA_ERR(api_.fpgaReadMMIO64(fpga_, 0, MMIO_DEV_CAPS, &dev_caps_), {
        api_.fpgaClose(fpga_);
        return -1;
      });

      // Determine global memory size
      uint64_t num_banks, bank_size;
      this->get_caps(VX_CAPS_NUM_MEM_BANKS, &num_banks);
      this->get_caps(VX_CAPS_MEM_BANK_SIZE, &bank_size);
      global_mem_size_ = num_banks * bank_size;
    }

  #ifdef SCOPE
    {
      scope_callback_t callback;
      callback.registerWrite = [](vx_device_h hdevice, uint64_t value) -> int {
        auto device = (vx_device *)hdevice;
        return device->api_.fpgaWriteMMIO64(device->fpga_, 0, MMIO_SCOPE_WRITE, value);
      };

      callback.registerRead = [](vx_device_h hdevice, uint64_t *value) -> int {
        auto device = (vx_device *)hdevice;
        return device->api_.fpgaReadMMIO64(device->fpga_, 0, MMIO_SCOPE_READ, value);
      };

      CHECK_ERR(vx_scope_start(&callback, this, -1, -1), {
        api_.fpgaClose(fpga_);
        return err;
      });
    }
  #endif
    return 0;
  }

  // COMMAND BUFFER: this is an example of sending a write to memory
  int send_ring_buffer_dummy() {
  // 1. Build a fake payload (fits inside one cache block)
  uint8_t payload[40];

  // First 24 bytes = DEADBEEF pattern
  for (int i = 0; i < 24; ++i) {
      static const uint8_t pattern[4] = {0xDE, 0xAD, 0xBE, 0xEF};
      payload[i] = pattern[i & 3];
  }

  // Last 16 bytes = incremental 0..15
  for (int i = 24; i < 40; ++i) {
      payload[i] = i - 24;
  }

  // 2. Write command + payload into pinned command buffer
  bool ok = cmd_buffer_.push_command(
      CMD_MEM_WRITE,   // dummy opcode
      payload,         // dummy payload
      sizeof(payload)
  );

  if (!ok) {
    fprintf(stderr, "[VXDRV] CommandBuffer: push_command failed\n");
    return -1;
  }

  // 3. Write WPTR so AFU sees the new command
  size_t wptr = cmd_buffer_.used_space();

  CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_RING_BUFFER_WPTR, 1), { return -1; });
  CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_RING_BUFFER_RPTR, 0), { return -1; });
  fprintf(stdout, "[VXDRV] Dummy command written: payload=40 bytes, wptr=%zu, base=0x%lx\n", wptr, cmd_buffer_ioaddr_);

  CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_FLUSH, 1 ), { return -1; });
  // 4. Debug

  return 0;
}

  int flush(uint64_t dev_addr, const void *host_ptr, uint64_t size) {
    if (!is_aligned(dev_addr, CACHE_BLOCK_SIZE))
      return -1;

    auto asize = aligned_size(size, CACHE_BLOCK_SIZE);

    if (dev_addr + asize > global_mem_size_)
      return -1;

    // ensure ready for new command
    if (this->ready_wait(VX_MAX_TIMEOUT) != 0)
      return -1;

    // Allocate fresh staging buffer for this command
    StagingBuffer sb;
    if (this->allocate_staging_for_command(asize, &sb) != 0)
      return -1;

    // Copy host data to new staging buffer
    memcpy(sb.ptr, host_ptr, size);

    auto ls_shift = (int)std::log2(CACHE_BLOCK_SIZE);

    CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_FLUSH, sb.ioaddr >> ls_shift), {
      return -1;
    });

    if (this->ready_wait(VX_MAX_TIMEOUT) != 0)
      return -1;

    return 0;
  }

  int get_caps(uint32_t caps_id, uint64_t * value) {
    uint64_t _value;
    switch (caps_id) {
    case VX_CAPS_VERSION:
      _value = (dev_caps_ >> 0) & 0xff;
      break;
    case VX_CAPS_NUM_THREADS:
      _value = (dev_caps_ >> 8) & 0xff;
      break;
    case VX_CAPS_NUM_WARPS:
      _value = (dev_caps_ >> 16) & 0xff;
      break;
    case VX_CAPS_NUM_CORES:
      _value = (dev_caps_ >> 24) & 0xffff;
      break;
    case VX_CAPS_CACHE_LINE_SIZE:
      _value = CACHE_BLOCK_SIZE;
      break;
    case VX_CAPS_GLOBAL_MEM_SIZE:
      _value = global_mem_size_;
      break;
    case VX_CAPS_LOCAL_MEM_SIZE:
      _value = 1ull << ((dev_caps_ >> 40) & 0xff);
      break;
    case VX_CAPS_ISA_FLAGS:
      _value = isa_caps_;
      break;
    case VX_CAPS_NUM_MEM_BANKS:
      _value = 1 << ((dev_caps_ >> 48) & 0x7);
      break;
    case VX_CAPS_MEM_BANK_SIZE:
      _value = 1ull << (20 + ((dev_caps_ >> 51) & 0x1f));
      break;
    default:
      fprintf(stderr, "[VXDRV] Error: invalid caps id: %d\n", caps_id);
      std::abort();
      return -1;
    }

    *value = _value;

    return 0;
  }

  int mem_alloc(uint64_t size, int flags, uint64_t *dev_addr) {
    uint64_t addr;
    CHECK_ERR(global_mem_.allocate(size, &addr), {
      return err;
    });
    CHECK_ERR(this->mem_access(addr, size, flags), {
      global_mem_.release(addr);
      return err;
    });
    *dev_addr = addr;
    return 0;
  }

  int mem_reserve(uint64_t dev_addr, uint64_t size, int flags) {
    CHECK_ERR(global_mem_.reserve(dev_addr, size), {
      return err;
    });
    CHECK_ERR(this->mem_access(dev_addr, size, flags), {
      global_mem_.release(dev_addr);
      return err;
    });
    return 0;
  }

  int mem_free(uint64_t dev_addr) {
    return global_mem_.release(dev_addr);
  }

  int mem_access(uint64_t /*dev_addr*/, uint64_t /*size*/, int /*flags*/) {
    return 0;
  }

  int mem_info(uint64_t * mem_free, uint64_t * mem_used) const {
    if (mem_free)
      *mem_free = global_mem_.free();
    if (mem_used)
      *mem_used = global_mem_.allocated();
    return 0;
  }

  int upload(uint64_t dev_addr, const void *host_ptr, uint64_t size) {
    // check alignment
    if (!is_aligned(dev_addr, CACHE_BLOCK_SIZE))
      return -1;

    auto asize = aligned_size(size, CACHE_BLOCK_SIZE);

    // bound checking
    if (dev_addr + asize > global_mem_size_)
      return -1;

    // ensure ready for new command
    if (this->ready_wait(VX_MAX_TIMEOUT) != 0)
      return -1;

    // Allocate fresh staging buffer for this command
    StagingBuffer sb;
    if (this->allocate_staging_for_command(asize, &sb) != 0)
      return -1;

    // Copy host data to new staging buffer
    memcpy(sb.ptr, host_ptr, size);

    auto ls_shift = (int)std::log2(CACHE_BLOCK_SIZE);

    // COMMAND BUFFER: Debug statements



    uint64_t arg0 = sb.ioaddr >> ls_shift;
    uint64_t arg1 = dev_addr        >> ls_shift;
    uint64_t arg2 = asize           >> ls_shift;
    fprintf(stdout, "[COMMAND BUFFER SW]  CMD_ARG0 (hex): 0x%016lx\n", arg0);
    fprintf(stdout, "[COMMAND BUFFER SW]  CMD_ARG1 (hex): 0x%016lx\n", arg1);
    fprintf(stdout, "[COMMAND BUFFER SW]  CMD_ARG2 (hex): 0x%016lx\n", arg2);
    fprintf(stdout, "[COMMAND BUFFER SW] ls_shift = %d\n", ls_shift);
    fprintf(stdout, "[COMMAND BUFFER SW] staging_ioaddr = 0x%lx\n", sb.ioaddr);
    
    // uint64_t arg0 = staging_ioaddr_;
    // uint64_t arg1 = dev_addr;
    // uint64_t arg2 = asize;
    // --- Build 24-byte payload ---
    uint8_t payload[24];

    // memcpy(payload + 0,  &CMD_MEM_WRITE, 8);
    memcpy(payload + 0,  &arg0, 8);
    memcpy(payload + 8,  &arg1, 8);
    memcpy(payload + 16, &arg2, 8);

    // --- Push command into command buffer ---
    if (!enqueue_command(CMD_MEM_WRITE, payload, sizeof(payload)))
      return -1;

    // if (!enqueue_command(CMD_MEM_WRITE, payload, sizeof(payload)))
    //   return -1;

    // --- Update write pointer for hardware ---
    // size_t wptr = cmd_buffer_.used_space();

    // CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_RING_BUFFER_WPTR, 1), {
    //     return -1;
    // });

    // CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_RING_BUFFER_NUM_CMD_REMAINING, 1), { return -1; });

    // fprintf(stdout, "[COMMAND BUFFER SW] Upload command enqueued\n");
    // flush_commands();
    // if (this->ready_wait(VX_MAX_TIMEOUT) != 0)
    //   return -1;


    return 0;
  }
  
  /*
  int upload(uint64_t dev_addr, const void *host_ptr, uint64_t size) {
    // check alignment
    if (!is_aligned(dev_addr, CACHE_BLOCK_SIZE))
      return -1;

    auto asize = aligned_size(size, CACHE_BLOCK_SIZE);

    // bound checking
    if (dev_addr + asize > global_mem_size_)
      return -1;

    // ensure ready for new command
    if (this->ready_wait(VX_MAX_TIMEOUT) != 0)
      return -1;

    if (this->ensure_staging(asize) != 0)
      return -1;

    // update staging buffer
    memcpy(staging_ptr_, host_ptr, size);

    auto ls_shift = (int)std::log2(CACHE_BLOCK_SIZE);

    // COMMAND BUFFER: Debug statements
    fprintf(stdout, "[DEBUG] staging io addr = %ld\n", staging_ioaddr_);
    fprintf(stdout, "[DEBUG] dev_addr = %ld\n", dev_addr);
    fprintf(stdout, "[DEBUG] asize = %ld\n", asize);
    fprintf(stdout, "[DEBUG] ls_shift = %d\n", ls_shift);

    CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_CMD_ARG0, staging_ioaddr_ >> ls_shift), {
      return -1;
    });

    CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_CMD_ARG1, dev_addr >> ls_shift), {
      return -1;
    });

    CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_CMD_ARG2, asize >> ls_shift), {
      return -1;
    });

    CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_CMD_TYPE, CMD_MEM_WRITE), {
      return -1;
    });

    // Wait for the write operation to finish
    if (this->ready_wait(VX_MAX_TIMEOUT) != 0)
      return -1;

    return 0;
  }
  */

  int download(void *host_ptr, uint64_t dev_addr, uint64_t size) {
    // check alignment
    if (!is_aligned(dev_addr, CACHE_BLOCK_SIZE))
      return -1;

    auto asize = aligned_size(size, CACHE_BLOCK_SIZE);

    // bound checking
    if (dev_addr + asize > global_mem_size_)
      return -1;

    // ensure ready for new command
    if (this->ready_wait(VX_MAX_TIMEOUT) != 0)
      return -1;

    // Allocate fresh staging buffer for this command
    StagingBuffer sb;
    if (this->allocate_staging_for_command(asize, &sb) != 0)
      return -1;

    auto ls_shift = (int)std::log2(CACHE_BLOCK_SIZE);

    CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_CMD_ARG0, sb.ioaddr >> ls_shift), {
      return -1;
    });
    CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_CMD_ARG1, dev_addr >> ls_shift), {
      return -1;
    });
    CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_CMD_ARG2, asize >> ls_shift), {
      return -1;
    });
    CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_CMD_TYPE, CMD_MEM_READ), {
      return -1;
    });

    // Wait for the write operation to finish
    if (this->ready_wait(VX_MAX_TIMEOUT) != 0)
      return -1;

    // read staging buffer
    memcpy(host_ptr, sb.ptr, size);

    return 0;
  }

  int start(uint64_t krnl_addr, uint64_t args_addr) {
    // set kernel info
    CHECK_ERR(this->dcr_write(VX_DCR_BASE_STARTUP_ADDR0, krnl_addr & 0xffffffff), {
      return err;
    });
    CHECK_ERR(this->dcr_write(VX_DCR_BASE_STARTUP_ADDR1, krnl_addr >> 32), {
      return err;
    });
    CHECK_ERR(this->dcr_write(VX_DCR_BASE_STARTUP_ARG0, args_addr & 0xffffffff), {
      return err;
    });
    CHECK_ERR(this->dcr_write(VX_DCR_BASE_STARTUP_ARG1, args_addr >> 32), {
      return err;
    });

    // start execution
    CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_CMD_TYPE, CMD_RUN), {
      return -1;
    });

    // clear mpm cache
    mpm_cache_.clear();

    return 0;
  }

  int ready_wait(uint64_t timeout) {
    std::unordered_map<uint32_t, std::stringstream> print_bufs;

    struct timespec sleep_time;
    sleep_time.tv_sec = 0;
    sleep_time.tv_nsec = 1000000;

    // to milliseconds
    uint64_t sleep_time_ms = (sleep_time.tv_sec * 1000) + (sleep_time.tv_nsec / 1000000);

    for (;;) {
      uint64_t status;
      CHECK_FPGA_ERR(api_.fpgaReadMMIO64(fpga_, 0, MMIO_STATUS, &status), {
        return -1;
      });

      // check for console data
      uint32_t cout_data = status >> STATUS_STATE_BITS;
      if (cout_data & 0x1) {
        // retrieve console data
        do {
          char cout_char = (cout_data >> 1) & 0xff;
          uint32_t cout_tid = (cout_data >> 9) & 0xff;
          auto &ss_buf = print_bufs[cout_tid];
          ss_buf << cout_char;
          if (cout_char == '\n') {
            std::cout << std::dec << "#" << cout_tid << ": " << ss_buf.str() << std::flush;
            ss_buf.str("");
          }
          CHECK_FPGA_ERR(api_.fpgaReadMMIO64(fpga_, 0, MMIO_STATUS, &status), {
            return -1;
          });
          cout_data = status >> STATUS_STATE_BITS;
        } while (cout_data & 0x1);
      }

      uint32_t state = status & ((1 << STATUS_STATE_BITS) - 1);

      if (0 == state || 0 == timeout) {
        for (auto &buf : print_bufs) {
          auto str = buf.second.str();
          if (!str.empty()) {
            std::cout << "#" << buf.first << ": " << str << std::endl;
          }
        }
        if (state != 0) {
          fprintf(stdout, "[VXDRV] ready-wait timed out: state=%d\n", state);
          return -1;
        }
        break;
      }

      nanosleep(&sleep_time, nullptr);
      timeout -= sleep_time_ms;
    };

    return 0;
  }

  int dcr_write(uint32_t addr, uint32_t value) {
    CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_CMD_ARG0, addr), {
      return -1;
    });
    CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_CMD_ARG1, value), {
      return -1;
    });
    CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_CMD_TYPE, CMD_DCR_WRITE), {
      return -1;
    });
    dcrs_.write(addr, value);
    return 0;
  }

  int dcr_read(uint32_t addr, uint32_t * value) const {
    return dcrs_.read(addr, value);
  }

  int mpm_query(uint32_t addr, uint32_t core_id, uint64_t * value) {
    uint32_t offset = addr - VX_CSR_MPM_BASE;
    if (offset > 31)
      return -1;
    if (mpm_cache_.count(core_id) == 0) {
      uint64_t mpm_mem_addr = IO_MPM_ADDR + core_id * 32 * sizeof(uint64_t);
      CHECK_ERR(this->download(mpm_cache_[core_id].data(), mpm_mem_addr, 32 * sizeof(uint64_t)), {
        return err;
      });
    }
    *value = mpm_cache_.at(core_id).at(offset);
    return 0;
  }

  int flush_commands() {
    CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_RING_BUFFER_NUM_CMD_REMAINING, 1), { return -1; });

    
    CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_FLUSH, 1), { return -1; });
    fprintf(stdout, "[COMMAND BUFFER SW] Finish writing to MMIO_FLUSH \n");
    return 0;
  }

private:

  // Command Buffer:
  bool enqueue_command(uint32_t cmd_type, const void* payload, size_t payload_size) {
    
    if (!cmd_buffer_.push_command(cmd_type, payload, payload_size))
        return false;

    // size_t wptr = cmd_buffer_.used_space();

    // TODO: change from 1 to wptr
    CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_RING_BUFFER_WPTR, 1), { return -1; });

    // MMIO_RING_BUFFER_NUM_CMD_REMAINING
    // CHECK_FPGA_ERR(api_.fpgaWriteMMIO64(fpga_, 0, MMIO_RING_BUFFER_NUM_CMD_REMAINING, 1), { return -1; });

    return true;
  }


  

  int allocate_staging_for_command(uint64_t size, StagingBuffer* sb_out) {
    StagingBuffer sb;
    sb.size = size;
    
    // Allocate new buffer for this command
    CHECK_FPGA_ERR(api_.fpgaPrepareBuffer(fpga_, size, (void **)&sb.ptr, &sb.wsid, 0), {
      return -1;
    });

    // Get the physical address of the buffer in the accelerator
    CHECK_FPGA_ERR(api_.fpgaGetIOAddress(fpga_, sb.wsid, &sb.ioaddr), {
      api_.fpgaReleaseBuffer(fpga_, sb.wsid);
      return -1;
    });
    
    fprintf(stdout, "[COMMAND BUFFER SW] allocate_staging_for_command: new staging @ ioaddr=0x%lx\n", sb.ioaddr);

    // Track for cleanup
    staging_buffers_.push_back(sb);
    *sb_out = sb;

    return 0;
  }

  opae_drv_api_t api_;
  fpga_handle fpga_;
  MemoryAllocator global_mem_;
  DeviceConfig dcrs_;
  uint64_t dev_caps_;
  uint64_t isa_caps_;
  uint64_t global_mem_size_;
  std::vector<StagingBuffer> staging_buffers_;
  std::unordered_map<uint32_t, std::array<uint64_t, 32>> mpm_cache_;

  // COMMAND BUFFER:
  uint64_t cmd_buffer_wsid_;
  void*    cmd_buffer_ptr_;
  uint64_t cmd_buffer_ioaddr_;
  CommandBuffer cmd_buffer_;
  static constexpr size_t CMD_BUFFER_CAPACITY = 1024 * 1024; // 1 MB
};

#include <callbacks.inc>
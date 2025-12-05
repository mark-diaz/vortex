#include <vx_spawn.h>
#include "common.h"
#include <vx_print.h>

void kernel_body(kernel_arg_t* __UNIFORM__ arg) {
	auto src0_ptr = reinterpret_cast<TYPE*>(arg->src0_addr);
	auto src1_ptr = reinterpret_cast<TYPE*>(arg->src1_addr);
	auto dst_ptr  = reinterpret_cast<TYPE*>(arg->dst_addr);
	
	// vx_printf("[KERNEL] src0_addr=0x%lx, src1_addr=0x%lx, dst_addr=0x%lx\n", 
	// 	arg->src0_addr, arg->src1_addr, arg->dst_addr);
	// vx_printf("[KERNEL] blockIdx.x=%d, src0_ptr[%d]=0x%x, src1_ptr[%d]=0x%x, sum=0x%x\n",
	// 	blockIdx.x, blockIdx.x, src0_ptr[blockIdx.x], blockIdx.x, src1_ptr[blockIdx.x], 
	// 	src0_ptr[blockIdx.x] + src1_ptr[blockIdx.x]);
	
	dst_ptr[blockIdx.x] = src0_ptr[blockIdx.x] + src1_ptr[blockIdx.x];
}

int main() {
	kernel_arg_t* arg = (kernel_arg_t*)csr_read(VX_CSR_MSCRATCH);
	return vx_spawn_threads(1, &arg->num_points, nullptr, (vx_kernel_func_cb)kernel_body, arg);
}

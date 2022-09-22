#include <CL/cl.hpp>
#include <fstream>

namespace opencl_utils {

static const cl_command_queue_properties queue_props = CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE;

cl::Platform getOpenCLPlatform(int opencl_platform);

std::vector<cl::Device> getOpenCLDevices(cl::Platform &platform);

size_t getWorkgroupSize(cl::Device &device);

cl::NDRange getWorkgroupRange(size_t workgroup_size);

cl::CommandQueue getCommandQueue(cl::Context &context, cl::Device &device);

void buildOpenCLProgram(std::string path_program, cl::Program &program, cl::Context &context,
                        std::vector<cl::Device> *devices, const std::string &opencl_kernel_options);

} // namespace opencl_utils
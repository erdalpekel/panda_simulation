#include <panda_safety/opencl_utils.h>

cl::Platform opencl_utils::getOpenCLPlatform(int opencl_platform) {
  std::vector<cl::Platform> platforms;
  cl::Platform::get(&platforms);
  if (platforms.size() == 0) {
    throw std::runtime_error("Could not find an OpenCL platform");
  }

  if (platforms.size() < opencl_platform) {
    throw std::runtime_error("Desired OpenCL platform not availbale");
  }

  return platforms[opencl_platform];
}

std::vector<cl::Device> opencl_utils::getOpenCLDevices(cl::Platform &platform) {
  std::string platform_name;
  platform.getInfo(CL_PLATFORM_NAME, &platform_name);

  std::vector<cl::Device> devices;
  cl_int success = platform.getDevices(CL_DEVICE_TYPE_GPU, &devices);

  return std::move(devices);
}

size_t opencl_utils::getWorkgroupSize(cl::Device &device) { return device.getInfo<CL_DEVICE_MAX_WORK_GROUP_SIZE>(); }

cl::NDRange opencl_utils::getWorkgroupRange(size_t workgroup_size) { return cl::NDRange(workgroup_size); }

cl::CommandQueue opencl_utils::getCommandQueue(cl::Context &context, cl::Device &device) {
  return cl::CommandQueue(context, device, queue_props);
}

void opencl_utils::buildOpenCLProgram(std::string path_program, cl::Program &program, cl::Context &context,
                                      std::vector<cl::Device> *devices, const std::string &opencl_kernel_options) {
  std::ifstream ifs(path_program);
  std::string source_str((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));
  program = cl::Program(context, cl::Program::Sources(1, std::make_pair(source_str.c_str(), source_str.length())));

  try {
    cl_int status_build = program.build(*devices, opencl_kernel_options.c_str());

    if (status_build != 0) {
      throw std::runtime_error("Could not build OpenCL program - status code: " + status_build);
    }
  } catch (const error_t &err) {
    throw std::runtime_error("Could not build OpenCL kernel - error code: " + path_program);
  }
}
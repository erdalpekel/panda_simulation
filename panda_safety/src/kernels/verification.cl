kernel void verification_kernel(
  volatile __global bool *verified_collision_return,
   __read_only image3d_t opencl_image) {
  
  int4 dimensions = get_image_dim(opencl_image);

  int3 index = (int3)((int)get_global_id(0), (int)get_global_id(1), (int)get_global_id(2));
  size_t result_array_index = 0;
  const sampler_t samplerA = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP | CLK_FILTER_NEAREST;
  int4 center_value = read_imagei(opencl_image, samplerA, (int4)(index.x, index.y, index.z, 0));
  if (center_value.s0 > 0) {
    int neighbours = 0;
    for (int i = -1; i < 2; i++) {
      for (int j = -1; j < 2; j++) {
        for (int k = -1; k < 2; k++) {
          if (j != 0 && k != 0 && i != 0) {
            int4 value = read_imagei(opencl_image, samplerA, (int4)(index.x + i, index.y + j, index.z + k, 0));
            if (value.s0 > 0) {
              neighbours += 1;
            }
          }
        }
      }
    }
    if (neighbours >= 1) {
      #ifdef VISUALIZE 
        int4 dimensions;
        dimensions = get_image_dim(opencl_image);
        result_array_index = (index.z * dimensions.s0 * dimensions.s1) + (index.y * dimensions.s0) + index.x;
        // printf("dimensions index result_index: %d %d %d %d %d %d %d \n", dimensions.s0, dimensions.s1, dimensions.s2, index.x, index.y, index.z, result_array_index);
      #endif
      
      verified_collision_return[result_array_index] = true;
    }
  }
}
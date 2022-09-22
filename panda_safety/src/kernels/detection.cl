#define JOINT_NUM (7)
#define LINK_NUM_FILTER (9)
#define MAX_TRAJECTORY_SIZE (50)
#define MESH_OFFSET (0.1f)
#define MESH_POINT_DISTANCE (0.03f)
#pragma OPENCL EXTENSION cl_khr_3d_image_writes : enable

float16 create_matrix_single_step(float alpha, float a, float d, float theta)
{
    float16 result = (float16)(
        cos(theta),                -sin(theta),                 0.0,           a,
        sin(theta) * cos(alpha),    cos(theta) * cos(alpha),   -sin(alpha),   -d * sin(alpha),
        sin(theta) * sin(alpha),    cos(theta) * sin(alpha),    cos(alpha),    d * cos(alpha),
        0.0,                        0.0,                        0.0,           1.0);
    return result;
}

float16 matrix_mult_4x4(float16 a, float16 b)
{
    return (float16) (  dot(a.s0123, b.s048c), dot(a.s0123, b.s159d), dot(a.s0123, b.s26ae), dot(a.s0123, b.s37bf),
                        dot(a.s4567, b.s048c), dot(a.s4567, b.s159d), dot(a.s4567, b.s26ae), dot(a.s4567, b.s37bf),
                        dot(a.s89ab, b.s048c), dot(a.s89ab, b.s159d), dot(a.s89ab, b.s26ae), dot(a.s89ab, b.s37bf),
                        dot(a.scdef, b.s048c), dot(a.scdef, b.s159d), dot(a.scdef, b.s26ae), dot(a.scdef, b.s37bf));
}

kernel void detection_kernel(
    __global float8 *points,
    volatile __global const float16 *transforms,
    __global const float4 *transform_camera,
    uint trajectory_size,
    __write_only image3d_t opencl_image,
    __global const float3 *mesh,
    __global const float *theta,
    const float3 maximum_reachable_box,
    const float3 minimum_reachable_box,
    __global const uint *num_vertices)
{
    __local float16 transforms_local[LINK_NUM_FILTER];
    __local float theta_local[MAX_TRAJECTORY_SIZE * JOINT_NUM];
    __local float3 mesh_local[NUM_MESH_VERTICES];
    __local uint num_vertices_local[LINK_NUM_FILTER];
    __local float3 d_h_parameter_local[JOINT_NUM + 1];

    if (get_local_id(0) == 0)
    {
        for (int i = 0; i < LINK_NUM_FILTER; ++i)
        {
            transforms_local[i] = transforms[i];
        }

        for (int i = 0; i < trajectory_size; ++i)
        {
            theta_local[i] = theta[i];
        }

        for (int i = 0; i < NUM_MESH_VERTICES; ++i)
        {
            mesh_local[i] = mesh[i];
        }

        for (int i = 0; i < LINK_NUM_FILTER; ++i)
        {
            num_vertices_local[i] = num_vertices[i];
        }
        d_h_parameter_local[0] = (float3)(JOINT_0_alpha, JOINT_0_a, JOINT_0_d);
        d_h_parameter_local[1] = (float3)(JOINT_1_alpha, JOINT_1_a, JOINT_1_d);
        d_h_parameter_local[2] = (float3)(JOINT_2_alpha, JOINT_2_a, JOINT_2_d);
        d_h_parameter_local[3] = (float3)(JOINT_3_alpha, JOINT_3_a, JOINT_3_d);
        d_h_parameter_local[4] = (float3)(JOINT_4_alpha, JOINT_4_a, JOINT_4_d);
        d_h_parameter_local[5] = (float3)(JOINT_5_alpha, JOINT_5_a, JOINT_5_d);
        d_h_parameter_local[6] = (float3)(JOINT_6_alpha, JOINT_6_a, JOINT_6_d);
        d_h_parameter_local[7] = (float3)(JOINT_7_alpha, JOINT_7_a, JOINT_7_d);
    }
    barrier(CLK_LOCAL_MEM_FENCE);

    size_t global_id = get_global_id(0);
    float3 point = (float3)(points[global_id].s012);
    bool point_in_robot = false;
    if (isnan(point.x) == 0)
    {
        float3 reachable_box_max = (float3)(maximum_reachable_box.x, maximum_reachable_box.y, maximum_reachable_box.z);
        float3 reachable_box_min = (float3)(minimum_reachable_box.x, minimum_reachable_box.y, minimum_reachable_box.z);
        float4 point_hom = (float4)(point.s012, 1.0f);
        float3 point_transformed = (float3)(dot(transform_camera[0], point_hom), dot(transform_camera[1], point_hom), dot(transform_camera[2], point_hom));
        
        if (point_transformed.x >= reachable_box_max.x || point_transformed.x < reachable_box_min.x ||
            point_transformed.y >= reachable_box_max.y || point_transformed.y < reachable_box_min.y ||
            point_transformed.z >= reachable_box_max.z || point_transformed.z < reachable_box_min.z)
        {
            return;
        }

        uint counter_array = 0;
        for (int link_counter = 0; link_counter < LINK_NUM_FILTER && !point_in_robot; ++link_counter)
        {
            for (int i = 0; i < num_vertices_local[link_counter] && !point_in_robot; ++i)
            {
                float4 vertex_hom = (float4)(mesh_local[counter_array].x, mesh_local[counter_array].y, mesh_local[counter_array].z, 1.0f);
                float3 vertex = (float3)(
                    dot(transforms_local[link_counter].s0123, vertex_hom),
                    dot(transforms_local[link_counter].s4567, vertex_hom),
                    dot(transforms_local[link_counter].s89ab, vertex_hom));
                float length_tmp = distance(vertex, point_transformed);
                if (length_tmp < MESH_OFFSET)
                {
                    point_in_robot = true;
                }
                ++counter_array;
            }
        }

        float4 object_center_hom = (float4)(0.0f, 0.0f, 0.20f, 1.0f);
        float3 object_center = (float3)(
            dot(transforms_local[LINK_NUM_FILTER - 1].s0123, object_center_hom),
            dot(transforms_local[LINK_NUM_FILTER - 1].s4567, object_center_hom),
            dot(transforms_local[LINK_NUM_FILTER - 1].s89ab, object_center_hom));
        if (distance(object_center, point_transformed) < 0.1f)
        {
            point_in_robot = true;
        }
        
        if (point_in_robot)
        {
            return;
        }

        bool collision_detected = false;
        uint counter = 0;
        float16 transform_matrix_local[JOINT_NUM + 2];
        float16 base_transform = transforms_local[0];
        
        transform_matrix_local[0] = base_transform;
        for (uint joint_index = 0; joint_index < JOINT_NUM; joint_index++)
        {
            float16 single_step = create_matrix_single_step(d_h_parameter_local[joint_index].x, 
                                                            d_h_parameter_local[joint_index].y, 
                                                            d_h_parameter_local[joint_index].z, 
                                                            theta_local[joint_index]);
            transform_matrix_local[joint_index + 1] = matrix_mult_4x4(transform_matrix_local[joint_index], single_step);
        }
        //hand transform
        float16 single_step = create_matrix_single_step(d_h_parameter_local[JOINT_NUM].x, 
                                                        d_h_parameter_local[JOINT_NUM].y, 
                                                        d_h_parameter_local[JOINT_NUM].z, 
                                                        0.0f);
        transform_matrix_local[JOINT_NUM + 1] = matrix_mult_4x4(transform_matrix_local[JOINT_NUM], single_step);

        //Begin trajectory collision checking
        //because the current state is not contained in the trajectory transforms
        for (uint trajectory_index = 0; trajectory_index < (trajectory_size - JOINT_NUM) && !collision_detected; trajectory_index = trajectory_index + JOINT_NUM)
        {
            //skip link0, as it can't move
            counter = num_vertices_local[0];
            uint next_trajectory_index = trajectory_index + JOINT_NUM;
            
            float16 transforms_next_temporary;
            //skip link0, as it can't move
            for (ushort link_index = 1; link_index < LINK_NUM_FILTER && !collision_detected; ++link_index)
            {
                float tmp_theta = 0.0f;
                if (link_index < JOINT_NUM + 1){
                    tmp_theta = theta_local[next_trajectory_index + link_index - 1];
                }
                float16 step = create_matrix_single_step( 
                    d_h_parameter_local[link_index - 1].x, 
                    d_h_parameter_local[link_index - 1].y, 
                    d_h_parameter_local[link_index - 1].z, 
                    tmp_theta);
                transforms_next_temporary = matrix_mult_4x4(transform_matrix_local[link_index-1], step);

                for (uint link_mesh_index = 0; link_mesh_index < num_vertices_local[link_index] && !collision_detected; ++link_mesh_index)
                {

                    float4 current_mesh_point = (float4)(mesh_local[counter].x, mesh_local[counter].y, mesh_local[counter].z, 1.0f);

                    float3 transformed_mesh_vertex = (float3)(
                        dot(transform_matrix_local[link_index].s0123, current_mesh_point),
                        dot(transform_matrix_local[link_index].s4567, current_mesh_point),
                        dot(transform_matrix_local[link_index].s89ab, current_mesh_point));

                    float3 next_transformed_mesh_vertex = (float3)(
                        dot(transforms_next_temporary.s0123, current_mesh_point),
                        dot(transforms_next_temporary.s4567, current_mesh_point),
                        dot(transforms_next_temporary.s89ab, current_mesh_point));
                    
                    if (distance(point_transformed, next_transformed_mesh_vertex) < MESH_POINT_DISTANCE)
                    {
                        collision_detected = true;
                    }
                    else if (distance(point_transformed, transformed_mesh_vertex) < MESH_POINT_DISTANCE)
                    {
                        collision_detected = true;
                    }
                    else
                    {
                        // check if point cloud point near infinite vector between mesh point and next position
                        float3 vector_ab = next_transformed_mesh_vertex - transformed_mesh_vertex;
                        float3 vector_ap = point_transformed - transformed_mesh_vertex;
                        float3 cross_product = cross(vector_ap, vector_ab);
                        float distance = fast_length(cross_product) / fast_length(vector_ab);

                        if (distance < MESH_POINT_DISTANCE)
                        {
                            //if near enough check if the point cloud point that is projected on this vector is at most the mesh distance away from line (mesh -> next position)
                            float3 vector_ad = dot(vector_ap, vector_ab) / dot(vector_ab, vector_ab) * vector_ab;
                            float3 projected_point = transformed_mesh_vertex + vector_ad;

                            float3 box_min = (float3)(
                                fmin(transformed_mesh_vertex.x, next_transformed_mesh_vertex.x),
                                fmin(transformed_mesh_vertex.y, next_transformed_mesh_vertex.y),
                                fmin(transformed_mesh_vertex.z, next_transformed_mesh_vertex.z));
                            float3 box_max = (float3)(
                                fmax(transformed_mesh_vertex.x, next_transformed_mesh_vertex.x),
                                fmax(transformed_mesh_vertex.y, next_transformed_mesh_vertex.y),
                                fmax(transformed_mesh_vertex.z, next_transformed_mesh_vertex.z));
                            if ((   box_min.x <= projected_point.x && projected_point.x <= box_max.x) 
                                && (box_min.y <= projected_point.y && projected_point.y <= box_max.y) 
                                && (box_min.z <= projected_point.z && projected_point.z <= box_max.z))
                            {
                                collision_detected = true;
                            }
                        }
                    }
                    
                    if (collision_detected)
                    {
                        int3 index;
                        int4 dimensions;

                        float3 point_cloud_box_dim = reachable_box_max - reachable_box_min;
                        float3 point_transformed_moved = point_transformed - reachable_box_min;
                        float3 point_transformed_normalized = point_transformed_moved / point_cloud_box_dim;
                        dimensions = get_image_dim(opencl_image);
                        index.x = (int)floor((point_transformed_normalized.x) * ((float)dimensions.s0));
                        index.y = (int)floor((point_transformed_normalized.y) * ((float)dimensions.s1));
                        index.z = (int)floor((point_transformed_normalized.z) * ((float)dimensions.s2));
                        write_imagei(opencl_image, (int4)(index.x, index.y, index.z, 0), (int4)(1, 1, 1, 1));

                        return;
                    }

                    counter++;
                }
                transform_matrix_local[link_index] = transforms_next_temporary;
            }
        }
    }
}

// Copyright 2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

// ---------------------------------------------------------------------------
// Note: We generate files vdb_topology_<level>.h from this 
//       template using CMake.
// ---------------------------------------------------------------------------

/*
 * The number of levels on this tree. This define will be the same
 * for all generated files.
 */
#ifndef VKL_VDB_NUM_LEVELS
# define VKL_VDB_NUM_LEVELS 4
#endif

/*
 * The actual topology configuration. These constants define the resolution
 * of nodes on the current level.
 */
#define VKL_VDB_LOG_RES_0       6
#define VKL_VDB_STORAGE_RES_0   64
#define VKL_VDB_NUM_VOXELS_0    262144
#define VKL_VDB_TOTAL_LOG_RES_0 18
#define VKL_VDB_RES_0           262144

// ---------------------------------------------------------------------------
// Recurse, or define sensible recursion stoppers.
// ---------------------------------------------------------------------------

#if (1 < 4)

#include "topology_1.h"

#else

#define VKL_VDB_LOG_RES_1       0
#define VKL_VDB_STORAGE_RES_1   1
#define VKL_VDB_NUM_VOXELS_1    1
#define VKL_VDB_TOTAL_LOG_RES_1 0
#define VKL_VDB_RES_1           1

#define VKL_VDB_LEAF_LEVEL         0
#define VKL_VDB_LOG_RES_LEAF       6
#define VKL_VDB_STORAGE_RES_LEAF   64
#define VKL_VDB_NUM_VOXELS_LEAF    262144
#define VKL_VDB_TOTAL_LOG_RES_LEAF 18
#define VKL_VDB_RES_LEAF           262144

#define __vkl_vdb_iterate_levels_1(Macro, ...)

#endif // 1 < VKL_VDB_NUM_LEVELS

// ---------------------------------------------------------------------------
// Apply a macro to all levels of the tree.
// ---------------------------------------------------------------------------

/* 
 * Force expansion of the argument.
 * This is required because Visual Studio will not expand __VA_ARGS__ when
 * passed to another macro on its own.
 */
#if !defined(__vkl_vdb_expand)
  #define __vkl_vdb_expand(A) A
#endif

/*
 * Apply a macro to all levels recursively.
 */
#define __vkl_vdb_iterate_levels_0(Macro, ...)                          \
  __vkl_vdb_expand(__vkl_vdb_iterate_levels_1(Macro, __VA_ARGS__)) \
  __vkl_vdb_expand(Macro(0, __VA_ARGS__))

// ---------------------------------------------------------------------------
// Map offsets to voxels and linear indices.
// ---------------------------------------------------------------------------

/*
 * Map a domain level offset (w.r.t. the root node) to a voxel index inside
 * a node on the current level.
 *
 * This macro operates on offsets in domain units, i.e. leaf voxel indices.
 * It first masks out the upper bits. This is essentially a modulo
 * operation, giving the offset inside a block of size VKL_VDB_RES_<level>.
 * This offset is still in domain units.
 *
 * |........|........|.x......|  --> |.x.....|
 *
 * It then divides by the next level domain resolution to get index of the 
 * voxel inside the current node that contains the offset.
 *
 * |.x......| -> (voxels have resolution 4) -> |x.|
 */
#define __vkl_vdb_domain_offset_to_voxel_uniform_0(offset)           \
  ((VKL_INTEROP_UNIFORM vkl_uint64) ((offset) & (VKL_VDB_RES_0 - 1)) \
    >> VKL_VDB_TOTAL_LOG_RES_1)

#if defined(ISPC)

#define __vkl_vdb_domain_offset_to_voxel_varying_0(offset) \
  ((varying vkl_uint64) ((offset) & (VKL_VDB_RES_0 - 1))   \
    >> VKL_VDB_TOTAL_LOG_RES_1)

#endif // defined(ISPC)


/*
 * Map a 3D index to a linear index for nodes on the given level.
 * Note: VDB data uses column major ordering!
 *
 * This is the normal (x * sliceSize + y * rowSize + z), but instead of
 * multiplying by VKL_VDB_RES_<level> (rowSize) and (VKL_VDB_RES_<level>)^2
 * (sliceSize), we perform shifts with the base-2 logarithms.
 */
#define __vkl_vdb_3d_to_linear_uniform_0(offx, offy, offz)              \
   ((((VKL_INTEROP_UNIFORM vkl_uint64)offx) << (2 * VKL_VDB_LOG_RES_0)) \
  + (((VKL_INTEROP_UNIFORM vkl_uint64)offy) <<      VKL_VDB_LOG_RES_0 ) \
  + (((VKL_INTEROP_UNIFORM vkl_uint64)offz)))

#if defined(ISPC)

#define __vkl_vdb_3d_to_linear_varying_0(offx, offy, offz)  \
   ((((varying vkl_uint64)offx) << (2 * VKL_VDB_LOG_RES_0)) \
  + (((varying vkl_uint64)offy) <<      VKL_VDB_LOG_RES_0 ) \
  + (((varying vkl_uint64)offz)))

#endif

/*
 * Map a 3D domain offset to a linear voxel index.
 * This macro is here for convenience as it simply combines the above two
 * macros.
 */
#define __vkl_vdb_domain_offset_to_linear_uniform_0(offx, offy, offz) \
  __vkl_vdb_3d_to_linear_uniform_0(                                   \
    __vkl_vdb_domain_offset_to_voxel_uniform_0(offx),                 \
    __vkl_vdb_domain_offset_to_voxel_uniform_0(offy),                 \
    __vkl_vdb_domain_offset_to_voxel_uniform_0(offz))

#if !(1 < 4)

#define __vkl_vdb_domain_offset_to_linear_uniform_leaf(offx, offy, offz)            \
  __vkl_vdb_3d_to_linear_uniform_0(                                   \
    __vkl_vdb_domain_offset_to_voxel_uniform_0(offx),                 \
    __vkl_vdb_domain_offset_to_voxel_uniform_0(offy),                 \
    __vkl_vdb_domain_offset_to_voxel_uniform_0(offz))

#endif

#if defined(ISPC)

#define __vkl_vdb_domain_offset_to_linear_varying_0(offx, offy, offz) \
  __vkl_vdb_3d_to_linear_varying_0(                                   \
    __vkl_vdb_domain_offset_to_voxel_varying_0(offx),                 \
    __vkl_vdb_domain_offset_to_voxel_varying_0(offy),                 \
    __vkl_vdb_domain_offset_to_voxel_varying_0(offz))

#if !(1 < 4)

#define __vkl_vdb_domain_offset_to_linear_varying_leaf(offx, offy, offz)            \
  __vkl_vdb_3d_to_linear_varying_0(                                   \
    __vkl_vdb_domain_offset_to_voxel_varying_0(offx),                 \
    __vkl_vdb_domain_offset_to_voxel_varying_0(offy),                 \
    __vkl_vdb_domain_offset_to_voxel_varying_0(offz))

#endif

#endif


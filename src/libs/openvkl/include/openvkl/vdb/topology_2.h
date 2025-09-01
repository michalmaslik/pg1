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
#define VKL_VDB_LOG_RES_2       4
#define VKL_VDB_STORAGE_RES_2   16
#define VKL_VDB_NUM_VOXELS_2    4096
#define VKL_VDB_TOTAL_LOG_RES_2 7
#define VKL_VDB_RES_2           128

// ---------------------------------------------------------------------------
// Recurse, or define sensible recursion stoppers.
// ---------------------------------------------------------------------------

#if (3 < 4)

#include "topology_3.h"

#else

#define VKL_VDB_LOG_RES_3       0
#define VKL_VDB_STORAGE_RES_3   1
#define VKL_VDB_NUM_VOXELS_3    1
#define VKL_VDB_TOTAL_LOG_RES_3 0
#define VKL_VDB_RES_3           1

#define VKL_VDB_LEAF_LEVEL         2
#define VKL_VDB_LOG_RES_LEAF       4
#define VKL_VDB_STORAGE_RES_LEAF   16
#define VKL_VDB_NUM_VOXELS_LEAF    4096
#define VKL_VDB_TOTAL_LOG_RES_LEAF 7
#define VKL_VDB_RES_LEAF           128

#define __vkl_vdb_iterate_levels_3(Macro, ...)

#endif // 3 < VKL_VDB_NUM_LEVELS

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
#define __vkl_vdb_iterate_levels_2(Macro, ...)                          \
  __vkl_vdb_expand(__vkl_vdb_iterate_levels_3(Macro, __VA_ARGS__)) \
  __vkl_vdb_expand(Macro(2, __VA_ARGS__))

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
#define __vkl_vdb_domain_offset_to_voxel_uniform_2(offset)           \
  ((VKL_INTEROP_UNIFORM vkl_uint64) ((offset) & (VKL_VDB_RES_2 - 1)) \
    >> VKL_VDB_TOTAL_LOG_RES_3)

#if defined(ISPC)

#define __vkl_vdb_domain_offset_to_voxel_varying_2(offset) \
  ((varying vkl_uint64) ((offset) & (VKL_VDB_RES_2 - 1))   \
    >> VKL_VDB_TOTAL_LOG_RES_3)

#endif // defined(ISPC)


/*
 * Map a 3D index to a linear index for nodes on the given level.
 * Note: VDB data uses column major ordering!
 *
 * This is the normal (x * sliceSize + y * rowSize + z), but instead of
 * multiplying by VKL_VDB_RES_<level> (rowSize) and (VKL_VDB_RES_<level>)^2
 * (sliceSize), we perform shifts with the base-2 logarithms.
 */
#define __vkl_vdb_3d_to_linear_uniform_2(offx, offy, offz)              \
   ((((VKL_INTEROP_UNIFORM vkl_uint64)offx) << (2 * VKL_VDB_LOG_RES_2)) \
  + (((VKL_INTEROP_UNIFORM vkl_uint64)offy) <<      VKL_VDB_LOG_RES_2 ) \
  + (((VKL_INTEROP_UNIFORM vkl_uint64)offz)))

#if defined(ISPC)

#define __vkl_vdb_3d_to_linear_varying_2(offx, offy, offz)  \
   ((((varying vkl_uint64)offx) << (2 * VKL_VDB_LOG_RES_2)) \
  + (((varying vkl_uint64)offy) <<      VKL_VDB_LOG_RES_2 ) \
  + (((varying vkl_uint64)offz)))

#endif

/*
 * Map a 3D domain offset to a linear voxel index.
 * This macro is here for convenience as it simply combines the above two
 * macros.
 */
#define __vkl_vdb_domain_offset_to_linear_uniform_2(offx, offy, offz) \
  __vkl_vdb_3d_to_linear_uniform_2(                                   \
    __vkl_vdb_domain_offset_to_voxel_uniform_2(offx),                 \
    __vkl_vdb_domain_offset_to_voxel_uniform_2(offy),                 \
    __vkl_vdb_domain_offset_to_voxel_uniform_2(offz))

#if !(3 < 4)

#define __vkl_vdb_domain_offset_to_linear_uniform_leaf(offx, offy, offz)            \
  __vkl_vdb_3d_to_linear_uniform_2(                                   \
    __vkl_vdb_domain_offset_to_voxel_uniform_2(offx),                 \
    __vkl_vdb_domain_offset_to_voxel_uniform_2(offy),                 \
    __vkl_vdb_domain_offset_to_voxel_uniform_2(offz))

#endif

#if defined(ISPC)

#define __vkl_vdb_domain_offset_to_linear_varying_2(offx, offy, offz) \
  __vkl_vdb_3d_to_linear_varying_2(                                   \
    __vkl_vdb_domain_offset_to_voxel_varying_2(offx),                 \
    __vkl_vdb_domain_offset_to_voxel_varying_2(offy),                 \
    __vkl_vdb_domain_offset_to_voxel_varying_2(offz))

#if !(3 < 4)

#define __vkl_vdb_domain_offset_to_linear_varying_leaf(offx, offy, offz)            \
  __vkl_vdb_3d_to_linear_varying_2(                                   \
    __vkl_vdb_domain_offset_to_voxel_varying_2(offx),                 \
    __vkl_vdb_domain_offset_to_voxel_varying_2(offy),                 \
    __vkl_vdb_domain_offset_to_voxel_varying_2(offz))

#endif

#endif


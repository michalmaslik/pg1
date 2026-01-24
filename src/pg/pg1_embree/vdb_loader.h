#ifndef VDB_LOADER_H_
#define VDB_LOADER_H_

#include <openvdb/openvdb.h>
#include <openvkl/openvkl.h>
#include <vector>
#include <string>
#include "vector3.h"

/*! \class VdbLoader
\brief Handles loading OpenVDB files and converting them to OpenVKL volumes.

This class provides functionality to load .vdb files using OpenVDB library
and convert the data to OpenVKL VDB volumes for volumetric rendering.
*/
class VdbLoader {
public:
    //! Constructor
    VdbLoader();

    //! Destructor
    ~VdbLoader();

    //! Load VDB file and create OpenVKL volume
    /*!
    \param filename Path to the .vdb file
    \param gridName Name of the grid to load from the file
    \param device OpenVKL device to use
    \return OpenVKL volume handle, or nullptr on failure
    */
    VKLVolume LoadVdbFile(const std::string& filename, const std::string& gridName, VKLDevice device);

    //! Get the bounding box of the loaded volume in world coordinates
    Vector3 GetBoundingBoxMin() const { return boundingBoxMin_; }
    Vector3 GetBoundingBoxMax() const { return boundingBoxMax_; }

    //! Get value range of the loaded volume
    float GetMinValue() const { return minValue_; }
    float GetMaxValue() const { return maxValue_; }

    //! Get voxel size
    Vector3 GetVoxelSize() const { return voxelSize_; }

    //! Get number of active voxels
    size_t GetActiveVoxelCount() const { return activeVoxelCount_; }

private:
    //! Initialize OpenVDB library
    void InitializeOpenVDB();

    //! Process OpenVDB grid and extract data for OpenVKL
    bool ProcessGrid(openvdb::FloatGrid::Ptr grid, VKLDevice device, VKLVolume volume);

    //! Extract leaf node data from OpenVDB tree
    bool ExtractLeafNodes(const openvdb::FloatTree& tree, VKLDevice device, VKLVolume volume);

    //! Set volume transform based on OpenVDB grid transform
    void SetVolumeTransform(openvdb::FloatGrid::Ptr grid, VKLVolume volume);

    // Volume properties
    Vector3 boundingBoxMin_;
    Vector3 boundingBoxMax_;
    Vector3 voxelSize_;
    float minValue_;
    float maxValue_;
    size_t activeVoxelCount_;
    bool initialized_;

    // OpenVKL data objects for cleanup
    std::vector<VKLData> vklDataObjects_;
};

#endif
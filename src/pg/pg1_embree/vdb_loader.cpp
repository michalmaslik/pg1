#include "stdafx.h"
#include "vdb_loader.h"
#include <iostream>
#include <limits>
#include <algorithm>

VdbLoader::VdbLoader() :
	boundingBoxMin_(0.0f, 0.0f, 0.0f),
	boundingBoxMax_(0.0f, 0.0f, 0.0f),
	voxelSize_(1.0f, 1.0f, 1.0f),
	minValue_(0.0f),
	maxValue_(0.0f),
	activeVoxelCount_(0),
	initialized_(false)
{
	InitializeOpenVDB();
}

VdbLoader::~VdbLoader() {
	// Cleanup VKL data objects
	for (auto& data : vklDataObjects_) {
		vklRelease(data);
	}
	vklDataObjects_.clear();
}

void VdbLoader::InitializeOpenVDB() {
	if (!initialized_) {
		openvdb::initialize();
		initialized_ = true;
		std::cout << "[VDB] OpenVDB initialized successfully" << std::endl;
	}
}

VKLVolume VdbLoader::LoadVdbFile(const std::string& filename, const std::string& gridName, VKLDevice device) {
	std::cout << "[VDB] Loading VDB file: " << filename << std::endl;
	std::cout << "[VDB] Grid name: " << gridName << std::endl;

	// Clear previous data
	for (auto& data : vklDataObjects_) {
		vklRelease(data);
	}
	vklDataObjects_.clear();

	// Open VDB file
	openvdb::io::File file(filename);
	try {
		file.open();
	}
	catch (const std::exception& e) {
		std::cerr << "[VDB ERROR] Failed to open VDB file: " << e.what() << std::endl;
		return VKLVolume{};
	}

	// Read specified grid
	openvdb::GridBase::Ptr gridBase;
	try {
		gridBase = file.readGrid(gridName);
		if (!gridBase) {
			std::cerr << "[VDB ERROR] Grid '" << gridName << "' not found in file!" << std::endl;
			return VKLVolume{};
		}
	}
	catch (const std::exception& e) {
		std::cerr << "[VDB ERROR] Exception while reading grid: " << e.what() << std::endl;
		return VKLVolume{};
	}
	file.close();

	std::cout << "[VDB] Grid type: " << gridBase->type() << std::endl;

	// Cast to FloatGrid
	openvdb::FloatGrid::Ptr grid = openvdb::gridPtrCast<openvdb::FloatGrid>(gridBase);
	if (!grid) {
		std::cerr << "[VDB ERROR] Grid is not a FloatGrid!" << std::endl;
		return VKLVolume{};
	}

	// Analyze grid properties
	auto bbox = grid->evalActiveVoxelBoundingBox();
	auto transform = grid->transform();
	auto voxelSizeVec = transform.voxelSize();

	voxelSize_ = Vector3(static_cast<float>(voxelSizeVec.x()),
		static_cast<float>(voxelSizeVec.y()),
		static_cast<float>(voxelSizeVec.z()));

	activeVoxelCount_ = grid->activeVoxelCount();

	std::cout << "[VDB] Bounding box: min(" << bbox.min().x() << "," << bbox.min().y() << "," << bbox.min().z() << ")"
		<< " max(" << bbox.max().x() << "," << bbox.max().y() << "," << bbox.max().z() << ")" << std::endl;
	std::cout << "[VDB] Voxel size: " << voxelSize_.x << " x " << voxelSize_.y << " x " << voxelSize_.z << std::endl;
	std::cout << "[VDB] Active voxels: " << activeVoxelCount_ << std::endl;

	// Analyze value range
	minValue_ = std::numeric_limits<float>::max();
	maxValue_ = std::numeric_limits<float>::lowest();

	for (auto iter = grid->cbeginValueOn(); iter; ++iter) {
		float val = *iter;
		if (minValue_ > val) minValue_ = val;
		if (maxValue_ < val) maxValue_ = val;
	}

	std::cout << "[VDB] Value range: [" << minValue_ << ", " << maxValue_ << "]" << std::endl;

	// Create OpenVKL VDB volume
	VKLVolume volume = vklNewVolume(device, "vdb");
	if (!volume) {
		std::cerr << "[VDB ERROR] Failed to create OpenVKL VDB volume!" << std::endl;
		return VKLVolume{};
	}

	// Process grid and populate volume
	if (!ProcessGrid(grid, device, volume)) {
		vklRelease(volume);
		return VKLVolume{};
	}

	std::cout << "[VDB] VDB volume created successfully" << std::endl;
	return volume;
}

bool VdbLoader::ProcessGrid(openvdb::FloatGrid::Ptr grid, VKLDevice device, VKLVolume volume) {
	// Set volume transform
	SetVolumeTransform(grid, volume);

	// Extract leaf node data
	if (!ExtractLeafNodes(grid->tree(), device, volume)) {
		return false;
	}

	// Set background value
	float background = static_cast<float>(grid->background());
	vklSetFloat(volume, "background", background);

	// Commit the volume
	vklCommit(volume);

	// Update bounding box
	vkl_box3f volumeBounds = vklGetBoundingBox(volume);
	boundingBoxMin_ = Vector3(volumeBounds.lower.x, volumeBounds.lower.y, volumeBounds.lower.z);
	boundingBoxMax_ = Vector3(volumeBounds.upper.x, volumeBounds.upper.y, volumeBounds.upper.z);

	return true;
}

bool VdbLoader::ExtractLeafNodes(const openvdb::FloatTree& tree, VKLDevice device, VKLVolume volume) {
	std::vector<VKLData> leafNodeData;
	std::vector<vkl_vec3i> leafOrigins;
	std::vector<uint32_t> leafLevels;
	std::vector<uint32_t> leafFormats;

	size_t leafCount = 0;
	std::cout << "[VDB] Processing leaf nodes..." << std::endl;

	// Process each leaf node
	for (auto leafIter = tree.cbeginLeaf(); leafIter; ++leafIter) {
		const auto& leaf = *leafIter;

		// Get leaf data
		const float* leafData = leaf.buffer().data();
		size_t leafSize = leaf.SIZE; // Usually 8^3 = 512

		// Create VKL data for this leaf node
		VKLData nodeData = vklNewData(device, leafSize, VKL_FLOAT, leafData, VKL_DATA_DEFAULT, 0);
		leafNodeData.push_back(nodeData);
		vklDataObjects_.push_back(nodeData); // Store for cleanup

		// Get leaf origin
		auto origin = leaf.origin();
		leafOrigins.push_back({ origin.x(), origin.y(), origin.z() });

		// VDB leaf nodes are at the lowest level (level 3 in 4-level hierarchy)
		leafLevels.push_back(3);

		// Dense format for leaf nodes
		leafFormats.push_back(VKL_FORMAT_DENSE_ZYX);

		leafCount++;
	}

	if (leafCount == 0) {
		std::cerr << "[VDB ERROR] No leaf nodes found in grid!" << std::endl;
		return false;
	}

	std::cout << "[VDB] Processed " << leafCount << " leaf nodes" << std::endl;

	// Create VKL data arrays
	VKLData nodeDataArray = vklNewData(device, leafNodeData.size(), VKL_DATA, leafNodeData.data(), VKL_DATA_DEFAULT, 0);
	VKLData originsArray = vklNewData(device, leafOrigins.size(), VKL_VEC3I, leafOrigins.data(), VKL_DATA_DEFAULT, 0);
	VKLData levelsArray = vklNewData(device, leafLevels.size(), VKL_UINT, leafLevels.data(), VKL_DATA_DEFAULT, 0);
	VKLData formatsArray = vklNewData(device, leafFormats.size(), VKL_UINT, leafFormats.data(), VKL_DATA_DEFAULT, 0);

	// Store for cleanup
	vklDataObjects_.push_back(nodeDataArray);
	vklDataObjects_.push_back(originsArray);
	vklDataObjects_.push_back(levelsArray);
	vklDataObjects_.push_back(formatsArray);

	// Set VDB volume parameters
	vklSetData(volume, "node.data", nodeDataArray);
	vklSetData(volume, "node.origin", originsArray);
	vklSetData(volume, "node.level", levelsArray);
	vklSetData(volume, "node.format", formatsArray);

	return true;
}

void VdbLoader::SetVolumeTransform(openvdb::FloatGrid::Ptr grid, VKLVolume volume) {
	auto transform = grid->transform();
	auto voxelSizeVec = transform.voxelSize();

	// Create index to object transformation matrix
	// OpenVKL expects 12 floats: 3x3 linear transform + 3D translation
	float indexToObject[12] = {
		static_cast<float>(voxelSizeVec.x()), 0.0f, 0.0f,  // X scaling
		0.0f, static_cast<float>(voxelSizeVec.y()), 0.0f,  // Y scaling
		0.0f, 0.0f, static_cast<float>(voxelSizeVec.z()),  // Z scaling
		0.0f, 0.0f, 0.0f                                   // Translation (origin)
	};

	vklSetParam(volume, "indexToObject", VKL_AFFINE3F, indexToObject);
}
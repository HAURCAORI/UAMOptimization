#include "visualization/ArchitectureViewerApp.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visualization/ArchitectureSceneBuilder.hpp"
#include "visualization/GltfLoader.hpp"
#include "visualization/PrimitiveMeshFactory.hpp"

#if __has_include(<GLFW/glfw3.h>) && __has_include(<vulkan/vulkan.h>) && __has_include(<volk.h>)
#define HEXAARCH_VISUALIZATION_HAS_VULKAN 1
#define GLFW_INCLUDE_NONE
#define VK_NO_PROTOTYPES
#include <vulkan/vulkan.h>
#include <GLFW/glfw3.h>
#include <volk.h>
#if __has_include(<imgui.h>) && __has_include(<imgui_impl_glfw.h>) && __has_include(<imgui_impl_vulkan.h>)
#define HEXAARCH_HAS_IMGUI 1
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>
#endif
#if defined(_WIN32)
#include <Windows.h>
#endif
#endif

namespace hexaarch::visualization {
namespace fs = std::filesystem;

namespace {

std::string primitiveKindName(const core::GeometryPrimitive::Kind kind) {
    switch (kind) {
    case core::GeometryPrimitive::Kind::sphere:
        return "sphere";
    case core::GeometryPrimitive::Kind::box:
        return "box";
    case core::GeometryPrimitive::Kind::cylinder:
        return "cylinder";
    case core::GeometryPrimitive::Kind::disk:
        return "disk";
    case core::GeometryPrimitive::Kind::segment:
        return "segment";
    }
    return "unknown";
}

double primitiveBoundingRadius(const PrimitiveInstance& instance) {
    switch (instance.primitive_type) {
    case core::GeometryPrimitive::Kind::sphere:
        return instance.dimensions.x() + instance.padding;
    case core::GeometryPrimitive::Kind::box:
        return instance.dimensions.norm() + instance.padding;
    case core::GeometryPrimitive::Kind::cylinder: {
        const double radius = instance.dimensions.x() + instance.padding;
        const double half_length = 0.5 * instance.dimensions.y() + instance.padding;
        return std::sqrt(radius * radius + half_length * half_length);
    }
    case core::GeometryPrimitive::Kind::disk:
        return instance.dimensions.x() + instance.padding;
    case core::GeometryPrimitive::Kind::segment:
        return 0.5 * instance.dimensions.x() + instance.padding;
    }
    return 1.0;
}

void printSceneSummary(
    const ArchitectureViewerApp::Config& config,
    const core::HexacopterArchitecture& architecture,
    const std::vector<PrimitiveInstance>& instances,
    const std::string_view banner) {
    std::map<std::string, std::size_t> primitive_type_counts;
    std::map<std::string, std::size_t> element_type_counts;
    for (const auto& instance : instances) {
        primitive_type_counts[primitiveKindName(instance.primitive_type)] += 1U;
        element_type_counts[instance.source_element_type] += 1U;
    }

    std::cout << config.title << '\n';
    std::cout << banner << '\n';
    std::cout << "Window: " << config.width << "x" << config.height << '\n';
    std::cout << "Assembled elements: " << architecture.assemblyState().elements.size() << '\n';
    std::cout << "Primitive instances: " << instances.size() << '\n';
    std::cout << "Primitive types:\n";
    for (const auto& [name, count] : primitive_type_counts) {
        std::cout << "  " << name << ": " << count << '\n';
    }
    std::cout << "Element types:\n";
    for (const auto& [name, count] : element_type_counts) {
        std::cout << "  " << name << ": " << count << '\n';
    }
    std::cout << "Controls:\n";
    std::cout << "  LMB drag: orbit\n";
    std::cout << "  RMB drag: pan\n";
    std::cout << "  Wheel: zoom\n";
    std::cout << "  Esc: close\n";
}

#ifdef HEXAARCH_VISUALIZATION_HAS_VULKAN

constexpr std::size_t kFramesInFlight = 2;
constexpr float kOrbitSpeed = 0.0085f;
constexpr float kPanSpeed = 0.0025f;
constexpr float kZoomSpeed = 0.12f;
constexpr float kDiskThickness = 0.02f;
constexpr float kSegmentThickness = 0.08f;

struct QueueFamilySelection {
    std::optional<std::uint32_t> graphics_family;
    std::optional<std::uint32_t> present_family;

    [[nodiscard]] bool complete() const {
        return graphics_family.has_value() && present_family.has_value();
    }
};

struct SwapchainSupportDetails {
    VkSurfaceCapabilitiesKHR capabilities{};
    std::vector<VkSurfaceFormatKHR> formats;
    std::vector<VkPresentModeKHR> present_modes;
};

struct BufferResource {
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    void* mapped = nullptr;
    VkDeviceSize size = 0;
};

struct MeshResource {
    BufferResource vertex_buffer;
    BufferResource index_buffer;
    std::uint32_t index_count = 0;
    std::string material_name;
};

struct FrameResources {
    VkCommandPool command_pool = VK_NULL_HANDLE;
    VkCommandBuffer command_buffer = VK_NULL_HANDLE;
    VkSemaphore image_available = VK_NULL_HANDLE;
    VkFence in_flight = VK_NULL_HANDLE;
    BufferResource uniform_buffer;
    VkDescriptorSet descriptor_set = VK_NULL_HANDLE;
};

struct GlobalUniformData {
    alignas(16) float view[16]{};
    alignas(16) float projection[16]{};
    alignas(16) float light_direction[4]{-0.35f, -0.80f, -0.45f, 0.0f};
};

struct PushConstants {
    alignas(16) float model[16]{};
    alignas(16) float color[4]{};
};

std::array<float, 16> toFloatArray(const Eigen::Matrix4f& matrix) {
    std::array<float, 16> values{};
    std::memcpy(values.data(), matrix.data(), sizeof(float) * values.size());
    return values;
}

Eigen::Matrix4f castMatrix(const Eigen::Matrix4d& matrix) {
    return matrix.cast<float>();
}

Eigen::Matrix4f modelMatrixForInstance(const PrimitiveInstance& instance) {
    Eigen::Affine3d primitive_transform = instance.world_transform;
    const double padding = instance.padding;

    Eigen::Vector3d scale = Eigen::Vector3d::Ones();
    switch (instance.primitive_type) {
    case core::GeometryPrimitive::Kind::sphere: {
        const double diameter = 2.0 * (instance.dimensions.x() + padding);
        scale = Eigen::Vector3d::Constant(diameter);
        break;
    }
    case core::GeometryPrimitive::Kind::box:
        scale = 2.0 * (instance.dimensions + Eigen::Vector3d::Constant(padding));
        break;
    case core::GeometryPrimitive::Kind::cylinder: {
        const double diameter = 2.0 * (instance.dimensions.x() + padding);
        const double length = instance.dimensions.y() + 2.0 * padding;
        scale = {diameter, length, diameter};
        break;
    }
    case core::GeometryPrimitive::Kind::disk: {
        const double diameter = 2.0 * (instance.dimensions.x() + padding);
        scale = {diameter, diameter, std::max(static_cast<double>(kDiskThickness), 2.0 * padding)};
        break;
    }
    case core::GeometryPrimitive::Kind::segment: {
        const double length = instance.dimensions.x() + 2.0 * padding;
        const double diameter = padding > 1e-6 ? 2.0 * padding : static_cast<double>(kSegmentThickness);
        primitive_transform.translate(Eigen::Vector3d(0.5 * length, 0.0, 0.0));
        // Rotate the unit cylinder (axis along Y) to align with the arm (axis along X).
        primitive_transform.rotate(Eigen::AngleAxisd(-1.5707963267948966, Eigen::Vector3d::UnitZ()));
        scale = {diameter, length, diameter};
        break;
    }
    }

    primitive_transform.scale(scale);
    return castMatrix(primitive_transform.matrix());
}

Eigen::Vector3d sceneCenter(const std::vector<PrimitiveInstance>& instances) {
    if (instances.empty()) {
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double total_weight = 0.0;
    for (const auto& instance : instances) {
        const double weight = std::max(primitiveBoundingRadius(instance), 0.1);
        center += weight * instance.world_transform.translation();
        total_weight += weight;
    }
    if (total_weight <= 0.0) {
        return instances.front().world_transform.translation();
    }
    return center / total_weight;
}

double sceneRadius(const std::vector<PrimitiveInstance>& instances, const Eigen::Vector3d& center) {
    double radius = 1.0;
    for (const auto& instance : instances) {
        const double distance = (instance.world_transform.translation() - center).norm();
        radius = std::max(radius, distance + primitiveBoundingRadius(instance));
    }
    return radius;
}

double sceneMinZ(const std::vector<PrimitiveInstance>& instances) {
    double min_z = 0.0;
    bool first = true;
    for (const auto& instance : instances) {
        const double z = instance.world_transform.translation().z() - primitiveBoundingRadius(instance);
        if (first) {
            min_z = z;
            first = false;
        } else {
            min_z = std::min(min_z, z);
        }
    }
    return min_z;
}

PrimitiveInstance makeReferenceBox(
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& half_extents,
    const std::array<float, 4>& color,
    const std::string& id,
    const std::string& type) {
    PrimitiveInstance instance;
    instance.primitive_type = core::GeometryPrimitive::Kind::box;
    instance.world_transform = Eigen::Isometry3d::Identity();
    instance.world_transform.translation() = center;
    instance.dimensions = half_extents;
    instance.color = color;
    instance.source_element_id = id;
    instance.source_element_type = type;
    return instance;
}

void appendSpatialReferences(std::vector<PrimitiveInstance>& instances) {
    const Eigen::Vector3d center = sceneCenter(instances);
    const double radius = sceneRadius(instances, center);
    const double min_z = sceneMinZ(instances);
    const double grid_z = min_z - std::max(0.05 * radius, 0.05);
    const double axis_length = std::max(radius * 1.2, 2.0);
    const double axis_thickness = std::max(0.015 * radius, 0.03);
    const double grid_span = std::max(radius * 1.5, 3.0);
    const int grid_divisions = 10;
    const double grid_thickness = std::max(0.006 * radius, 0.01);

    instances.push_back(makeReferenceBox(
        Eigen::Vector3d(0.5 * axis_length, 0.0, 0.0),
        Eigen::Vector3d(0.5 * axis_length, axis_thickness, axis_thickness),
        {0.85f, 0.20f, 0.20f, 1.0f},
        "ref_axis_x",
        "ReferenceAxis"));
    instances.push_back(makeReferenceBox(
        Eigen::Vector3d(0.0, 0.5 * axis_length, 0.0),
        Eigen::Vector3d(axis_thickness, 0.5 * axis_length, axis_thickness),
        {0.20f, 0.75f, 0.25f, 1.0f},
        "ref_axis_y",
        "ReferenceAxis"));
    instances.push_back(makeReferenceBox(
        Eigen::Vector3d(0.0, 0.0, 0.5 * axis_length),
        Eigen::Vector3d(axis_thickness, axis_thickness, 0.5 * axis_length),
        {0.20f, 0.40f, 0.90f, 1.0f},
        "ref_axis_z",
        "ReferenceAxis"));

    const double step = (2.0 * grid_span) / static_cast<double>(grid_divisions);
    for (int index = 0; index <= grid_divisions; ++index) {
        const double offset = -grid_span + static_cast<double>(index) * step;
        const float shade = (index == grid_divisions / 2) ? 0.58f : 0.82f;
        const std::array<float, 4> color{shade, shade, shade, 1.0f};

        instances.push_back(makeReferenceBox(
            Eigen::Vector3d(0.0, offset, grid_z),
            Eigen::Vector3d(grid_span, grid_thickness, grid_thickness),
            color,
            "ref_grid_x_" + std::to_string(index),
            "ReferenceGrid"));
        instances.push_back(makeReferenceBox(
            Eigen::Vector3d(offset, 0.0, grid_z),
            Eigen::Vector3d(grid_thickness, grid_span, grid_thickness),
            color,
            "ref_grid_y_" + std::to_string(index),
            "ReferenceGrid"));
    }
}

Eigen::Vector3d cameraForward(const ViewerCamera& camera) {
    return (camera.target() - camera.position()).normalized();
}

Eigen::Vector3d cameraRight(const ViewerCamera& camera) {
    return cameraForward(camera).cross(Eigen::Vector3d::UnitZ()).normalized();
}

Eigen::Vector3d cameraUp(const ViewerCamera& camera) {
    return cameraRight(camera).cross(cameraForward(camera)).normalized();
}

std::vector<char> readBinaryFile(const fs::path& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        throw std::runtime_error("Failed to open file: " + path.string());
    }
    const std::streamsize size = file.tellg();
    file.seekg(0);
    std::vector<char> bytes(static_cast<std::size_t>(size));
    file.read(bytes.data(), size);
    return bytes;
}

std::vector<fs::path> candidateShaderRoots() {
    std::vector<fs::path> roots;
    roots.push_back(fs::current_path() / "visualization" / "shaders");
    roots.push_back(fs::current_path() / "FrameworkCpp" / "visualization" / "shaders");
#if defined(_WIN32)
    char buffer[MAX_PATH]{};
    const DWORD count = GetModuleFileNameA(nullptr, buffer, MAX_PATH);
    if (count > 0) {
        const fs::path exe_dir = fs::path(buffer).parent_path();
        roots.push_back(exe_dir / "visualization" / "shaders");
        roots.push_back(exe_dir.parent_path() / "visualization" / "shaders");
        roots.push_back(exe_dir.parent_path().parent_path() / "visualization" / "shaders");
        roots.push_back(exe_dir.parent_path().parent_path().parent_path() / "visualization" / "shaders");
    }
#endif
    return roots;
}

fs::path resolveShaderRoot() {
    for (const auto& candidate : candidateShaderRoots()) {
        if (fs::exists(candidate / "compiled" / "primitive.vert.spv")
            && fs::exists(candidate / "compiled" / "primitive.frag.spv")) {
            return candidate;
        }
    }
    throw std::runtime_error("Could not locate compiled visualization shaders.");
}

VkVertexInputBindingDescription vertexBindingDescription() {
    VkVertexInputBindingDescription binding{};
    binding.binding = 0;
    binding.stride = static_cast<std::uint32_t>(sizeof(MeshVertex));
    binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    return binding;
}

std::array<VkVertexInputAttributeDescription, 2> vertexAttributeDescriptions() {
    std::array<VkVertexInputAttributeDescription, 2> attributes{};
    attributes[0].location = 0;
    attributes[0].binding = 0;
    attributes[0].format = VK_FORMAT_R32G32B32_SFLOAT;
    attributes[0].offset = static_cast<std::uint32_t>(offsetof(MeshVertex, position));

    attributes[1].location = 1;
    attributes[1].binding = 0;
    attributes[1].format = VK_FORMAT_R32G32B32_SFLOAT;
    attributes[1].offset = static_cast<std::uint32_t>(offsetof(MeshVertex, normal));
    return attributes;
}

QueueFamilySelection findQueueFamilies(
    const VkPhysicalDevice physical_device,
    const VkSurfaceKHR surface) {
    QueueFamilySelection selection;
    std::uint32_t family_count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(physical_device, &family_count, nullptr);
    std::vector<VkQueueFamilyProperties> families(family_count);
    vkGetPhysicalDeviceQueueFamilyProperties(physical_device, &family_count, families.data());

    for (std::uint32_t index = 0; index < family_count; ++index) {
        const auto& family = families[index];
        if (family.queueFlags & VK_QUEUE_GRAPHICS_BIT) {
            selection.graphics_family = index;
        }

        VkBool32 present_supported = VK_FALSE;
        vkGetPhysicalDeviceSurfaceSupportKHR(physical_device, index, surface, &present_supported);
        if (present_supported == VK_TRUE) {
            selection.present_family = index;
        }

        if (selection.complete()) {
            break;
        }
    }
    return selection;
}

SwapchainSupportDetails querySwapchainSupport(
    const VkPhysicalDevice physical_device,
    const VkSurfaceKHR surface) {
    SwapchainSupportDetails details;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physical_device, surface, &details.capabilities);

    std::uint32_t format_count = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(physical_device, surface, &format_count, nullptr);
    if (format_count > 0) {
        details.formats.resize(format_count);
        vkGetPhysicalDeviceSurfaceFormatsKHR(physical_device, surface, &format_count, details.formats.data());
    }

    std::uint32_t present_mode_count = 0;
    vkGetPhysicalDeviceSurfacePresentModesKHR(physical_device, surface, &present_mode_count, nullptr);
    if (present_mode_count > 0) {
        details.present_modes.resize(present_mode_count);
        vkGetPhysicalDeviceSurfacePresentModesKHR(
            physical_device,
            surface,
            &present_mode_count,
            details.present_modes.data());
    }
    return details;
}

VkSurfaceFormatKHR chooseSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& formats) {
    for (const auto& format : formats) {
        if (format.format == VK_FORMAT_B8G8R8A8_UNORM
            && format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            return format;
        }
    }
    return formats.front();
}

VkPresentModeKHR choosePresentMode(const std::vector<VkPresentModeKHR>& present_modes) {
    for (const auto present_mode : present_modes) {
        if (present_mode == VK_PRESENT_MODE_MAILBOX_KHR) {
            return present_mode;
        }
    }
    return VK_PRESENT_MODE_FIFO_KHR;
}

VkExtent2D chooseSwapExtent(
    const VkSurfaceCapabilitiesKHR& capabilities,
    GLFWwindow* window) {
    if (capabilities.currentExtent.width != std::numeric_limits<std::uint32_t>::max()) {
        return capabilities.currentExtent;
    }

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window, &width, &height);

    VkExtent2D extent{};
    extent.width = std::clamp(
        static_cast<std::uint32_t>(std::max(width, 1)),
        capabilities.minImageExtent.width,
        capabilities.maxImageExtent.width);
    extent.height = std::clamp(
        static_cast<std::uint32_t>(std::max(height, 1)),
        capabilities.minImageExtent.height,
        capabilities.maxImageExtent.height);
    return extent;
}

bool deviceSupportsExtensions(const VkPhysicalDevice physical_device) {
    std::uint32_t extension_count = 0;
    vkEnumerateDeviceExtensionProperties(physical_device, nullptr, &extension_count, nullptr);
    std::vector<VkExtensionProperties> available_extensions(extension_count);
    vkEnumerateDeviceExtensionProperties(
        physical_device,
        nullptr,
        &extension_count,
        available_extensions.data());

    std::set<std::string> required_extensions{VK_KHR_SWAPCHAIN_EXTENSION_NAME};
    for (const auto& extension : available_extensions) {
        required_extensions.erase(extension.extensionName);
    }
    return required_extensions.empty();
}

bool isDeviceSuitable(const VkPhysicalDevice physical_device, const VkSurfaceKHR surface) {
    const QueueFamilySelection families = findQueueFamilies(physical_device, surface);
    if (!families.complete() || !deviceSupportsExtensions(physical_device)) {
        return false;
    }

    const SwapchainSupportDetails swapchain_support = querySwapchainSupport(physical_device, surface);
    return !swapchain_support.formats.empty() && !swapchain_support.present_modes.empty();
}

VkFormat findDepthFormat(const VkPhysicalDevice physical_device) {
    constexpr std::array<VkFormat, 3> candidates{
        VK_FORMAT_D32_SFLOAT,
        VK_FORMAT_D32_SFLOAT_S8_UINT,
        VK_FORMAT_D24_UNORM_S8_UINT};

    for (const auto format : candidates) {
        VkFormatProperties properties{};
        vkGetPhysicalDeviceFormatProperties(physical_device, format, &properties);
        if (properties.optimalTilingFeatures & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT) {
            return format;
        }
    }
    throw std::runtime_error("No supported depth format found.");
}

std::uint32_t findMemoryType(
    const VkPhysicalDevice physical_device,
    const std::uint32_t type_filter,
    const VkMemoryPropertyFlags properties) {
    VkPhysicalDeviceMemoryProperties memory_properties{};
    vkGetPhysicalDeviceMemoryProperties(physical_device, &memory_properties);

    for (std::uint32_t index = 0; index < memory_properties.memoryTypeCount; ++index) {
        if ((type_filter & (1U << index)) != 0U
            && (memory_properties.memoryTypes[index].propertyFlags & properties) == properties) {
            return index;
        }
    }
    throw std::runtime_error("Failed to find suitable memory type.");
}

void destroyBuffer(const VkDevice device, BufferResource& buffer) {
    if (buffer.mapped != nullptr) {
        vkUnmapMemory(device, buffer.memory);
        buffer.mapped = nullptr;
    }
    if (buffer.buffer != VK_NULL_HANDLE) {
        vkDestroyBuffer(device, buffer.buffer, nullptr);
        buffer.buffer = VK_NULL_HANDLE;
    }
    if (buffer.memory != VK_NULL_HANDLE) {
        vkFreeMemory(device, buffer.memory, nullptr);
        buffer.memory = VK_NULL_HANDLE;
    }
    buffer.size = 0;
}

void createBuffer(
    const VkPhysicalDevice physical_device,
    const VkDevice device,
    const VkDeviceSize size,
    const VkBufferUsageFlags usage,
    const VkMemoryPropertyFlags properties,
    BufferResource& buffer) {
    buffer.size = size;

    VkBufferCreateInfo buffer_info{};
    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_info.size = size;
    buffer_info.usage = usage;
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (vkCreateBuffer(device, &buffer_info, nullptr, &buffer.buffer) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create buffer.");
    }

    VkMemoryRequirements memory_requirements{};
    vkGetBufferMemoryRequirements(device, buffer.buffer, &memory_requirements);

    VkMemoryAllocateInfo allocation_info{};
    allocation_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocation_info.allocationSize = memory_requirements.size;
    allocation_info.memoryTypeIndex =
        findMemoryType(physical_device, memory_requirements.memoryTypeBits, properties);

    if (vkAllocateMemory(device, &allocation_info, nullptr, &buffer.memory) != VK_SUCCESS) {
        throw std::runtime_error("Failed to allocate buffer memory.");
    }

    vkBindBufferMemory(device, buffer.buffer, buffer.memory, 0);
}

VkCommandBuffer beginSingleTimeCommands(const VkDevice device, const VkCommandPool pool) {
    VkCommandBufferAllocateInfo allocation_info{};
    allocation_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocation_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocation_info.commandPool = pool;
    allocation_info.commandBufferCount = 1;

    VkCommandBuffer command_buffer = VK_NULL_HANDLE;
    if (vkAllocateCommandBuffers(device, &allocation_info, &command_buffer) != VK_SUCCESS) {
        throw std::runtime_error("Failed to allocate single-time command buffer.");
    }

    VkCommandBufferBeginInfo begin_info{};
    begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(command_buffer, &begin_info);
    return command_buffer;
}

void endSingleTimeCommands(
    const VkDevice device,
    const VkQueue graphics_queue,
    const VkCommandPool pool,
    const VkCommandBuffer command_buffer) {
    vkEndCommandBuffer(command_buffer);

    VkSubmitInfo submit_info{};
    submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit_info.commandBufferCount = 1;
    submit_info.pCommandBuffers = &command_buffer;

    vkQueueSubmit(graphics_queue, 1, &submit_info, VK_NULL_HANDLE);
    vkQueueWaitIdle(graphics_queue);
    vkFreeCommandBuffers(device, pool, 1, &command_buffer);
}

VkShaderModule createShaderModule(const VkDevice device, const std::vector<char>& code) {
    VkShaderModuleCreateInfo create_info{};
    create_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    create_info.codeSize = code.size();
    create_info.pCode = reinterpret_cast<const std::uint32_t*>(code.data());

    VkShaderModule module = VK_NULL_HANDLE;
    if (vkCreateShaderModule(device, &create_info, nullptr, &module) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create shader module.");
    }
    return module;
}

}  // namespace

struct ArchitectureViewerApp::Impl {
    explicit Impl(ArchitectureViewerApp::Config config_in)
        : config(std::move(config_in)) {}

    ArchitectureViewerApp::Config config;
    ViewerCamera camera;
    std::vector<PrimitiveInstance> instances;
    std::unordered_map<core::GeometryPrimitive::Kind, MeshResource> meshes;
    fs::path shader_root;

    GLFWwindow* window = nullptr;
    bool framebuffer_resized = false;
    bool should_close = false;

    VkInstance instance = VK_NULL_HANDLE;
    VkSurfaceKHR surface = VK_NULL_HANDLE;
    VkPhysicalDevice physical_device = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    VkQueue graphics_queue = VK_NULL_HANDLE;
    VkQueue present_queue = VK_NULL_HANDLE;
    QueueFamilySelection queue_families;

    VkSwapchainKHR swapchain = VK_NULL_HANDLE;
    VkFormat swapchain_format = VK_FORMAT_UNDEFINED;
    VkExtent2D swapchain_extent{};
    std::vector<VkImage> swapchain_images;
    std::vector<VkImageView> swapchain_image_views;

    VkRenderPass render_pass = VK_NULL_HANDLE;
    VkDescriptorSetLayout descriptor_set_layout = VK_NULL_HANDLE;
    VkPipelineLayout pipeline_layout = VK_NULL_HANDLE;
    VkPipeline graphics_pipeline = VK_NULL_HANDLE;
    VkDescriptorPool descriptor_pool = VK_NULL_HANDLE;

    VkImage depth_image = VK_NULL_HANDLE;
    VkDeviceMemory depth_memory = VK_NULL_HANDLE;
    VkImageView depth_image_view = VK_NULL_HANDLE;
    VkFormat depth_format = VK_FORMAT_UNDEFINED;

    std::vector<VkFramebuffer> swapchain_framebuffers;

    std::array<FrameResources, kFramesInFlight> frames{};
    std::vector<VkSemaphore> render_finished_semaphores;
    std::vector<VkFence> images_in_flight;
    std::size_t current_frame = 0;

    bool fillmode_nonsolid_supported = false;
    VkPipeline wireframe_pipeline = VK_NULL_HANDLE;
    VkPipeline alpha_pipeline = VK_NULL_HANDLE;

    std::vector<MeshResource> mannequin_meshes;
    bool mannequin_loaded = false;
    // Human reference figure placed 9 m in +Y, feet at Z = 0 (hub height).
    Eigen::Vector3d mannequin_world_pos{0.0, 9.0, 0.0};
    VkSampleCountFlagBits msaa_samples = VK_SAMPLE_COUNT_4_BIT;
    VkImage msaa_color_image = VK_NULL_HANDLE;
    VkDeviceMemory msaa_color_memory = VK_NULL_HANDLE;
    VkImageView msaa_color_view = VK_NULL_HANDLE;
    VkCommandPool transfer_pool = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT debug_messenger = VK_NULL_HANDLE;

    bool first_mouse = true;
    double last_cursor_x = 0.0;
    double last_cursor_y = 0.0;

    bool ui_show_axes = true;
    bool ui_show_grid = true;
    bool ui_show_labels = true;
    bool ui_wireframe = false;
    std::unordered_map<std::string, bool> element_visibility;

#ifdef HEXAARCH_HAS_IMGUI
    VkDescriptorPool imgui_pool = VK_NULL_HANDLE;
    float ui_dpi_scale = 1.0f;
    float ui_scale = 1.0f;       // committed scale — drives panel size and style
    float ui_scale_drag = 1.0f;  // live slider value during drag; diverges from ui_scale
    ImGuiStyle imgui_style_base{};
#endif

    std::mutex pending_mutex;
    std::optional<core::HexacopterArchitecture> pending_arch;
    std::string pending_title;
    std::optional<core::HexacopterArchitecture> owned_arch;

    [[nodiscard]] fs::path compiledShaderPath(const std::string_view source_name) const {
        return shader_root / "compiled" / (std::string(source_name) + ".spv");
    }

    static void framebufferResizeCallback(GLFWwindow* window, int, int) {
        auto* impl = static_cast<Impl*>(glfwGetWindowUserPointer(window));
        if (impl != nullptr) {
            impl->framebuffer_resized = true;
        }
    }

    static void scrollCallback(GLFWwindow* window, double, double yoffset) {
        auto* impl = static_cast<Impl*>(glfwGetWindowUserPointer(window));
        if (impl == nullptr) { return; }
#ifdef HEXAARCH_HAS_IMGUI
        if (ImGui::GetIO().WantCaptureMouse) { return; }
#endif
        impl->camera.zoom(-yoffset * kZoomSpeed * std::max(impl->camera.distance(), 1.0));
    }
};

namespace {

VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT,
    VkDebugUtilsMessageTypeFlagsEXT,
    const VkDebugUtilsMessengerCallbackDataEXT* data,
    void*) {
    std::cerr << "[Vulkan] " << data->pMessage << '\n';
    return VK_FALSE;
}

void initializeImGui(ArchitectureViewerApp::Impl& impl);
void cleanupImGui(ArchitectureViewerApp::Impl& impl);

VkSampleCountFlagBits pickMsaaSamples(const VkPhysicalDevice device) {
    VkPhysicalDeviceProperties props{};
    vkGetPhysicalDeviceProperties(device, &props);
    const VkSampleCountFlags counts =
        props.limits.framebufferColorSampleCounts & props.limits.framebufferDepthSampleCounts;
    for (const auto s : {VK_SAMPLE_COUNT_4_BIT, VK_SAMPLE_COUNT_2_BIT}) {
        if (counts & s) { return s; }
    }
    return VK_SAMPLE_COUNT_1_BIT;
}

void destroySwapchainResources(ArchitectureViewerApp::Impl& impl) {
    for (auto framebuffer : impl.swapchain_framebuffers) {
        vkDestroyFramebuffer(impl.device, framebuffer, nullptr);
    }
    impl.swapchain_framebuffers.clear();

    if (impl.msaa_color_view != VK_NULL_HANDLE) {
        vkDestroyImageView(impl.device, impl.msaa_color_view, nullptr);
        impl.msaa_color_view = VK_NULL_HANDLE;
    }
    if (impl.msaa_color_image != VK_NULL_HANDLE) {
        vkDestroyImage(impl.device, impl.msaa_color_image, nullptr);
        impl.msaa_color_image = VK_NULL_HANDLE;
    }
    if (impl.msaa_color_memory != VK_NULL_HANDLE) {
        vkFreeMemory(impl.device, impl.msaa_color_memory, nullptr);
        impl.msaa_color_memory = VK_NULL_HANDLE;
    }

    if (impl.depth_image_view != VK_NULL_HANDLE) {
        vkDestroyImageView(impl.device, impl.depth_image_view, nullptr);
        impl.depth_image_view = VK_NULL_HANDLE;
    }
    if (impl.depth_image != VK_NULL_HANDLE) {
        vkDestroyImage(impl.device, impl.depth_image, nullptr);
        impl.depth_image = VK_NULL_HANDLE;
    }
    if (impl.depth_memory != VK_NULL_HANDLE) {
        vkFreeMemory(impl.device, impl.depth_memory, nullptr);
        impl.depth_memory = VK_NULL_HANDLE;
    }

    if (impl.alpha_pipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(impl.device, impl.alpha_pipeline, nullptr);
        impl.alpha_pipeline = VK_NULL_HANDLE;
    }
    if (impl.wireframe_pipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(impl.device, impl.wireframe_pipeline, nullptr);
        impl.wireframe_pipeline = VK_NULL_HANDLE;
    }
    if (impl.graphics_pipeline != VK_NULL_HANDLE) {
        vkDestroyPipeline(impl.device, impl.graphics_pipeline, nullptr);
        impl.graphics_pipeline = VK_NULL_HANDLE;
    }
    if (impl.pipeline_layout != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(impl.device, impl.pipeline_layout, nullptr);
        impl.pipeline_layout = VK_NULL_HANDLE;
    }
    if (impl.render_pass != VK_NULL_HANDLE) {
        vkDestroyRenderPass(impl.device, impl.render_pass, nullptr);
        impl.render_pass = VK_NULL_HANDLE;
    }

    for (auto image_view : impl.swapchain_image_views) {
        vkDestroyImageView(impl.device, image_view, nullptr);
    }
    impl.swapchain_image_views.clear();

    if (impl.swapchain != VK_NULL_HANDLE) {
        vkDestroySwapchainKHR(impl.device, impl.swapchain, nullptr);
        impl.swapchain = VK_NULL_HANDLE;
    }

    for (auto semaphore : impl.render_finished_semaphores) {
        vkDestroySemaphore(impl.device, semaphore, nullptr);
    }
    impl.render_finished_semaphores.clear();
    impl.images_in_flight.clear();
}

void createSwapchain(ArchitectureViewerApp::Impl& impl) {
    const SwapchainSupportDetails support = querySwapchainSupport(impl.physical_device, impl.surface);
    const VkSurfaceFormatKHR surface_format = chooseSurfaceFormat(support.formats);
    const VkPresentModeKHR present_mode = choosePresentMode(support.present_modes);
    const VkExtent2D extent = chooseSwapExtent(support.capabilities, impl.window);

    std::uint32_t image_count = support.capabilities.minImageCount + 1;
    if (support.capabilities.maxImageCount > 0 && image_count > support.capabilities.maxImageCount) {
        image_count = support.capabilities.maxImageCount;
    }

    VkSwapchainCreateInfoKHR create_info{};
    create_info.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    create_info.surface = impl.surface;
    create_info.minImageCount = image_count;
    create_info.imageFormat = surface_format.format;
    create_info.imageColorSpace = surface_format.colorSpace;
    create_info.imageExtent = extent;
    create_info.imageArrayLayers = 1;
    create_info.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

    const std::uint32_t queue_family_indices[] = {
        impl.queue_families.graphics_family.value(),
        impl.queue_families.present_family.value()};
    if (impl.queue_families.graphics_family != impl.queue_families.present_family) {
        create_info.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
        create_info.queueFamilyIndexCount = 2;
        create_info.pQueueFamilyIndices = queue_family_indices;
    } else {
        create_info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    }

    create_info.preTransform = support.capabilities.currentTransform;
    create_info.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    create_info.presentMode = present_mode;
    create_info.clipped = VK_TRUE;
    create_info.oldSwapchain = VK_NULL_HANDLE;

    if (vkCreateSwapchainKHR(impl.device, &create_info, nullptr, &impl.swapchain) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create swapchain.");
    }

    vkGetSwapchainImagesKHR(impl.device, impl.swapchain, &image_count, nullptr);
    impl.swapchain_images.resize(image_count);
    vkGetSwapchainImagesKHR(impl.device, impl.swapchain, &image_count, impl.swapchain_images.data());
    impl.swapchain_format = surface_format.format;
    impl.swapchain_extent = extent;
    impl.images_in_flight.assign(image_count, VK_NULL_HANDLE);
}

void createSwapchainImageViews(ArchitectureViewerApp::Impl& impl) {
    impl.swapchain_image_views.resize(impl.swapchain_images.size());
    for (std::size_t index = 0; index < impl.swapchain_images.size(); ++index) {
        VkImageViewCreateInfo view_info{};
        view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        view_info.image = impl.swapchain_images[index];
        view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
        view_info.format = impl.swapchain_format;
        view_info.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
        view_info.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
        view_info.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
        view_info.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
        view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        view_info.subresourceRange.baseMipLevel = 0;
        view_info.subresourceRange.levelCount = 1;
        view_info.subresourceRange.baseArrayLayer = 0;
        view_info.subresourceRange.layerCount = 1;
        if (vkCreateImageView(impl.device, &view_info, nullptr, &impl.swapchain_image_views[index]) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create swapchain image view.");
        }
    }
}

void createColorResources(ArchitectureViewerApp::Impl& impl) {
    if (impl.msaa_samples == VK_SAMPLE_COUNT_1_BIT) { return; }

    VkImageCreateInfo image_info{};
    image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image_info.imageType = VK_IMAGE_TYPE_2D;
    image_info.extent.width = impl.swapchain_extent.width;
    image_info.extent.height = impl.swapchain_extent.height;
    image_info.extent.depth = 1;
    image_info.mipLevels = 1;
    image_info.arrayLayers = 1;
    image_info.format = impl.swapchain_format;
    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    image_info.usage = VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    image_info.samples = impl.msaa_samples;
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateImage(impl.device, &image_info, nullptr, &impl.msaa_color_image) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create MSAA color image.");
    }

    VkMemoryRequirements mem_reqs{};
    vkGetImageMemoryRequirements(impl.device, impl.msaa_color_image, &mem_reqs);
    VkMemoryAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc_info.allocationSize = mem_reqs.size;
    alloc_info.memoryTypeIndex = findMemoryType(
        impl.physical_device, mem_reqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    if (vkAllocateMemory(impl.device, &alloc_info, nullptr, &impl.msaa_color_memory) != VK_SUCCESS) {
        throw std::runtime_error("Failed to allocate MSAA color memory.");
    }
    vkBindImageMemory(impl.device, impl.msaa_color_image, impl.msaa_color_memory, 0);

    VkImageViewCreateInfo view_info{};
    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view_info.image = impl.msaa_color_image;
    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view_info.format = impl.swapchain_format;
    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    view_info.subresourceRange.levelCount = 1;
    view_info.subresourceRange.layerCount = 1;
    if (vkCreateImageView(impl.device, &view_info, nullptr, &impl.msaa_color_view) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create MSAA color image view.");
    }
}

void createRenderPass(ArchitectureViewerApp::Impl& impl) {
    const bool msaa = impl.msaa_samples != VK_SAMPLE_COUNT_1_BIT;

    VkAttachmentDescription color_attachment{};
    color_attachment.format = impl.swapchain_format;
    color_attachment.samples = msaa ? impl.msaa_samples : VK_SAMPLE_COUNT_1_BIT;
    color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color_attachment.storeOp = msaa ? VK_ATTACHMENT_STORE_OP_DONT_CARE : VK_ATTACHMENT_STORE_OP_STORE;
    color_attachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    color_attachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    color_attachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    color_attachment.finalLayout = msaa ? VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL : VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentDescription depth_attachment{};
    depth_attachment.format = impl.depth_format;
    depth_attachment.samples = msaa ? impl.msaa_samples : VK_SAMPLE_COUNT_1_BIT;
    depth_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depth_attachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depth_attachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depth_attachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depth_attachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    depth_attachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentReference color_reference{};
    color_reference.attachment = 0;
    color_reference.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentReference depth_reference{};
    depth_reference.attachment = 1;
    depth_reference.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentReference resolve_reference{};
    resolve_reference.attachment = 2;
    resolve_reference.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &color_reference;
    subpass.pDepthStencilAttachment = &depth_reference;
    if (msaa) { subpass.pResolveAttachments = &resolve_reference; }

    VkSubpassDependency dependency{};
    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass = 0;
    dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT
        | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT
        | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT
        | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    VkAttachmentDescription resolve_attachment{};
    if (msaa) {
        resolve_attachment.format = impl.swapchain_format;
        resolve_attachment.samples = VK_SAMPLE_COUNT_1_BIT;
        resolve_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        resolve_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        resolve_attachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        resolve_attachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        resolve_attachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        resolve_attachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    }

    std::vector<VkAttachmentDescription> attachments{color_attachment, depth_attachment};
    if (msaa) { attachments.push_back(resolve_attachment); }

    VkRenderPassCreateInfo render_pass_info{};
    render_pass_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    render_pass_info.attachmentCount = static_cast<std::uint32_t>(attachments.size());
    render_pass_info.pAttachments = attachments.data();
    render_pass_info.subpassCount = 1;
    render_pass_info.pSubpasses = &subpass;
    render_pass_info.dependencyCount = 1;
    render_pass_info.pDependencies = &dependency;

    if (vkCreateRenderPass(impl.device, &render_pass_info, nullptr, &impl.render_pass) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create render pass.");
    }
}

void createDescriptorSetLayout(ArchitectureViewerApp::Impl& impl) {
    VkDescriptorSetLayoutBinding binding{};
    binding.binding = 0;
    binding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    binding.descriptorCount = 1;
    binding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutCreateInfo create_info{};
    create_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    create_info.bindingCount = 1;
    create_info.pBindings = &binding;

    if (vkCreateDescriptorSetLayout(impl.device, &create_info, nullptr, &impl.descriptor_set_layout) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create descriptor set layout.");
    }
}

void createGraphicsPipeline(ArchitectureViewerApp::Impl& impl) {
    const auto vert_code = readBinaryFile(impl.compiledShaderPath("primitive.vert"));
    const auto frag_code = readBinaryFile(impl.compiledShaderPath("primitive.frag"));

    const VkShaderModule vert_module = createShaderModule(impl.device, vert_code);
    const VkShaderModule frag_module = createShaderModule(impl.device, frag_code);

    VkPipelineShaderStageCreateInfo vert_stage{};
    vert_stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vert_stage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vert_stage.module = vert_module;
    vert_stage.pName = "main";

    VkPipelineShaderStageCreateInfo frag_stage{};
    frag_stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    frag_stage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    frag_stage.module = frag_module;
    frag_stage.pName = "main";

    const std::array<VkPipelineShaderStageCreateInfo, 2> stages{vert_stage, frag_stage};
    const VkVertexInputBindingDescription binding = vertexBindingDescription();
    const auto attributes = vertexAttributeDescriptions();

    VkPipelineVertexInputStateCreateInfo vertex_input{};
    vertex_input.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertex_input.vertexBindingDescriptionCount = 1;
    vertex_input.pVertexBindingDescriptions = &binding;
    vertex_input.vertexAttributeDescriptionCount = static_cast<std::uint32_t>(attributes.size());
    vertex_input.pVertexAttributeDescriptions = attributes.data();

    VkPipelineInputAssemblyStateCreateInfo input_assembly{};
    input_assembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkViewport viewport{};
    viewport.width = static_cast<float>(impl.swapchain_extent.width);
    viewport.height = static_cast<float>(impl.swapchain_extent.height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor{};
    scissor.extent = impl.swapchain_extent;

    VkPipelineViewportStateCreateInfo viewport_state{};
    viewport_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    viewport_state.viewportCount = 1;
    viewport_state.pViewports = &viewport;
    viewport_state.scissorCount = 1;
    viewport_state.pScissors = &scissor;

    VkPipelineRasterizationStateCreateInfo rasterizer{};
    rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.lineWidth = 1.0f;
    rasterizer.cullMode = VK_CULL_MODE_BACK_BIT;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;

    VkPipelineMultisampleStateCreateInfo multisampling{};
    multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    multisampling.rasterizationSamples = impl.msaa_samples;

    VkPipelineDepthStencilStateCreateInfo depth_stencil{};
    depth_stencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    depth_stencil.depthTestEnable = VK_TRUE;
    depth_stencil.depthWriteEnable = VK_TRUE;
    depth_stencil.depthCompareOp = VK_COMPARE_OP_LESS;

    VkPipelineColorBlendAttachmentState color_blend_attachment{};
    color_blend_attachment.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

    VkPipelineColorBlendStateCreateInfo color_blending{};
    color_blending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    color_blending.attachmentCount = 1;
    color_blending.pAttachments = &color_blend_attachment;

    VkPushConstantRange push_constants{};
    push_constants.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    push_constants.offset = 0;
    push_constants.size = sizeof(PushConstants);

    VkPipelineLayoutCreateInfo layout_info{};
    layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    layout_info.setLayoutCount = 1;
    layout_info.pSetLayouts = &impl.descriptor_set_layout;
    layout_info.pushConstantRangeCount = 1;
    layout_info.pPushConstantRanges = &push_constants;

    if (vkCreatePipelineLayout(impl.device, &layout_info, nullptr, &impl.pipeline_layout) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create pipeline layout.");
    }

    VkGraphicsPipelineCreateInfo pipeline_info{};
    pipeline_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    pipeline_info.stageCount = static_cast<std::uint32_t>(stages.size());
    pipeline_info.pStages = stages.data();
    pipeline_info.pVertexInputState = &vertex_input;
    pipeline_info.pInputAssemblyState = &input_assembly;
    pipeline_info.pViewportState = &viewport_state;
    pipeline_info.pRasterizationState = &rasterizer;
    pipeline_info.pMultisampleState = &multisampling;
    pipeline_info.pDepthStencilState = &depth_stencil;
    pipeline_info.pColorBlendState = &color_blending;
    pipeline_info.layout = impl.pipeline_layout;
    pipeline_info.renderPass = impl.render_pass;
    pipeline_info.subpass = 0;

    if (vkCreateGraphicsPipelines(
            impl.device,
            VK_NULL_HANDLE,
            1,
            &pipeline_info,
            nullptr,
            &impl.graphics_pipeline) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create graphics pipeline.");
    }

    if (impl.fillmode_nonsolid_supported) {
        rasterizer.polygonMode = VK_POLYGON_MODE_LINE;
        rasterizer.cullMode = VK_CULL_MODE_NONE;
        if (vkCreateGraphicsPipelines(
                impl.device,
                VK_NULL_HANDLE,
                1,
                &pipeline_info,
                nullptr,
                &impl.wireframe_pipeline) != VK_SUCCESS) {
            std::cerr << "[ArchitectureViewerApp] Wireframe pipeline creation failed; falling back to solid.\n";
            impl.fillmode_nonsolid_supported = false;
        }
    }

    // Alpha pipeline: solid fill, no back-face culling (both sides visible for envelopes),
    // standard src-alpha blending, depth test on but no depth write (transparent objects last).
    {
        rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
        rasterizer.cullMode = VK_CULL_MODE_NONE;

        depth_stencil.depthWriteEnable = VK_FALSE;

        color_blend_attachment.blendEnable = VK_TRUE;
        color_blend_attachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        color_blend_attachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        color_blend_attachment.colorBlendOp        = VK_BLEND_OP_ADD;
        color_blend_attachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        color_blend_attachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
        color_blend_attachment.alphaBlendOp        = VK_BLEND_OP_ADD;

        if (vkCreateGraphicsPipelines(
                impl.device,
                VK_NULL_HANDLE,
                1,
                &pipeline_info,
                nullptr,
                &impl.alpha_pipeline) != VK_SUCCESS) {
            std::cerr << "[ArchitectureViewerApp] Alpha pipeline creation failed; transparent objects will be opaque.\n";
        }
    }

    vkDestroyShaderModule(impl.device, frag_module, nullptr);
    vkDestroyShaderModule(impl.device, vert_module, nullptr);
}

void createDepthResources(ArchitectureViewerApp::Impl& impl) {
    VkImageCreateInfo image_info{};
    image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image_info.imageType = VK_IMAGE_TYPE_2D;
    image_info.extent.width = impl.swapchain_extent.width;
    image_info.extent.height = impl.swapchain_extent.height;
    image_info.extent.depth = 1;
    image_info.mipLevels = 1;
    image_info.arrayLayers = 1;
    image_info.format = impl.depth_format;
    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    image_info.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    image_info.samples = impl.msaa_samples;
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (vkCreateImage(impl.device, &image_info, nullptr, &impl.depth_image) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create depth image.");
    }

    VkMemoryRequirements memory_requirements{};
    vkGetImageMemoryRequirements(impl.device, impl.depth_image, &memory_requirements);

    VkMemoryAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc_info.allocationSize = memory_requirements.size;
    alloc_info.memoryTypeIndex = findMemoryType(
        impl.physical_device,
        memory_requirements.memoryTypeBits,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    if (vkAllocateMemory(impl.device, &alloc_info, nullptr, &impl.depth_memory) != VK_SUCCESS) {
        throw std::runtime_error("Failed to allocate depth memory.");
    }
    vkBindImageMemory(impl.device, impl.depth_image, impl.depth_memory, 0);

    VkImageViewCreateInfo view_info{};
    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view_info.image = impl.depth_image;
    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view_info.format = impl.depth_format;
    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    view_info.subresourceRange.levelCount = 1;
    view_info.subresourceRange.layerCount = 1;

    if (vkCreateImageView(impl.device, &view_info, nullptr, &impl.depth_image_view) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create depth image view.");
    }
}

void createFramebuffers(ArchitectureViewerApp::Impl& impl) {
    const bool msaa = impl.msaa_samples != VK_SAMPLE_COUNT_1_BIT;
    impl.swapchain_framebuffers.resize(impl.swapchain_image_views.size());
    for (std::size_t index = 0; index < impl.swapchain_image_views.size(); ++index) {
        std::vector<VkImageView> attachments;
        if (msaa) {
            attachments = {impl.msaa_color_view, impl.depth_image_view, impl.swapchain_image_views[index]};
        } else {
            attachments = {impl.swapchain_image_views[index], impl.depth_image_view};
        }

        VkFramebufferCreateInfo framebuffer_info{};
        framebuffer_info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        framebuffer_info.renderPass = impl.render_pass;
        framebuffer_info.attachmentCount = static_cast<std::uint32_t>(attachments.size());
        framebuffer_info.pAttachments = attachments.data();
        framebuffer_info.width = impl.swapchain_extent.width;
        framebuffer_info.height = impl.swapchain_extent.height;
        framebuffer_info.layers = 1;

        if (vkCreateFramebuffer(impl.device, &framebuffer_info, nullptr, &impl.swapchain_framebuffers[index]) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create framebuffer.");
        }
    }
}

void createPerImagePresentSemaphores(ArchitectureViewerApp::Impl& impl) {
    VkSemaphoreCreateInfo semaphore_info{};
    semaphore_info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
    impl.render_finished_semaphores.resize(impl.swapchain_images.size(), VK_NULL_HANDLE);
    for (auto& semaphore : impl.render_finished_semaphores) {
        if (vkCreateSemaphore(impl.device, &semaphore_info, nullptr, &semaphore) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create render-finished semaphore.");
        }
    }
}

void createDescriptorPoolAndSets(ArchitectureViewerApp::Impl& impl) {
    VkDescriptorPoolSize pool_size{};
    pool_size.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    pool_size.descriptorCount = static_cast<std::uint32_t>(kFramesInFlight);

    VkDescriptorPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool_info.poolSizeCount = 1;
    pool_info.pPoolSizes = &pool_size;
    pool_info.maxSets = static_cast<std::uint32_t>(kFramesInFlight);

    if (vkCreateDescriptorPool(impl.device, &pool_info, nullptr, &impl.descriptor_pool) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create descriptor pool.");
    }

    std::array<VkDescriptorSetLayout, kFramesInFlight> layouts{};
    layouts.fill(impl.descriptor_set_layout);

    VkDescriptorSetAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    alloc_info.descriptorPool = impl.descriptor_pool;
    alloc_info.descriptorSetCount = static_cast<std::uint32_t>(layouts.size());
    alloc_info.pSetLayouts = layouts.data();

    std::array<VkDescriptorSet, kFramesInFlight> descriptor_sets{};
    if (vkAllocateDescriptorSets(impl.device, &alloc_info, descriptor_sets.data()) != VK_SUCCESS) {
        throw std::runtime_error("Failed to allocate descriptor sets.");
    }

    for (std::size_t index = 0; index < kFramesInFlight; ++index) {
        impl.frames[index].descriptor_set = descriptor_sets[index];

        VkDescriptorBufferInfo buffer_info{};
        buffer_info.buffer = impl.frames[index].uniform_buffer.buffer;
        buffer_info.offset = 0;
        buffer_info.range = sizeof(GlobalUniformData);

        VkWriteDescriptorSet write{};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstSet = impl.frames[index].descriptor_set;
        write.dstBinding = 0;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        write.pBufferInfo = &buffer_info;
        vkUpdateDescriptorSets(impl.device, 1, &write, 0, nullptr);
    }
}

void createFrameResources(ArchitectureViewerApp::Impl& impl) {
    VkCommandPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    pool_info.queueFamilyIndex = impl.queue_families.graphics_family.value();

    VkSemaphoreCreateInfo semaphore_info{};
    semaphore_info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

    VkFenceCreateInfo fence_info{};
    fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fence_info.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    for (auto& frame : impl.frames) {
        if (vkCreateCommandPool(impl.device, &pool_info, nullptr, &frame.command_pool) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create command pool.");
        }

        VkCommandBufferAllocateInfo alloc_info{};
        alloc_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        alloc_info.commandPool = frame.command_pool;
        alloc_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        alloc_info.commandBufferCount = 1;
        if (vkAllocateCommandBuffers(impl.device, &alloc_info, &frame.command_buffer) != VK_SUCCESS) {
            throw std::runtime_error("Failed to allocate command buffer.");
        }

        if (vkCreateSemaphore(impl.device, &semaphore_info, nullptr, &frame.image_available) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create image-available semaphore.");
        }
        if (vkCreateFence(impl.device, &fence_info, nullptr, &frame.in_flight) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create in-flight fence.");
        }

        createBuffer(
            impl.physical_device,
            impl.device,
            sizeof(GlobalUniformData),
            VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
            frame.uniform_buffer);

        vkMapMemory(
            impl.device,
            frame.uniform_buffer.memory,
            0,
            frame.uniform_buffer.size,
            0,
            &frame.uniform_buffer.mapped);
    }
}

void createTransferPool(ArchitectureViewerApp::Impl& impl) {
    VkCommandPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    pool_info.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
    pool_info.queueFamilyIndex = impl.queue_families.graphics_family.value();
    if (vkCreateCommandPool(impl.device, &pool_info, nullptr, &impl.transfer_pool) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create transfer command pool.");
    }
}

void uploadMesh(
    ArchitectureViewerApp::Impl& impl,
    const MeshData& mesh,
    MeshResource& out_mesh) {
    const VkDeviceSize vertex_buffer_size = sizeof(MeshVertex) * mesh.vertices.size();
    const VkDeviceSize index_buffer_size = sizeof(std::uint32_t) * mesh.indices.size();

    BufferResource staging_vertex;
    createBuffer(
        impl.physical_device,
        impl.device,
        vertex_buffer_size,
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        staging_vertex);

    BufferResource staging_index;
    createBuffer(
        impl.physical_device,
        impl.device,
        index_buffer_size,
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        staging_index);

    void* mapped = nullptr;
    vkMapMemory(impl.device, staging_vertex.memory, 0, vertex_buffer_size, 0, &mapped);
    std::memcpy(mapped, mesh.vertices.data(), static_cast<std::size_t>(vertex_buffer_size));
    vkUnmapMemory(impl.device, staging_vertex.memory);

    vkMapMemory(impl.device, staging_index.memory, 0, index_buffer_size, 0, &mapped);
    std::memcpy(mapped, mesh.indices.data(), static_cast<std::size_t>(index_buffer_size));
    vkUnmapMemory(impl.device, staging_index.memory);

    createBuffer(
        impl.physical_device,
        impl.device,
        vertex_buffer_size,
        VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        out_mesh.vertex_buffer);

    createBuffer(
        impl.physical_device,
        impl.device,
        index_buffer_size,
        VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        out_mesh.index_buffer);

    const VkCommandBuffer command_buffer = beginSingleTimeCommands(impl.device, impl.transfer_pool);

    VkBufferCopy vertex_copy{};
    vertex_copy.size = vertex_buffer_size;
    vkCmdCopyBuffer(command_buffer, staging_vertex.buffer, out_mesh.vertex_buffer.buffer, 1, &vertex_copy);

    VkBufferCopy index_copy{};
    index_copy.size = index_buffer_size;
    vkCmdCopyBuffer(command_buffer, staging_index.buffer, out_mesh.index_buffer.buffer, 1, &index_copy);

    endSingleTimeCommands(impl.device, impl.graphics_queue, impl.transfer_pool, command_buffer);

    destroyBuffer(impl.device, staging_vertex);
    destroyBuffer(impl.device, staging_index);

    out_mesh.index_count = static_cast<std::uint32_t>(mesh.indices.size());
}

fs::path resolveAssetRoot() {
    std::vector<fs::path> candidates;
    candidates.push_back(fs::current_path() / "asset");
#if defined(_WIN32)
    char buf[MAX_PATH]{};
    if (GetModuleFileNameA(nullptr, buf, MAX_PATH) > 0) {
        const fs::path exe_dir = fs::path(buf).parent_path();
        candidates.push_back(exe_dir / "asset");
        candidates.push_back(exe_dir.parent_path() / "asset");
        candidates.push_back(exe_dir.parent_path().parent_path() / "asset");
        candidates.push_back(exe_dir.parent_path().parent_path().parent_path() / "asset");
    }
#endif
    const fs::path probe = "ue4_mannequin_base_mesh" / fs::path("scene.gltf");
    for (const auto& c : candidates) {
        if (fs::exists(c / probe)) { return c; }
    }
    return {};
}

void loadMannequin(ArchitectureViewerApp::Impl& impl) {
    const fs::path asset_root = resolveAssetRoot();
    if (asset_root.empty()) {
        std::cerr << "[ArchitectureViewerApp] Human reference model not found; skipping.\n";
        return;
    }
    const fs::path gltf = asset_root / "ue4_mannequin_base_mesh" / "scene.gltf";
    try {
        // UE4 mannequin: Y height ≈ 182.53 cm → scale to exactly 1.80 m
        constexpr float kMannequinHeight = 182.53f;
        const float scale = 1.80f / kMannequinHeight;
        const auto mesh_data = loadGltfMeshes(gltf, scale, true);
        for (const auto& md : mesh_data) {
            MeshResource resource;
            uploadMesh(impl, md, resource);
            resource.material_name = md.material_name;
            impl.mannequin_meshes.push_back(std::move(resource));
        }
        impl.mannequin_loaded = !impl.mannequin_meshes.empty();
        if (impl.mannequin_loaded) {
            std::cout << "[ArchitectureViewerApp] Human reference loaded ("
                      << impl.mannequin_meshes.size() << " meshes).\n";
            PrimitiveInstance phantom;
            phantom.primitive_type = core::GeometryPrimitive::Kind::box;
            phantom.world_transform = Eigen::Isometry3d::Identity();
            phantom.world_transform.translation() = impl.mannequin_world_pos
                + Eigen::Vector3d(0.0, 0.0, 0.9);
            phantom.dimensions = Eigen::Vector3d(0.35, 0.15, 0.9);
            phantom.color = {0.0f, 0.0f, 0.0f, 0.0f};
            phantom.visible = false;
            phantom.source_element_type = "HumanReference";
            impl.instances.push_back(phantom);
        }
    } catch (const std::exception& ex) {
        std::cerr << "[ArchitectureViewerApp] Failed to load human reference: " << ex.what() << "\n";
    }
}

void createMeshes(ArchitectureViewerApp::Impl& impl) {
    impl.meshes.clear();

    MeshResource box;
    uploadMesh(impl, PrimitiveMeshFactory::makeUnitBox(), box);
    impl.meshes.emplace(core::GeometryPrimitive::Kind::box, std::move(box));

    MeshResource sphere;
    uploadMesh(impl, PrimitiveMeshFactory::makeUnitSphere(), sphere);
    impl.meshes.emplace(core::GeometryPrimitive::Kind::sphere, std::move(sphere));

    MeshResource cylinder;
    uploadMesh(impl, PrimitiveMeshFactory::makeUnitCylinder(), cylinder);
    impl.meshes.emplace(core::GeometryPrimitive::Kind::cylinder, std::move(cylinder));

    MeshResource disk;
    uploadMesh(impl, PrimitiveMeshFactory::makeUnitDisk(), disk);
    impl.meshes.emplace(core::GeometryPrimitive::Kind::disk, std::move(disk));

    MeshResource segment;
    uploadMesh(impl, PrimitiveMeshFactory::makeUnitSegmentProxy(), segment);
    impl.meshes.emplace(core::GeometryPrimitive::Kind::segment, std::move(segment));

    loadMannequin(impl);
}

void cleanupMeshes(ArchitectureViewerApp::Impl& impl) {
    for (auto& [_, mesh] : impl.meshes) {
        destroyBuffer(impl.device, mesh.vertex_buffer);
        destroyBuffer(impl.device, mesh.index_buffer);
    }
    impl.meshes.clear();
    for (auto& mesh : impl.mannequin_meshes) {
        destroyBuffer(impl.device, mesh.vertex_buffer);
        destroyBuffer(impl.device, mesh.index_buffer);
    }
    impl.mannequin_meshes.clear();
    impl.mannequin_loaded = false;
}

void updateUniformBuffer(ArchitectureViewerApp::Impl& impl, const std::size_t frame_index) {
    GlobalUniformData uniforms{};
    const auto view = toFloatArray(impl.camera.viewMatrix());
    const auto projection = toFloatArray(impl.camera.projectionMatrix());
    std::memcpy(uniforms.view, view.data(), sizeof(uniforms.view));
    std::memcpy(uniforms.projection, projection.data(), sizeof(uniforms.projection));
    std::memcpy(impl.frames[frame_index].uniform_buffer.mapped, &uniforms, sizeof(uniforms));
}

void recordCommandBuffer(
    ArchitectureViewerApp::Impl& impl,
    const std::size_t frame_index,
    const std::uint32_t image_index) {
    VkCommandBuffer command_buffer = impl.frames[frame_index].command_buffer;
    vkResetCommandBuffer(command_buffer, 0);

    VkCommandBufferBeginInfo begin_info{};
    begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    if (vkBeginCommandBuffer(command_buffer, &begin_info) != VK_SUCCESS) {
        throw std::runtime_error("Failed to begin command buffer.");
    }

    std::array<VkClearValue, 3> clear_values{};
    clear_values[0].color = {{0.93f, 0.95f, 0.98f, 1.0f}};
    clear_values[1].depthStencil = {1.0f, 0};
    clear_values[2].color = {{0.93f, 0.95f, 0.98f, 1.0f}};
    const std::uint32_t clear_count = (impl.msaa_samples != VK_SAMPLE_COUNT_1_BIT) ? 3U : 2U;

    VkRenderPassBeginInfo render_pass_info{};
    render_pass_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    render_pass_info.renderPass = impl.render_pass;
    render_pass_info.framebuffer = impl.swapchain_framebuffers[image_index];
    render_pass_info.renderArea.offset = {0, 0};
    render_pass_info.renderArea.extent = impl.swapchain_extent;
    render_pass_info.clearValueCount = clear_count;
    render_pass_info.pClearValues = clear_values.data();

    vkCmdBeginRenderPass(command_buffer, &render_pass_info, VK_SUBPASS_CONTENTS_INLINE);

    vkCmdBindDescriptorSets(
        command_buffer,
        VK_PIPELINE_BIND_POINT_GRAPHICS,
        impl.pipeline_layout,
        0,
        1,
        &impl.frames[frame_index].descriptor_set,
        0,
        nullptr);

    // Two-pass rendering: opaque geometry first, then transparent (envelope) geometry.
    // Transparent instances use the alpha_pipeline (blending on, depth write off).
    const auto drawInstance = [&](const PrimitiveInstance& instance, VkPipeline& bound_pipeline) {
        if (!instance.visible) { return; }
        const auto mesh_it = impl.meshes.find(instance.primitive_type);
        if (mesh_it == impl.meshes.end()) { return; }

        const bool is_transparent = instance.color[3] < 0.99f;
        VkPipeline active_pipeline;
        if ((instance.wireframe || impl.ui_wireframe) && impl.wireframe_pipeline != VK_NULL_HANDLE) {
            active_pipeline = impl.wireframe_pipeline;
        } else if (is_transparent && impl.alpha_pipeline != VK_NULL_HANDLE) {
            active_pipeline = impl.alpha_pipeline;
        } else {
            active_pipeline = impl.graphics_pipeline;
        }

        if (active_pipeline != bound_pipeline) {
            vkCmdBindPipeline(command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, active_pipeline);
            bound_pipeline = active_pipeline;
        }

        const MeshResource& mesh = mesh_it->second;
        const VkBuffer vertex_buffers[] = {mesh.vertex_buffer.buffer};
        constexpr VkDeviceSize offsets[] = {0};
        vkCmdBindVertexBuffers(command_buffer, 0, 1, vertex_buffers, offsets);
        vkCmdBindIndexBuffer(command_buffer, mesh.index_buffer.buffer, 0, VK_INDEX_TYPE_UINT32);

        PushConstants push{};
        const auto model = toFloatArray(modelMatrixForInstance(instance));
        std::memcpy(push.model, model.data(), sizeof(push.model));
        std::memcpy(push.color, instance.color.data(), sizeof(push.color));

        vkCmdPushConstants(
            command_buffer,
            impl.pipeline_layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0,
            sizeof(PushConstants),
            &push);

        vkCmdDrawIndexed(command_buffer, mesh.index_count, 1, 0, 0, 0);
    };

    VkPipeline bound_pipeline = VK_NULL_HANDLE;
    for (const auto& instance : impl.instances) {
        if (instance.color[3] >= 0.99f) { drawInstance(instance, bound_pipeline); }
    }

    // Human reference mannequin: opaque, color driven by glTF material name.
    if (impl.mannequin_loaded) {
        // Neutral mannequin palette — both parts stay in the same warm-gray family
        // so the figure reads as a unified silhouette against the coloured vehicle.
        constexpr std::array<float, 4> kSkinColor  = {0.80f, 0.76f, 0.72f, 1.0f};  // light warm gray (body)
        constexpr std::array<float, 4> kMetalColor = {0.48f, 0.48f, 0.52f, 1.0f};  // cool mid-gray (hardware)
        if (impl.graphics_pipeline != bound_pipeline) {
            vkCmdBindPipeline(command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, impl.graphics_pipeline);
            bound_pipeline = impl.graphics_pipeline;
        }
        PushConstants mann_push{};
        mann_push.model[0] = mann_push.model[5] = mann_push.model[10] = mann_push.model[15] = 1.0f;
        mann_push.model[12] = static_cast<float>(impl.mannequin_world_pos.x());
        mann_push.model[13] = static_cast<float>(impl.mannequin_world_pos.y());
        mann_push.model[14] = static_cast<float>(impl.mannequin_world_pos.z());

        for (const auto& mesh : impl.mannequin_meshes) {
            const VkBuffer vbufs[] = {mesh.vertex_buffer.buffer};
            constexpr VkDeviceSize voffsets[] = {0};
            vkCmdBindVertexBuffers(command_buffer, 0, 1, vbufs, voffsets);
            vkCmdBindIndexBuffer(command_buffer, mesh.index_buffer.buffer, 0, VK_INDEX_TYPE_UINT32);
            const bool is_metal = (mesh.material_name.find("Metal") != std::string::npos
                                || mesh.material_name.find("metal") != std::string::npos);
            const auto& col = is_metal ? kMetalColor : kSkinColor;
            std::memcpy(mann_push.color, col.data(), sizeof(mann_push.color));
            vkCmdPushConstants(command_buffer, impl.pipeline_layout,
                VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                0, sizeof(PushConstants), &mann_push);
            vkCmdDrawIndexed(command_buffer, mesh.index_count, 1, 0, 0, 0);
        }
    }

    for (const auto& instance : impl.instances) {
        if (instance.color[3] < 0.99f) { drawInstance(instance, bound_pipeline); }
    }

#ifdef HEXAARCH_HAS_IMGUI
    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), command_buffer);
#endif

    vkCmdEndRenderPass(command_buffer);
    if (vkEndCommandBuffer(command_buffer) != VK_SUCCESS) {
        throw std::runtime_error("Failed to record command buffer.");
    }
}

void recreateSwapchain(ArchitectureViewerApp::Impl& impl) {
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(impl.window, &width, &height);
    while (width == 0 || height == 0) {
        glfwGetFramebufferSize(impl.window, &width, &height);
        glfwWaitEvents();
    }

    vkDeviceWaitIdle(impl.device);
    destroySwapchainResources(impl);
    createSwapchain(impl);
    createSwapchainImageViews(impl);
    createColorResources(impl);
    createRenderPass(impl);
    createGraphicsPipeline(impl);
    createDepthResources(impl);
    createFramebuffers(impl);
    createPerImagePresentSemaphores(impl);

    impl.camera.setViewport(static_cast<float>(impl.swapchain_extent.width), static_cast<float>(impl.swapchain_extent.height));
    impl.framebuffer_resized = false;
}

void updateInput(ArchitectureViewerApp::Impl& impl) {
    glfwPollEvents();

    double cursor_x = 0.0;
    double cursor_y = 0.0;
    glfwGetCursorPos(impl.window, &cursor_x, &cursor_y);
    if (impl.first_mouse) {
        impl.last_cursor_x = cursor_x;
        impl.last_cursor_y = cursor_y;
        impl.first_mouse = false;
    }

    const double delta_x = cursor_x - impl.last_cursor_x;
    const double delta_y = cursor_y - impl.last_cursor_y;
    impl.last_cursor_x = cursor_x;
    impl.last_cursor_y = cursor_y;

#ifdef HEXAARCH_HAS_IMGUI
    if (ImGui::GetIO().WantCaptureMouse) { return; }
#endif
    if (glfwGetMouseButton(impl.window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        impl.camera.orbit(-delta_x * kOrbitSpeed, -delta_y * kOrbitSpeed);
    }
    if (glfwGetMouseButton(impl.window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
        const double pan_scale = std::max(impl.camera.distance(), 1.0) * kPanSpeed;
        const Eigen::Vector3d delta_world =
            -cameraRight(impl.camera) * (delta_x * pan_scale)
            + cameraUp(impl.camera) * (delta_y * pan_scale);
        impl.camera.pan(delta_world);
    }

    if (glfwGetKey(impl.window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(impl.window, GLFW_TRUE);
    }

    // WASD+QE: move human reference model in world space (0.05 m/frame ≈ 3 m/s at 60 fps).
    if (impl.mannequin_loaded) {
#ifdef HEXAARCH_HAS_IMGUI
        if (!ImGui::GetIO().WantCaptureKeyboard) {
#endif
        constexpr double kStep = 0.01;
        if (glfwGetKey(impl.window, GLFW_KEY_W) == GLFW_PRESS) impl.mannequin_world_pos.y() += kStep;
        if (glfwGetKey(impl.window, GLFW_KEY_S) == GLFW_PRESS) impl.mannequin_world_pos.y() -= kStep;
        if (glfwGetKey(impl.window, GLFW_KEY_A) == GLFW_PRESS) impl.mannequin_world_pos.x() -= kStep;
        if (glfwGetKey(impl.window, GLFW_KEY_D) == GLFW_PRESS) impl.mannequin_world_pos.x() += kStep;
        if (glfwGetKey(impl.window, GLFW_KEY_Q) == GLFW_PRESS) impl.mannequin_world_pos.z() += kStep;
        if (glfwGetKey(impl.window, GLFW_KEY_E) == GLFW_PRESS) impl.mannequin_world_pos.z() -= kStep;
#ifdef HEXAARCH_HAS_IMGUI
        }
#endif
    }
}

void drawFrame(ArchitectureViewerApp::Impl& impl) {
    FrameResources& frame = impl.frames[impl.current_frame];
    vkWaitForFences(impl.device, 1, &frame.in_flight, VK_TRUE, UINT64_MAX);

    std::uint32_t image_index = 0;
    VkResult result = vkAcquireNextImageKHR(
        impl.device,
        impl.swapchain,
        UINT64_MAX,
        frame.image_available,
        VK_NULL_HANDLE,
        &image_index);

    if (result == VK_ERROR_OUT_OF_DATE_KHR) {
        recreateSwapchain(impl);
        return;
    }
    if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
        throw std::runtime_error("Failed to acquire swapchain image.");
    }

    if (impl.images_in_flight[image_index] != VK_NULL_HANDLE) {
        vkWaitForFences(impl.device, 1, &impl.images_in_flight[image_index], VK_TRUE, UINT64_MAX);
    }
    impl.images_in_flight[image_index] = frame.in_flight;

    updateUniformBuffer(impl, impl.current_frame);
    recordCommandBuffer(impl, impl.current_frame, image_index);

    vkResetFences(impl.device, 1, &frame.in_flight);

    constexpr VkPipelineStageFlags wait_stage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    VkSubmitInfo submit_info{};
    submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit_info.waitSemaphoreCount = 1;
    submit_info.pWaitSemaphores = &frame.image_available;
    submit_info.pWaitDstStageMask = &wait_stage;
    submit_info.commandBufferCount = 1;
    submit_info.pCommandBuffers = &frame.command_buffer;
    submit_info.signalSemaphoreCount = 1;
    submit_info.pSignalSemaphores = &impl.render_finished_semaphores[image_index];

    if (vkQueueSubmit(impl.graphics_queue, 1, &submit_info, frame.in_flight) != VK_SUCCESS) {
        throw std::runtime_error("Failed to submit draw command buffer.");
    }

    VkPresentInfoKHR present_info{};
    present_info.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    present_info.waitSemaphoreCount = 1;
    present_info.pWaitSemaphores = &impl.render_finished_semaphores[image_index];
    present_info.swapchainCount = 1;
    present_info.pSwapchains = &impl.swapchain;
    present_info.pImageIndices = &image_index;

    result = vkQueuePresentKHR(impl.present_queue, &present_info);
    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || impl.framebuffer_resized) {
        recreateSwapchain(impl);
    } else if (result != VK_SUCCESS) {
        throw std::runtime_error("Failed to present swapchain image.");
    }

    impl.current_frame = (impl.current_frame + 1) % kFramesInFlight;
}

void cleanupViewer(ArchitectureViewerApp::Impl& impl) {
    if (impl.device != VK_NULL_HANDLE) {
        vkDeviceWaitIdle(impl.device);
        cleanupImGui(impl);
        cleanupMeshes(impl);
        destroySwapchainResources(impl);
    }

    for (auto& frame : impl.frames) {
        if (impl.device != VK_NULL_HANDLE) {
            destroyBuffer(impl.device, frame.uniform_buffer);
        }
        if (impl.device != VK_NULL_HANDLE && frame.image_available != VK_NULL_HANDLE) {
            vkDestroySemaphore(impl.device, frame.image_available, nullptr);
            frame.image_available = VK_NULL_HANDLE;
        }
        if (impl.device != VK_NULL_HANDLE && frame.in_flight != VK_NULL_HANDLE) {
            vkDestroyFence(impl.device, frame.in_flight, nullptr);
            frame.in_flight = VK_NULL_HANDLE;
        }
        if (impl.device != VK_NULL_HANDLE && frame.command_pool != VK_NULL_HANDLE) {
            vkDestroyCommandPool(impl.device, frame.command_pool, nullptr);
            frame.command_pool = VK_NULL_HANDLE;
        }
    }

    if (impl.device != VK_NULL_HANDLE && impl.descriptor_pool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(impl.device, impl.descriptor_pool, nullptr);
        impl.descriptor_pool = VK_NULL_HANDLE;
    }
    if (impl.device != VK_NULL_HANDLE && impl.descriptor_set_layout != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(impl.device, impl.descriptor_set_layout, nullptr);
        impl.descriptor_set_layout = VK_NULL_HANDLE;
    }
    if (impl.device != VK_NULL_HANDLE && impl.transfer_pool != VK_NULL_HANDLE) {
        vkDestroyCommandPool(impl.device, impl.transfer_pool, nullptr);
        impl.transfer_pool = VK_NULL_HANDLE;
    }
    if (impl.device != VK_NULL_HANDLE) {
        vkDestroyDevice(impl.device, nullptr);
        impl.device = VK_NULL_HANDLE;
    }
    if (impl.surface != VK_NULL_HANDLE) {
        vkDestroySurfaceKHR(impl.instance, impl.surface, nullptr);
        impl.surface = VK_NULL_HANDLE;
    }
    if (impl.debug_messenger != VK_NULL_HANDLE) {
        vkDestroyDebugUtilsMessengerEXT(impl.instance, impl.debug_messenger, nullptr);
        impl.debug_messenger = VK_NULL_HANDLE;
    }
    if (impl.instance != VK_NULL_HANDLE) {
        vkDestroyInstance(impl.instance, nullptr);
        impl.instance = VK_NULL_HANDLE;
    }
    if (impl.window != nullptr) {
        glfwDestroyWindow(impl.window);
        impl.window = nullptr;
    }
    glfwTerminate();
}

void initWindow(ArchitectureViewerApp::Impl& impl) {
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW.");
    }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    impl.window = glfwCreateWindow(impl.config.width, impl.config.height, impl.config.title.c_str(), nullptr, nullptr);
    if (impl.window == nullptr) {
        throw std::runtime_error("Failed to create GLFW window.");
    }
    glfwSetWindowUserPointer(impl.window, &impl);
    glfwSetFramebufferSizeCallback(impl.window, ArchitectureViewerApp::Impl::framebufferResizeCallback);
    glfwSetScrollCallback(impl.window, ArchitectureViewerApp::Impl::scrollCallback);
}

void initInstance(ArchitectureViewerApp::Impl& impl) {
    if (volkInitialize() != VK_SUCCESS) {
        throw std::runtime_error("Failed to initialize Volk.");
    }

    VkApplicationInfo app_info{};
    app_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    app_info.pApplicationName = impl.config.title.c_str();
    app_info.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    app_info.pEngineName = "HexaArch";
    app_info.engineVersion = VK_MAKE_VERSION(1, 0, 0);
    app_info.apiVersion = VK_API_VERSION_1_0;

    std::uint32_t glfw_extension_count = 0;
    const char** glfw_extensions = glfwGetRequiredInstanceExtensions(&glfw_extension_count);
    std::vector<const char*> extensions(glfw_extensions, glfw_extensions + glfw_extension_count);

    constexpr const char* kValidationLayer = "VK_LAYER_KHRONOS_validation";
    bool validation_available = false;
    if (impl.config.validation_layers) {
        std::uint32_t layer_count = 0;
        vkEnumerateInstanceLayerProperties(&layer_count, nullptr);
        std::vector<VkLayerProperties> layers(layer_count);
        vkEnumerateInstanceLayerProperties(&layer_count, layers.data());
        for (const auto& layer : layers) {
            if (std::string_view(layer.layerName) == kValidationLayer) {
                validation_available = true;
                break;
            }
        }
        if (!validation_available) {
            std::cerr << "[ArchitectureViewerApp] Validation layers requested but VK_LAYER_KHRONOS_validation is not available.\n";
        } else {
            extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
        }
    }

    VkInstanceCreateInfo create_info{};
    create_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    create_info.pApplicationInfo = &app_info;
    create_info.enabledExtensionCount = static_cast<std::uint32_t>(extensions.size());
    create_info.ppEnabledExtensionNames = extensions.data();
    if (validation_available) {
        create_info.enabledLayerCount = 1;
        create_info.ppEnabledLayerNames = &kValidationLayer;
    }

    if (vkCreateInstance(&create_info, nullptr, &impl.instance) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create Vulkan instance.");
    }
    volkLoadInstance(impl.instance);

    if (validation_available) {
        VkDebugUtilsMessengerCreateInfoEXT messenger_info{};
        messenger_info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
        messenger_info.messageSeverity =
            VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
            VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        messenger_info.messageType =
            VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
            VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
            VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
        messenger_info.pfnUserCallback = debugCallback;
        if (vkCreateDebugUtilsMessengerEXT(impl.instance, &messenger_info, nullptr, &impl.debug_messenger) != VK_SUCCESS) {
            std::cerr << "[ArchitectureViewerApp] Failed to create debug messenger.\n";
        }
    }
}

void initSurface(ArchitectureViewerApp::Impl& impl) {
    if (glfwCreateWindowSurface(impl.instance, impl.window, nullptr, &impl.surface) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create Vulkan surface.");
    }
}

void pickPhysicalDevice(ArchitectureViewerApp::Impl& impl) {
    std::uint32_t device_count = 0;
    vkEnumeratePhysicalDevices(impl.instance, &device_count, nullptr);
    if (device_count == 0) {
        throw std::runtime_error("No Vulkan physical devices found.");
    }

    std::vector<VkPhysicalDevice> devices(device_count);
    vkEnumeratePhysicalDevices(impl.instance, &device_count, devices.data());

    for (const auto device : devices) {
        if (isDeviceSuitable(device, impl.surface)) {
            impl.physical_device = device;
            impl.queue_families = findQueueFamilies(device, impl.surface);
            impl.depth_format = findDepthFormat(device);
            VkPhysicalDeviceFeatures features{};
            vkGetPhysicalDeviceFeatures(device, &features);
            impl.fillmode_nonsolid_supported = features.fillModeNonSolid == VK_TRUE;
            impl.msaa_samples = pickMsaaSamples(device);
            return;
        }
    }
    throw std::runtime_error("Failed to find a suitable Vulkan device.");
}

void createLogicalDevice(ArchitectureViewerApp::Impl& impl) {
    std::set<std::uint32_t> unique_queue_families{
        impl.queue_families.graphics_family.value(),
        impl.queue_families.present_family.value()};

    float queue_priority = 1.0f;
    std::vector<VkDeviceQueueCreateInfo> queue_infos;
    queue_infos.reserve(unique_queue_families.size());
    for (const auto queue_family : unique_queue_families) {
        VkDeviceQueueCreateInfo queue_info{};
        queue_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        queue_info.queueFamilyIndex = queue_family;
        queue_info.queueCount = 1;
        queue_info.pQueuePriorities = &queue_priority;
        queue_infos.push_back(queue_info);
    }

    VkPhysicalDeviceFeatures device_features{};
    if (impl.fillmode_nonsolid_supported) {
        device_features.fillModeNonSolid = VK_TRUE;
    }

    const char* extensions[] = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
    VkDeviceCreateInfo create_info{};
    create_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    create_info.queueCreateInfoCount = static_cast<std::uint32_t>(queue_infos.size());
    create_info.pQueueCreateInfos = queue_infos.data();
    create_info.pEnabledFeatures = &device_features;
    create_info.enabledExtensionCount = 1;
    create_info.ppEnabledExtensionNames = extensions;

    if (vkCreateDevice(impl.physical_device, &create_info, nullptr, &impl.device) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create logical device.");
    }
    volkLoadDevice(impl.device);

    vkGetDeviceQueue(impl.device, impl.queue_families.graphics_family.value(), 0, &impl.graphics_queue);
    vkGetDeviceQueue(impl.device, impl.queue_families.present_family.value(), 0, &impl.present_queue);
}

void initializeSceneCamera(ArchitectureViewerApp::Impl& impl) {
    const Eigen::Vector3d center = sceneCenter(impl.instances);
    const double radius = sceneRadius(impl.instances, center);
    impl.camera.setViewport(static_cast<float>(impl.swapchain_extent.width), static_cast<float>(impl.swapchain_extent.height));
    impl.camera.setPerspective(60.0f * 3.14159265358979323846f / 180.0f, 0.1f, 1000.0f);
    impl.camera.reset(center, std::max(radius * 2.5, 6.0));
}

void initViewerRuntime(ArchitectureViewerApp::Impl& impl) {
    impl.shader_root = resolveShaderRoot();
    initWindow(impl);
    initInstance(impl);
    initSurface(impl);
    pickPhysicalDevice(impl);
    createLogicalDevice(impl);
    createDescriptorSetLayout(impl);
    createFrameResources(impl);
    createTransferPool(impl);
    createDescriptorPoolAndSets(impl);
    createSwapchain(impl);
    createSwapchainImageViews(impl);
    createColorResources(impl);
    createRenderPass(impl);
    createGraphicsPipeline(impl);
    createDepthResources(impl);
    createFramebuffers(impl);
    createPerImagePresentSemaphores(impl);
    createMeshes(impl);
    initializeSceneCamera(impl);
    initializeImGui(impl);
}

void applyVisibilityFlags(ArchitectureViewerApp::Impl& impl) {
    for (auto& instance : impl.instances) {
        if (instance.source_element_type == "ReferenceAxis") {
            instance.visible = impl.ui_show_axes;
        } else if (instance.source_element_type == "ReferenceGrid") {
            instance.visible = impl.ui_show_grid;
        } else {
            const auto it = impl.element_visibility.find(instance.source_element_id);
            instance.visible = (it == impl.element_visibility.end()) || it->second;
        }
    }
}

void initializeImGui(ArchitectureViewerApp::Impl& impl) {
#ifdef HEXAARCH_HAS_IMGUI
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui::GetStyle().WindowRounding = 6.0f;
    ImGui::GetStyle().Alpha = 0.92f;

    // Detect Windows content scale (DPI). glfwGetWindowContentScale returns
    // e.g. 1.5 on a 150% display — use it to size fonts and widgets correctly.
    {
        float xscale = 1.0f;
        glfwGetWindowContentScale(impl.window, &xscale, nullptr);
        impl.ui_dpi_scale = (xscale > 0.0f) ? xscale : 1.0f;
    }
    impl.ui_scale = 1.0f;

    // Store the pristine (pre-scale) style so the runtime slider can re-apply cleanly.
    impl.imgui_style_base = ImGui::GetStyle();

    // Load the default font at native DPI resolution for crisp text.
    {
        ImFontConfig cfg;
        cfg.SizePixels = std::max(8.0f, std::round(13.0f * impl.ui_dpi_scale));
        ImGui::GetIO().Fonts->AddFontDefault(&cfg);
    }

    // Scale widget sizes to DPI; FontGlobalScale stays at 1.0 because the font
    // is already loaded at the right pixel size above.
    ImGui::GetStyle().ScaleAllSizes(impl.ui_dpi_scale);

    VkDescriptorPoolSize pool_sizes[] = {
        {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 16}
    };
    VkDescriptorPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool_info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    pool_info.maxSets = 16;
    pool_info.poolSizeCount = 1;
    pool_info.pPoolSizes = pool_sizes;
    if (vkCreateDescriptorPool(impl.device, &pool_info, nullptr, &impl.imgui_pool) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create ImGui descriptor pool.");
    }

    ImGui_ImplGlfw_InitForVulkan(impl.window, true);

    ImGui_ImplVulkan_InitInfo init_info{};
    init_info.ApiVersion = VK_API_VERSION_1_0;
    init_info.Instance = impl.instance;
    init_info.PhysicalDevice = impl.physical_device;
    init_info.Device = impl.device;
    init_info.QueueFamily = impl.queue_families.graphics_family.value();
    init_info.Queue = impl.graphics_queue;
    init_info.DescriptorPool = impl.imgui_pool;
    init_info.MinImageCount = 2;
    init_info.ImageCount = static_cast<std::uint32_t>(impl.swapchain_images.size());
    init_info.PipelineInfoMain.RenderPass = impl.render_pass;
    init_info.PipelineInfoMain.MSAASamples = impl.msaa_samples;

    // volk uses VK_NO_PROTOTYPES; the vcpkg imgui build links against Vulkan::Vulkan
    // (vulkan-1.dll), so its internal function pointers may be null. Provide
    // volk-backed dispatch before Init to avoid the null-ptr access violation.
    ImGui_ImplVulkan_LoadFunctions(VK_API_VERSION_1_0,
        [](const char* fn_name, void* ud) -> PFN_vkVoidFunction {
            auto& impl_ref = *static_cast<ArchitectureViewerApp::Impl*>(ud);
            if (PFN_vkVoidFunction fn = vkGetDeviceProcAddr(impl_ref.device, fn_name)) {
                return fn;
            }
            return vkGetInstanceProcAddr(impl_ref.instance, fn_name);
        },
        &impl
    );
    ImGui_ImplVulkan_Init(&init_info);
#else
    (void)impl;
#endif
}

void cleanupImGui(ArchitectureViewerApp::Impl& impl) {
#ifdef HEXAARCH_HAS_IMGUI
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    if (impl.device != VK_NULL_HANDLE && impl.imgui_pool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(impl.device, impl.imgui_pool, nullptr);
        impl.imgui_pool = VK_NULL_HANDLE;
    }
#else
    (void)impl;
#endif
}

void renderUiPanel(ArchitectureViewerApp::Impl& impl) {
#ifdef HEXAARCH_HAS_IMGUI
    const float eff_scale = impl.ui_dpi_scale * impl.ui_scale;
    const float panel_w = std::round(220.0f * eff_scale);
    const float W = static_cast<float>(impl.swapchain_extent.width);
    ImGui::SetNextWindowPos(ImVec2(W - panel_w - 10.0f, 10.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(panel_w, 0.0f), ImGuiCond_Always);
    ImGui::Begin("View Options", nullptr,
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

    bool changed = false;
    changed |= ImGui::Checkbox("Reference Axes", &impl.ui_show_axes);
    changed |= ImGui::Checkbox("Reference Grid", &impl.ui_show_grid);
    ImGui::Separator();
    ImGui::Checkbox("Labels", &impl.ui_show_labels);
    ImGui::Separator();
    ImGui::Checkbox("Wireframe", &impl.ui_wireframe);
    if (changed) { applyVisibilityFlags(impl); }

    ImGui::Separator();
    ImGui::SetNextItemWidth(panel_w - std::round(20.0f * eff_scale));
    ImGui::SliderFloat("##scale", &impl.ui_scale_drag, 0.5f, 3.0f, "Scale %.2f");
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        // Commit the drag value and apply the style rescale once on mouse release.
        impl.ui_scale = std::max(0.5f, std::min(3.0f, impl.ui_scale_drag));
        impl.ui_scale_drag = impl.ui_scale;
        const float new_eff = impl.ui_dpi_scale * impl.ui_scale;
        ImGui::GetStyle() = impl.imgui_style_base;
        ImGui::GetStyle().ScaleAllSizes(new_eff);
        ImGui::GetIO().FontGlobalScale = impl.ui_scale;
    }

    ImGui::End();
#else
    (void)impl;
#endif
}

void renderElementListPanel(ArchitectureViewerApp::Impl& impl) {
#ifdef HEXAARCH_HAS_IMGUI
    if (!impl.owned_arch.has_value()) { return; }
    const auto& assembled = impl.owned_arch->assemblyState().elements;

    const float eff_scale = impl.ui_dpi_scale * impl.ui_scale;
    const float panel_w = std::round(460.0f * eff_scale);
    const float panel_h = std::round(340.0f * eff_scale);
    ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(panel_w, panel_h), ImGuiCond_Always);
    ImGui::Begin("Elements", nullptr,
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

    const float table_h = panel_h - std::round(60.0f * eff_scale);
    const float col_vis_w  = std::round(26.0f * eff_scale);
    const float col_mass_w = std::round(58.0f * eff_scale);
    bool vis_changed = false;
    if (ImGui::BeginTable("##elems", 5,
            ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg,
            ImVec2(0.0f, table_h))) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("##vis",  ImGuiTableColumnFlags_WidthFixed,   col_vis_w);
        ImGui::TableSetupColumn("ID",     ImGuiTableColumnFlags_WidthStretch, 1.2f);
        ImGui::TableSetupColumn("Type",   ImGuiTableColumnFlags_WidthStretch, 0.8f);
        ImGui::TableSetupColumn("Mass",   ImGuiTableColumnFlags_WidthFixed,   col_mass_w);
        ImGui::TableSetupColumn("Pos(m)", ImGuiTableColumnFlags_WidthStretch, 1.5f);
        ImGui::TableHeadersRow();

        for (const auto& ae : assembled) {
            if (ae.element == nullptr) { continue; }
            const std::string& etype = ae.element->type();
            if (etype == "ReferenceAxis" || etype == "ReferenceGrid") { continue; }

            const std::string& eid = ae.element->id();
            const bool has_suffix = etype.size() > 7 &&
                etype.compare(etype.size() - 7, 7, "Element") == 0;
            const std::string short_type = has_suffix ? etype.substr(0, etype.size() - 7) : etype;
            const Eigen::Vector3d pos = ae.world_pose.translation();

            ImGui::TableNextRow();

            ImGui::TableSetColumnIndex(0);
            const auto vis_it = impl.element_visibility.find(eid);
            bool vis = (vis_it == impl.element_visibility.end()) || vis_it->second;
            ImGui::PushID(eid.c_str());
            if (ImGui::Checkbox("##v", &vis)) {
                impl.element_visibility[eid] = vis;
                vis_changed = true;
            }
            ImGui::PopID();

            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted(eid.c_str());
            ImGui::TableSetColumnIndex(2);
            ImGui::TextUnformatted(short_type.c_str());
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%.2f", ae.element->mass());
            ImGui::TableSetColumnIndex(4);
            ImGui::Text("%.1f,%.1f,%.1f", pos.x(), pos.y(), pos.z());
        }

        ImGui::EndTable();
    }
    if (vis_changed) { applyVisibilityFlags(impl); }

    ImGui::Text("%zu elements", assembled.size());
    ImGui::End();
#else
    (void)impl;
#endif
}

void renderLabels(ArchitectureViewerApp::Impl& impl) {
#ifdef HEXAARCH_HAS_IMGUI
    if (!impl.ui_show_labels) { return; }

    const Eigen::Matrix4f vp = impl.camera.projectionMatrix() * impl.camera.viewMatrix();
    const float W = static_cast<float>(impl.swapchain_extent.width);
    const float H = static_cast<float>(impl.swapchain_extent.height);

    std::map<std::string, Eigen::Vector3d> label_positions;
    for (const auto& inst : impl.instances) {
        if (inst.source_element_type == "ReferenceAxis" || inst.source_element_type == "ReferenceGrid") {
            continue;
        }
        if (!inst.source_element_id.empty() && label_positions.count(inst.source_element_id) == 0U) {
            label_positions[inst.source_element_id] = inst.world_transform.translation();
        }
    }

    ImDrawList* draw_list = ImGui::GetForegroundDrawList();
    for (const auto& [id, world_pos] : label_positions) {
        const Eigen::Vector4f clip = vp * Eigen::Vector4f(
            static_cast<float>(world_pos.x()),
            static_cast<float>(world_pos.y()),
            static_cast<float>(world_pos.z()),
            1.0f);
        if (clip.w() <= 0.0f) { continue; }
        const float ndc_z = clip.z() / clip.w();
        if (ndc_z < 0.0f || ndc_z > 1.0f) { continue; }
        const float sx = (clip.x() / clip.w() + 1.0f) * 0.5f * W;
        const float sy = (clip.y() / clip.w() + 1.0f) * 0.5f * H;
        draw_list->AddText(ImVec2(sx + 2.0f, sy - 8.0f), IM_COL32(20, 20, 20, 210), id.c_str());
    }
#else
    (void)impl;
#endif
}

void processPendingUpdates(ArchitectureViewerApp::Impl& impl) {
    std::optional<core::HexacopterArchitecture> new_arch;
    std::string new_title;
    {
        std::lock_guard<std::mutex> lock(impl.pending_mutex);
        new_arch = std::move(impl.pending_arch);
        impl.pending_arch.reset();
        new_title = std::move(impl.pending_title);
        impl.pending_title.clear();
    }
    if (new_arch.has_value()) {
        impl.owned_arch = std::move(new_arch);
        // Rebuild visibility map, preserving any existing per-element states.
        std::unordered_map<std::string, bool> new_vis;
        for (const auto& ae : impl.owned_arch->assemblyState().elements) {
            if (ae.element == nullptr) { continue; }
            const std::string& eid = ae.element->id();
            const auto it = impl.element_visibility.find(eid);
            new_vis[eid] = (it != impl.element_visibility.end()) ? it->second : true;
        }
        impl.element_visibility = std::move(new_vis);
        ArchitectureSceneBuilder scene_builder;
        impl.instances = scene_builder.build(*impl.owned_arch);
        if (impl.mannequin_loaded) {
            PrimitiveInstance ph;
            ph.primitive_type = core::GeometryPrimitive::Kind::box;
            ph.world_transform = Eigen::Isometry3d::Identity();
            ph.world_transform.translation() = impl.mannequin_world_pos + Eigen::Vector3d(0.0, 0.0, 0.9);
            ph.dimensions = Eigen::Vector3d(0.35, 0.15, 0.9);
            ph.color = {0.0f, 0.0f, 0.0f, 0.0f};
            ph.visible = false;
            ph.source_element_type = "HumanReference";
            impl.instances.push_back(ph);
        }
        appendSpatialReferences(impl.instances);
        applyVisibilityFlags(impl);
    }
    if (!new_title.empty() && impl.window != nullptr) {
        glfwSetWindowTitle(impl.window, new_title.c_str());
    }
}

void runViewerLoop(ArchitectureViewerApp::Impl& impl) {
    applyVisibilityFlags(impl);
    while (!glfwWindowShouldClose(impl.window)) {
        processPendingUpdates(impl);
        updateInput(impl);
#ifdef HEXAARCH_HAS_IMGUI
        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        renderUiPanel(impl);
        renderElementListPanel(impl);
        renderLabels(impl);
        ImGui::Render();
#endif
        drawFrame(impl);
    }
    vkDeviceWaitIdle(impl.device);
}

}  // namespace

#else

struct ArchitectureViewerApp::Impl {
    explicit Impl(ArchitectureViewerApp::Config config_in)
        : config(std::move(config_in)) {}

    ArchitectureViewerApp::Config config;
    ViewerCamera camera;
    std::vector<PrimitiveInstance> instances;
};

#endif

ArchitectureViewerApp::ArchitectureViewerApp()
    : ArchitectureViewerApp(Config{}) {}

ArchitectureViewerApp::ArchitectureViewerApp(Config config)
    : config_(std::move(config)),
      impl_(std::make_unique<Impl>(config_)) {
    impl_->camera.setViewport(static_cast<float>(config_.width), static_cast<float>(config_.height));
    impl_->camera.setPerspective(60.0f * 3.14159265358979323846f / 180.0f, 0.1f, 1000.0f);
}

ArchitectureViewerApp::~ArchitectureViewerApp() = default;

void ArchitectureViewerApp::setArchitecture(const core::HexacopterArchitecture& architecture) {
    architecture_ = &architecture;
}

void ArchitectureViewerApp::postArchitecture(core::HexacopterArchitecture architecture, std::string title) {
    std::lock_guard<std::mutex> lock(impl_->pending_mutex);
    impl_->pending_arch = std::move(architecture);
    if (!title.empty()) {
        impl_->pending_title = std::move(title);
    }
}

void ArchitectureViewerApp::requestClose() {
#ifdef HEXAARCH_VISUALIZATION_HAS_VULKAN
    if (impl_ && impl_->window) {
        glfwSetWindowShouldClose(impl_->window, GLFW_TRUE);
    }
#endif
}

int ArchitectureViewerApp::run() {
    if (architecture_ == nullptr) {
        throw std::logic_error("ArchitectureViewerApp requires a bound architecture before run().");
    }

    ArchitectureSceneBuilder scene_builder;
    impl_->instances = scene_builder.build(*architecture_);
    appendSpatialReferences(impl_->instances);
    printSceneSummary(config_, *architecture_, impl_->instances, "Phase V3 Vulkan viewer initialized.");

#ifdef HEXAARCH_VISUALIZATION_HAS_VULKAN
    try {
        initViewerRuntime(*impl_);
        runViewerLoop(*impl_);
        cleanupViewer(*impl_);
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Visualization error: " << ex.what() << '\n';
        cleanupViewer(*impl_);
        return 1;
    }
#else
    std::cout << "Visualization backend is unavailable in this build: GLFW/Vulkan/Volk headers were not found.\n";
    return 0;
#endif
}

const ViewerCamera& ArchitectureViewerApp::camera() const {
    return impl_->camera;
}

}  // namespace hexaarch::visualization

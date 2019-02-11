/*
Passing variables / arrays between cython and cpp
Example from
http://docs.cython.org/src/userguide/wrapping_CPlusPlus.html

Adapted to include passing of multidimensional arrays

*/

#include <vector>
#include <zi/mesh/marching_cubes.hpp>

struct MeshObject {
  std::vector<float> points;
  std::vector<float> normals;
  std::vector<unsigned int> faces;
};

// struct DracoEncodedMeshObject {
//   // std::vector<char> buffer;
//   // char *buffer;
//   std::vector<char> *buffer;
// };

class CMesher {
 private:
  zi::mesh::marching_cubes<uint64_t> marchingcubes_;
  zi::mesh::simplifier<double> simplifier_;
  std::vector<uint32_t> voxelresolution_;

 public:
  CMesher(const std::vector<uint32_t> &voxelresolution);
  ~CMesher();
  void mesh(const std::vector<uint64_t> &data, unsigned int sx, unsigned int sy,
            unsigned int sz);
  std::vector<uint64_t> ids();
  MeshObject get_mesh(uint64_t id, bool generate_normals, int simplification_factor, int max_error);
  std::vector<char> *get_draco_encoded_mesh(uint64_t id, bool generate_normals, int simplification_factor, int max_error);
  // get_draco_mesh_buffer
};

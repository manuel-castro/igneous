/*
Passing variables / arrays between cython and cpp
Example from
http://docs.cython.org/src/userguide/wrapping_CPlusPlus.html

Adapted to include passing of multidimensional arrays

*/

#include <draco/mesh/triangle_soup_mesh_builder.h>
#include <vector>
#include <zi/mesh/int_mesh.hpp>
#include <zi/mesh/quadratic_simplifier.hpp>
#include <zi/vl/vec.hpp>

#include "cMesher.h"

//////////////////////////////////
CMesher::CMesher(const std::vector<uint32_t> &voxelresolution) {
  voxelresolution_ = voxelresolution;
}

CMesher::~CMesher() {}

void CMesher::mesh(const std::vector<uint64_t> &data, unsigned int sx,
                  unsigned int sy, unsigned int sz) {
  // Create Marching Cubes class for type T volume

  const uint64_t *a = &data[0];
  // Run global marching cubes, a mesh is generated for each segment ID group
  marchingcubes_.marche(a, sx, sy, sz);
}

std::vector<uint64_t> CMesher::ids() {
  std::vector<uint64_t> keys;
  for (auto it = marchingcubes_.meshes().begin();
       it != marchingcubes_.meshes().end(); ++it) {
    keys.push_back(it->first);
  }

  return keys;
}

MeshObject CMesher::get_mesh(uint64_t id, bool generate_normals,
                            int simplification_factor,
                            int max_simplification_error) {
  MeshObject obj;

  if (marchingcubes_.count(id) == 0) {  // MC produces no triangles if either
                                        // none or all voxels were labeled!
    return obj;
  }

  zi::mesh::int_mesh im;
  im.add(marchingcubes_.get_triangles(id));
  im.fill_simplifier<double>(simplifier_, 0, 0, 0, voxelresolution_[2],
      voxelresolution_[1], voxelresolution_[0]);
  simplifier_.prepare(generate_normals);

  if (simplification_factor > 0) {
    // This is the most cpu intensive line
    simplifier_.optimize(
        simplifier_.face_count() / simplification_factor,
        max_simplification_error);
  }

  std::vector<zi::vl::vec3d> points;
  std::vector<zi::vl::vec3d> normals;
  std::vector<zi::vl::vec<unsigned, 3> > faces;

  simplifier_.get_faces(points, normals, faces);
  obj.points.reserve(3 * points.size());
  obj.faces.reserve(3 * faces.size());

  if (generate_normals) {
    obj.normals.reserve(3 * points.size());
  }

  for (auto v = points.begin(); v != points.end(); ++v) {
    obj.points.push_back((*v)[2]);
    obj.points.push_back((*v)[1]);
    obj.points.push_back((*v)[0]);
  }

  if (generate_normals) {
    for (auto vn = normals.begin(); vn != normals.end(); ++vn) {
      obj.normals.push_back((*vn)[2]);
      obj.normals.push_back((*vn)[1]);
      obj.normals.push_back((*vn)[0]);
    }
  }

  for (auto f = faces.begin(); f != faces.end(); ++f) {
    obj.faces.push_back((*f)[0]);
    obj.faces.push_back((*f)[2]);
    obj.faces.push_back((*f)[1]);
  }

  return obj;
}

// int main()
// {
//     TriangleSoupMeshBuilder mb;
//     mb.Start(12);
//     const int pos_att_id =
//         mb.AddAttribute(GeometryAttribute::POSITION, 3, DT_FLOAT32);
//     // clang-format off
//   // Front face.
// //   std::vector<float> points{0.f, 0.f, 0.f, }
//     std::vector<float> points(24);
//     for (int i = 0; i < 2; ++i) {
//         for (int j = 0; j < 2; ++j) {
//             for (int k = 0; k < 2; ++k) {
//                 points.push_back(i);
//                 points.push_back(j);
//                 points.push_back(k);
//             }
//         }
//     }

//     std::vector<unsigned int> faces{0, 2, 4, 2, 4, 6, 2, 3, 6, 3, 6, 7, 0, 1, 2, 1, 2, 3, 4, 5, 6, 5, 6, 7, 1, 3, 7, 1, 5, 7, 0, 1, 5, 1, 4, 5};

//     for (int i = 0; i < faces.size(); ++i) {
//         mb.SetAttributeValuesForFace(pos_att_id, FaceIndex(0), )
//         faces[i]
//     }
//     mb.SetAttributeValuesForFace(pos_att_id, FaceIndex(0),
//                                Vector3f(0.f, 0.f, 0.f).data(),
//                                Vector3f(1.f, 0.f, 0.f).data(),
//                                Vector3f(0.f, 1.f, 0.f).data());
//     std::unique_ptr<Mesh> mesh = mb.Finalize();
  
// }

// int main() {
//   std::unique_ptr<Mesh> mesh(new Mesh());
//   out_mesh_ = mesh.get();
//   out_mesh_->SetNumFaces(num_faces);
//   Mesh::Face face;
//   FaceIndex face_index(0);
//   // SetFace for each triplet of vertices that is a face
//   for (int i = 0; i < num_faces; ++i) {
//     const int64_t list_offset = vertex_indices->GetListEntryOffset(i);
//     const int64_t list_size = vertex_indices->GetListEntryNumValues(i);
//     // TODO(ostava): Assume triangular faces only for now.
//     if (list_size != 3)
//         continue;  // All non-triangular faces are skipped.
//     for (int64_t c = 0; c < 3; ++c)
//         face[c] =
//         void mesh(vector[uint64_t], unsigned int, unsigned int,
//             vertex_index_reader.ReadValue(static_cast<int>(list_offset + c));
//     out_mesh_->SetFace(face_index, face);
//     face_index++;
//   }
//   out_mesh_->SetNumFaces(face_index.value());
//   out_point_cloud_ = (cast to point cloud)out_mesh_
//   out_point_cloud_->set_num_points(num_vertices);
//   const DataType dt = x_prop->data_type();
//   GeometryAttribute va;
//   va.Init(GeometryAttribute::POSITION, nullptr, 3, dt, false,
//           DataTypeLength(dt) * 3, 0);
//   const int att_id = out_point_cloud_->AddAttribute(va, true, num_vertices);
//   attribute = out_point_cloud_->attribute(att_id)

//   std::vector<float> memory(3);
//   for (PointIndex::ValueType i = 0; i < static_cast<uint32_t>(num_vertices);
//       ++i) {
//   for (int prop = 0; prop < 3; ++prop) {
//       memory[prop] = get point i;
//   }
//   attribute->SetAttributeValue(AttributeValueIndex(i), memory.data());
//   }
//   Why do we need AttributeValueIndex??
//   if (!out_point_cloud_->DeduplicateAttributeValues())
//         return Status(Status::ERROR, "Could not deduplicate attribute values");
//   out_point_cloud_->DeduplicatePointIds();
// }

// #include <vector>
// #include <zi/mesh/int_mesh.hpp>
// #include <zi/mesh/quadratic_simplifier.hpp>
// #include <zi/vl/vec.hpp>

// #include "cMesher.h"

// //////////////////////////////////
// CMesher::CMesher(const std::vector<uint32_t> &voxelresolution) {
//   voxelresolution_ = voxelresolution;
// }

// CMesher::~CMesher() {}

// void CMesher::mesh(const std::vector<uint64_t> &data, unsigned int sx,
//                   unsigned int sy, unsigned int sz) {
//   // Create Marching Cubes class for type T volume

//   const uint64_t *a = &data[0];
//   // Run global marching cubes, a mesh is generated for each segment ID group
//   marchingcubes_.marche(a, sx, sy, sz);
// }

// std::vector<uint64_t> CMesher::ids() {
//   std::vector<uint64_t> keys;
//   for (auto it = marchingcubes_.meshes().begin();
//        it != marchingcubes_.meshes().end(); ++it) {
//     keys.push_back(it->first);
//   }

//   return keys;
// }

// MeshObject CMesher::get_mesh(uint64_t id, bool generate_normals,
//                             int simplification_factor,
//                             int max_simplification_error) {
//   MeshObject obj;

//   if (marchingcubes_.count(id) == 0) {  // MC produces no triangles if either
//                                         // none or all voxels were labeled!
//     return obj;
//   }

//   zi::mesh::int_mesh im;
//   im.add(marchingcubes_.get_triangles(id));
//   im.fill_simplifier<double>(simplifier_, 0, 0, 0, voxelresolution_[2],
//       voxelresolution_[1], voxelresolution_[0]);
//   simplifier_.prepare(generate_normals);

//   if (simplification_factor > 0) {
//     // This is the most cpu intensive line
//     simplifier_.optimize(
//         simplifier_.face_count() / simplification_factor,
//         max_simplification_error);
//   }

//   std::vector<zi::vl::vec3d> points;
//   std::vector<zi::vl::vec3d> normals;
//   std::vector<zi::vl::vec<unsigned, 3> > faces;

//   simplifier_.get_faces(points, normals, faces);
//   obj.points.reserve(3 * points.size());
//   obj.faces.reserve(3 * faces.size());

//   if (generate_normals) {
//     obj.normals.reserve(3 * points.size());
//   }

//   for (auto v = points.begin(); v != points.end(); ++v) {
//     obj.points.push_back((*v)[2]);
//     obj.points.push_back((*v)[1]);
//     obj.points.push_back((*v)[0]);
//   }

//   if (generate_normals) {
//     for (auto vn = normals.begin(); vn != normals.end(); ++vn) {
//       obj.normals.push_back((*vn)[2]);
//       obj.normals.push_back((*vn)[1]);
//       obj.normals.push_back((*vn)[0]);
//     }
//   }

//   for (auto f = faces.begin(); f != faces.end(); ++f) {
//     obj.faces.push_back((*f)[0]);
//     obj.faces.push_back((*f)[2]);
//     obj.faces.push_back((*f)[1]);
//   }

//   return obj;
// }

// int main() {
//   std::unique_ptr<Mesh> mesh(new Mesh());
//   out_mesh_ = mesh.get();
//   out_mesh_->SetNumFaces(num_faces);
//   Mesh::Face face;
//   FaceIndex face_index(0);
//   // SetFace for each triplet of vertices that is a face
//   for (int i = 0; i < num_faces; ++i) {
//     const int64_t list_offset = vertex_indices->GetListEntryOffset(i);
//     const int64_t list_size = vertex_indices->GetListEntryNumValues(i);
//     // TODO(ostava): Assume triangular faces only for now.
//     if (list_size != 3)
//         continue;  // All non-triangular faces are skipped.
//     for (int64_t c = 0; c < 3; ++c)
//         face[c] =
//             vertex_index_reader.ReadValue(static_cast<int>(list_offset + c));
//     out_mesh_->SetFace(face_index, face);
//     face_index++;
//   }
//   out_mesh_->SetNumFaces(face_index.value());
//   out_point_cloud_ = (cast to point cloud)out_mesh_
//   out_point_cloud_->set_num_points(num_vertices);
//   const DataType dt = x_prop->data_type();
//   GeometryAttribute va;
//   va.Init(GeometryAttribute::POSITION, nullptr, 3, dt, false,
//           DataTypeLength(dt) * 3, 0);
//   const int att_id = out_point_cloud_->AddAttribute(va, true, num_vertices);
//   attribute = out_point_cloud_->attribute(att_id)

//   std::vector<float> memory(3);
//   for (PointIndex::ValueType i = 0; i < static_cast<uint32_t>(num_vertices);
//       ++i) {
//   for (int prop = 0; prop < 3; ++prop) {
//       memory[prop] = get point i;
//   }
//   attribute->SetAttributeValue(AttributeValueIndex(i), memory.data());
//   }
//   Why do we need AttributeValueIndex??
//   if (!out_point_cloud_->DeduplicateAttributeValues())
//         return Status(Status::ERROR, "Could not deduplicate attribute values");
//   out_point_cloud_->DeduplicatePointIds();
// }







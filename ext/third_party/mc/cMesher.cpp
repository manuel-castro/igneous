/*
Passing variables / arrays between cython and cpp
Example from
http://docs.cython.org/src/userguide/wrapping_CPlusPlus.html

Adapted to include passing of multidimensional arrays

*/

#include <vector>
#include <zi/mesh/int_mesh.hpp>
#include <zi/mesh/quadratic_simplifier.hpp>
#include <zi/vl/vec.hpp>
#include "draco/mesh/triangle_soup_mesh_builder.h"
#include "draco/compression/encode.h"
#include "draco/core/encoder_buffer.h"
#include "draco/core/vector_d.h"
#include <fstream>
#include <assert.h> 


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

// std::vector<char> *CMesher::get_draco_encoded_mesh(uint64_t id, bool generate_normals,
//                             int simplification_factor,
//                             int max_simplification_error) {
//   // DracoEncodedMeshObject obj;

//   if (marchingcubes_.count(id) == 0) {  // MC produces no triangles if either
//                                         // none or all voxels were labeled!
//     return NULL;
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

//   draco::TriangleSoupMeshBuilder mb;
//   mb.Start(faces.size());
//   const int pos_att_id =
//     mb.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DataType::DT_FLOAT32);
  
//   for (int i = 0; i < faces.size(); ++i) {
//     zi::vl::vec<unsigned, 3> cur_face = faces[i];
//     auto point1 = points[cur_face[0]];
//     auto point2 = points[cur_face[1]];
//     auto point3 = points[cur_face[2]];
//     mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(i), static_cast<void *>(&point1), static_cast<void *>(&point2), static_cast<void*>(&point3));
//   }

//   std::unique_ptr<draco::Mesh> mesh = mb.Finalize();

//   // std::unique_ptr<draco::Mesh> mesh(new Mesh());
//   // Mesh *out_mesh_ = mesh.get();
//   // out_mesh_->SetNumFaces(faces.size());
//   // draco::Mesh::Face face;
//   // draco::FaceIndex face_index(0);
//   // for (int i = 0; i < faces.size(); ++i) {
//   //   auto cur_face = faces[i];
//   //   for (int j = 0; j < 3; ++j) {
//   //     face[j] = cur_face[j];
//   //   }

//   // }

//   // draco::Options options;
//   // options.pos_quantization_bits = 14;
//   // options.compression_level = 7;
//   // const int speed = 10 - options.compression_level;
//   const int speed = 10 - 7;
//   draco::Encoder encoder;
//   encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION,14);
//   encoder.SetSpeedOptions(speed, speed);
//   draco::EncoderBuffer buffer;
//   // std:vector<char> data = ;
//   draco::Mesh in_mesh = *(mesh.get());
//   const draco::Status status = encoder->EncodeMeshToBuffer(*(mesh.get()), &buffer);
//   if (!status.ok()) {
//     printf("Failed to encode the mesh.\n");
//     printf("%s\n", status.error_msg());
//     // return -1;
//   }
//   // obj.buffer = buffer.buffer();

//   draco::DecoderBuffer decoderBuffer;
//   decoderBuffer.Init(buffer.data(), buffer.size());
//   draco::Decoder decoder;
//   auto statusor = decoder.DecodeMeshFromBuffer(&decoderBuffer);
//   if (!statusor.ok()) {
//     // return ReturnError(statusor.status());
//     printf("Failed to decode the mesh.\n");
//   } else {
//     std::unique_ptr<draco::Mesh> dec_mesh = std::move(statusor).value();
//     draco::Mesh out_mesh = *(dec_mesh.get());
//     assert(in_mesh.num_points() == out_mesh.num_points());
//     assert(in_mesh.num_faces() == out_mesh.num_faces());
//   }
//   // return obj;
//   return buffer.buffer();
//   // DecodeMeshFromBuffer?
//   // obj.points.reserve(3 * points.size());
//   // obj.faces.reserve(3 * faces.size());

//   // if (generate_normals) {
//   //   obj.normals.reserve(3 * points.size());
//   // }

//   // for (auto v = points.begin(); v != points.end(); ++v) {
//   //   obj.points.push_back((*v)[2]);
//   //   obj.points.push_back((*v)[1]);
//   //   obj.points.push_back((*v)[0]);
//   // }

//   // if (generate_normals) {
//   //   for (auto vn = normals.begin(); vn != normals.end(); ++vn) {
//   //     obj.normals.push_back((*vn)[2]);
//   //     obj.normals.push_back((*vn)[1]);
//   //     obj.normals.push_back((*vn)[0]);
//   //   }
//   // }

//   // for (auto f = faces.begin(); f != faces.end(); ++f) {
//   //   obj.faces.push_back((*f)[0]);
//   //   obj.faces.push_back((*f)[2]);
//   //   obj.faces.push_back((*f)[1]);
//   // }


// }

void CMesher::get_draco_encoded_mesh(uint64_t id, bool generate_normals,
                                                  int simplification_factor,
                                                  int max_simplification_error, float xmin, float ymin, float zmin, uint64_t remapped_id, const char **bytes_ptr, size_t *bytes_len)
{
  MeshObject obj;

  if (marchingcubes_.count(id) == 0)
  { // MC produces no triangles if either
    // none or all voxels were labeled!
    // return obj;
    printf("empty mesh exception\n");
    throw;
  }

  zi::mesh::int_mesh im;
  im.add(marchingcubes_.get_triangles(id));
  im.fill_simplifier<double>(simplifier_, 0, 0, 0, voxelresolution_[2],
                             voxelresolution_[1], voxelresolution_[0]);
  simplifier_.prepare(generate_normals);

  if (simplification_factor > 0)
  {
    // This is the most cpu intensive line
    simplifier_.optimize(
        simplifier_.face_count() / simplification_factor,
        max_simplification_error);
  }
  // printf("going for remapped_id %s\n", std::to_string(remapped_id));
  std::vector<zi::vl::vec3d> points;
  std::vector<zi::vl::vec3d> normals;
  std::vector<zi::vl::vec<unsigned, 3>> faces;

  simplifier_.get_faces(points, normals, faces);

  draco::TriangleSoupMeshBuilder mb;
  mb.Start(faces.size());
  const int pos_att_id =
    mb.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DataType::DT_FLOAT32);

  std::vector<int> resolution{32, 32, 40};
  std::vector<float> bounds{xmin, ymin, zmin};

  // printf("before faces\n");
  for (int i = 0; i < points.size(); ++i) {
    for (int j = 0; j < 3; ++j) {
      points[i][j] /= 2.0;
      points[i][j] += bounds[j] * resolution[j];
    }
  }
  for (std::size_t i = 0; i < faces.size(); ++i) {
    zi::vl::vec<unsigned, 3> cur_face = faces[i];
    zi::vl::vec3d point1 = points[cur_face[0]];
    zi::vl::vec3d point2 = points[cur_face[1]];
    zi::vl::vec3d point3 = points[cur_face[2]];
    // mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(i), static_cast<void *>(&point1), static_cast<void *>(&point2), static_cast<void*>(&point3));
    mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(i), draco::Vector3f(point1[0], point1[1], point1[2]).data(), draco::Vector3f(point2[0], point2[1], point2[2]).data(), draco::Vector3f(point3[0], point3[1], point3[2]).data());  
  }

  // printf("after faces\n");
  std::unique_ptr<draco::Mesh> ptr_mesh = mb.Finalize();
  draco::Mesh *mesh = ptr_mesh.get();
  draco::Encoder encoder;
  encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 14);
  encoder.SetSpeedOptions(3, 3);
  draco::EncoderBuffer buffer;
  const draco::Status status = encoder.EncodeMeshToBuffer(*mesh, &buffer);
  // printf("after encoder setup\n");
  // const std::string &file = "meshTest" + std::to_string(remapped_id) + ".drc";
  // std::ofstream out_file(file, std::ios::binary);
  // out_file.write(buffer.data(), buffer.size());
  // printf("after writing\n");
  // std::vector<char> dummy{'t'};
  // return dummy;
  // return *(buffer.buffer());
  *bytes_ptr = buffer.data();
  *bytes_len = buffer.size();
}

/*std::vector<char> int CMesher::get_draco_encoded_mesh(uint64_t id, bool generate_normals,
                            int simplification_factor,
                            int max_simplification_error) {
draco::TriangleSoupMeshBuilder mb;
        mb.Start(12);
        const int pos_att_id =
            mb.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DataType::DT_FLOAT32);
            mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(0),
                               draco::Vector3f(0.f, 0.f, 0.f).data(),
                               draco::Vector3f(1.f, 0.f, 0.f).data(),
                               draco::Vector3f(0.f, 1.f, 0.f).data());
  mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(1),
                               draco::Vector3f(0.f, 1.f, 0.f).data(),
                               draco::Vector3f(1.f, 0.f, 0.f).data(),
                               draco::Vector3f(1.f, 1.f, 0.f).data());

  // Back face.
  mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(2),
                               draco::Vector3f(0.f, 1.f, 1.f).data(),
                               draco::Vector3f(1.f, 0.f, 1.f).data(),
                               draco::Vector3f(0.f, 0.f, 1.f).data());
  mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(3),
                               draco::Vector3f(1.f, 1.f, 1.f).data(),
                               draco::Vector3f(1.f, 0.f, 1.f).data(),
                               draco::Vector3f(0.f, 1.f, 1.f).data());

  // Top face.
  mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(4),
                               draco::Vector3f(0.f, 1.f, 0.f).data(),
                               draco::Vector3f(1.f, 1.f, 0.f).data(),
                               draco::Vector3f(0.f, 1.f, 1.f).data());
  mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(5),
                               draco::Vector3f(0.f, 1.f, 1.f).data(),
                               draco::Vector3f(1.f, 1.f, 0.f).data(),
                               draco::Vector3f(1.f, 1.f, 1.f).data());

  // Bottom face.
  mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(6),
                               draco::Vector3f(0.f, 0.f, 1.f).data(),
                               draco::Vector3f(1.f, 0.f, 0.f).data(),
                               draco::Vector3f(0.f, 0.f, 0.f).data());
  mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(7),
                               draco::Vector3f(1.f, 0.f, 1.f).data(),
                               draco::Vector3f(1.f, 0.f, 0.f).data(),
                               draco::Vector3f(0.f, 0.f, 1.f).data());

  // Right face.
  mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(8),
                               draco::Vector3f(1.f, 0.f, 0.f).data(),
                               draco::Vector3f(1.f, 0.f, 1.f).data(),
                               draco::Vector3f(1.f, 1.f, 0.f).data());
  mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(9),
                               draco::Vector3f(1.f, 1.f, 0.f).data(),
                               draco::Vector3f(1.f, 0.f, 1.f).data(),
                               draco::Vector3f(1.f, 1.f, 1.f).data());

  // Left face.
  mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(10),
                               draco::Vector3f(0.f, 1.f, 0.f).data(),
                               draco::Vector3f(0.f, 0.f, 1.f).data(),
                               draco::Vector3f(0.f, 0.f, 0.f).data());
  mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(11),
                               draco::Vector3f(0.f, 1.f, 1.f).data(),
                               draco::Vector3f(0.f, 0.f, 1.f).data(),
                               draco::Vector3f(0.f, 1.f, 0.f).data());
        std::unique_ptr<draco::Mesh> maybe_mesh = mb.Finalize();
        draco::Mesh *mesh = maybe_mesh.get();
        draco::Encoder encoder;
        encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 14);
        encoder.SetSpeedOptions(3, 3);
        printf("after encoder setup\n");
        draco::EncoderBuffer buffer;
        const draco::Status status = encoder.EncodeMeshToBuffer(*mesh, &buffer);
        const std::string &file = "cubeTest.drc";
        std::ofstream out_file(file, std::ios::binary);
        out_file.write(buffer.data(), buffer.size());
        return 1;
                            }*/

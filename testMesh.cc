#include "draco/mesh/triangle_soup_mesh_builder.h"
#include "draco/compression/encode.h"
#include "draco/core/vector_d.h"
#include <fstream>

    int main()
    {
        draco::TriangleSoupMeshBuilder mb;
        mb.Start(12);
        const int pos_att_id =
            mb.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DataType::DT_FLOAT32);
        // clang-format off
    // Front face.
    //   std::vector<float> points{0.f, 0.f, 0.f, }
        // std::vector<float> points(24);
        // for (int i = 0; i < 2; ++i) {
        //     for (int j = 0; j < 2; ++j) {
        //         for (int k = 0; k < 2; ++k) {
        //             points.push_back(i);
        //             points.push_back(j);
        //             points.push_back(k);
        //         }
        //     }
        // }
        // printf("after points\n");
        // std::vector<unsigned int> faces{0, 2, 4, 2, 4, 6, 2, 3, 6, 3, 6, 7, 0, 1, 2, 1, 2, 3, 4, 5, 6, 5, 6, 7, 1, 3, 7, 1, 5, 7, 0, 1, 5, 1, 4, 5};
        // printf("before iter\n");
        // for (int i = 0; i < faces.size(); i += 3) {
        //     //mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(0), )
        //     // auto point1 = points[faces[i]];
        //     // auto point2 = points[faces[i+1]];
        //     // auto point3 = points[faces[i+2]];
        //     void *point1 = static_cast<void *>(&(points[faces[i]]));
        //     void *point2 = static_cast<void *>(&(points[faces[i+1]]));
        //     void *point3 = static_cast<void *>(&(points[faces[i+2]]));
        //     printf("point1 %d\n", point1);
        //     // mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(i), static_cast<void *>(&point1), static_cast<void *>(&point2), static_cast<void*>(&point3));
        //     mb.SetAttributeValuesForFace(pos_att_id, draco::FaceIndex(i/3), static_cast<void *>(&point1), static_cast<void *>(&point2), static_cast<void*>(&point3));

        // }
        // printf("after faces\n");
        // mb.SetAttributeValuesForFace(pos_att_id, FaceIndex(0),
        //                            draco::Vector3f(0.f, 0.f, 0.f).data(),
        //                            draco::Vector3f(1.f, 0.f, 0.f).data(),
        //                            draco::Vector3f(0.f, 1.f, 0.f).data());
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
    }

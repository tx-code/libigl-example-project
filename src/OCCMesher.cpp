#include "OCCMesher.h"

// OpenCASCADE headers
#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <IMeshTools_Parameters.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

OCCMesher::OCCMesher() {
  // Constructor implementation
}

bool OCCMesher::generateMesh(const TopoDS_Shape &shape, Eigen::MatrixXd &V,
                             Eigen::MatrixXi &F) {
  spdlog::stopwatch sw;

  // Create mesh parameters
  IMeshTools_Parameters params;
  params.Deflection = autoSelectLinearDeflection(shape);
  params.Angle = 0.5;
  params.Relative = true;
  params.MinSize = params.Deflection * 0.01;
  params.InParallel = true; // enable the parallel mode

  try {
    // Note: will automatically call Perform()
    BRepMesh_IncrementalMesh mesh(shape, params);
  } catch (Standard_Failure &e) {
    spdlog::error("OCC Meshing Error: {}", e.GetMessageString());
    return false;
  }

  // Summary info
  int total_vertices = 0;
  double max_deflection = 0.0;
  int total_triangles = 0;
  std::vector<int> face_vertex_offsets;

  TopExp_Explorer explorer;
  for (explorer.Init(shape, TopAbs_FACE); explorer.More(); explorer.Next()) {
    TopoDS_Face face = TopoDS::Face(explorer.Current());
    TopLoc_Location loc;

    Handle(Poly_Triangulation) triangulation =
        BRep_Tool::Triangulation(face, loc);
    if (!triangulation.IsNull()) {
      face_vertex_offsets.push_back(total_vertices);
      total_vertices += triangulation->NbNodes();
      total_triangles += triangulation->NbTriangles();
      max_deflection = std::max(max_deflection, triangulation->Deflection());
    }
  }

  spdlog::info("OCC mesh info before cleaning: {} vertices, {} triangles, max "
               "deflection: {}",
               total_vertices, total_triangles, max_deflection);

  // Adjust matrix sizes
  Eigen::MatrixXd V_Uncleaned;
  Eigen::MatrixXi F_Uncleaned;
  V_Uncleaned.resize(total_vertices, 3);
  F_Uncleaned.resize(total_triangles, 3);

  // Fill matrices
  int vertex_index = 0;
  int face_index = 0;
  int face_count = 0;

  for (explorer.Init(shape, TopAbs_FACE); explorer.More(); explorer.Next()) {
    TopoDS_Face face = TopoDS::Face(explorer.Current());
    TopLoc_Location loc;

    Handle(Poly_Triangulation) triangulation =
        BRep_Tool::Triangulation(face, loc);
    if (!triangulation.IsNull()) {
      int offset = face_vertex_offsets[face_count++];

      // Get vertices
      const auto &nodes = triangulation->MapNodeArray();
      for (int i = nodes->Lower(); i <= nodes->Upper(); i++) {
        gp_Pnt p = nodes->Value(i).Transformed(loc);
        V_Uncleaned(vertex_index, 0) = p.X();
        V_Uncleaned(vertex_index, 1) = p.Y();
        V_Uncleaned(vertex_index, 2) = p.Z();
        vertex_index++;
      }

      // Get triangles
      const auto &triangles = triangulation->MapTriangleArray();
      for (int i = triangles->Lower(); i <= triangles->Upper(); i++) {
        int n1, n2, n3;
        triangles->Value(i).Get(n1, n2, n3);

        // Adjust based on face orientation
        if (face.Orientation() == TopAbs_REVERSED) {
          std::swap(n2, n3);
        }

        F_Uncleaned(face_index, 0) = offset + n1 - 1;
        F_Uncleaned(face_index, 1) = offset + n2 - 1;
        F_Uncleaned(face_index, 2) = offset + n3 - 1;
        face_index++;
      }
    }
  }

  // Clean the mesh (remove duplicate vertices)
  cleanMesh(V_Uncleaned, F_Uncleaned, V, F);

  spdlog::info("OCC mesh info after cleaning: {} vertices, {} triangles",
               V.rows(), F.rows());
  spdlog::info("OCC meshing completed in {:.3f}s", sw);

  return true;
}
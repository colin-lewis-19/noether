/*
 * trim_edges.cpp
 *
 *  Created on: May 3, 2021
 *      Author: clewis
 */

#include <XmlRpcException.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <console_bridge/console.h>
#include "noether_filtering/utils.h"
#include "noether_filtering/mesh/trim_edges.h"
#include <pcl/point_types.h>
#include <pcl/geometry/triangle_mesh.h>
#include <pcl/geometry/mesh_conversion.h>
#include <pcl/geometry/mesh_indices.h>
#include <pcl/geometry/get_boundary.h>
#include <boost/optional.hpp>
#include <boost/bimap.hpp>
namespace noether_filtering
{
namespace mesh
{

TrimEdges::TrimEdges() {}

TrimEdges::~TrimEdges() {}

bool TrimEdges::configure(XmlRpc::XmlRpcValue config)
{
  try
  {
    config_.num_passes = static_cast<int>(config["num_passes"]);
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    CONSOLE_BRIDGE_logError(e.getMessage().c_str());
    return false;
  }
  return true;
}

bool TrimEdges::filter(const pcl::PolygonMesh& mesh_in, pcl::PolygonMesh& mesh_out)
{
  using MeshTraits = pcl::geometry::DefaultMeshTraits<pcl::PointXYZ, int, int, pcl::PointNormal>;
  typedef pcl::geometry::TriangleMesh<MeshTraits> TriangleMesh;

  TriangleMesh mesh;
  pcl::geometry::toHalfEdgeMesh(mesh_in, mesh);
  for(int i=0; i<config_.num_passes; i++)
    {
      std::vector<TriangleMesh::HalfEdgeIndices> boundary_edges;
      getBoundBoundaryHalfEdges (mesh, boundary_edges, 30);
      for(TriangleMesh::HalfEdgeIndices heid: boundary_edges)
	{
	  for(TriangleMesh::HalfEdgeIndex id : heid)
	    mesh.deleteEdge(id);
	}
      mesh.cleanUp();
    }
  toFaceVertexMesh(mesh, mesh_out);
  return true; // never fails, returning true to be consistent with other Noether filters.
}

  
} /* namespace mesh */
} /* namespace noether_filtering */

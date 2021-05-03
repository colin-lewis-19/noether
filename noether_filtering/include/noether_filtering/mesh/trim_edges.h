/*
 * trim_edges.h
 *
 *  Created on: May 3, 2021
 *      Author: clewis
 */

#ifndef SRC_MESH_TRIM_EDGES_H_
#define SRC_MESH_TRIM_EDGES_H_

#include "noether_filtering/mesh/mesh_filter_base.h"
#include <pcl/PolygonMesh.h>
#include <pcl/geometry/polygon_mesh.h>

namespace noether_filtering
{
namespace mesh
{
/**
 * @class noether_filtering:;mesh::TrimEdges
 * @details traverses the half edges to find boundaries and removes all triangles associeted with these edges
 *  operation repeats num_passes times. 
 */
class TrimEdges : public MeshFilterBase
{
public:
  struct Config
  {
    int num_passes = 1; // number of passes around boundary, 
  };

  TrimEdges();
  virtual ~TrimEdges();

  /**
   * @details Loads a configuration from yaml of the following form:
   *
   * num_passes: 1
   *
   * @param config
   * @return
   */
  bool configure(XmlRpc::XmlRpcValue config) override final;

  /**
   * @brief applies the filtering method
   * @param mesh_in   Input mesh
   * @param mesh_out  Output mesh
   * @return True on success, false otherwise
   */
  bool filter(const pcl::PolygonMesh& mesh_in, pcl::PolygonMesh& mesh_out) override final;

  /**
   * @brief returns the type name of the filter
   * @return  A string containing the filter type name
   */
  std::string getName() const override final;

private:

  Config config_;
};

} /* namespace mesh */
} /* namespace noether_filtering */

#endif /* SRC_MESH_TRIM_EDGES_H_ */

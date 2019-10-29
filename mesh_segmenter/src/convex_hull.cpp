#include <mesh_segmenter/convex_hull.h>


ConvexHullGenerator::ConvexHullGenerator(){}  

void ConvexHullGenerator::makeMesh(const std::string& input, pcl::PointCloud<pcl::PointXYZ>& inMesh)
{
  //expects a .ply file
  pcl::PLYReader Reader;
  Reader.read(input, inMesh); //populate inMesh
  return;
}

void ConvexHullGenerator::cleanMesh(const pcl::PointCloud<pcl::PointXYZ>& outMesh, pcl::PolygonMesh& outMeshPoly)
{
  //find centroid coords by finding average x, y, z 

  Eigen::Matrix< float, 4, 1 > mid;
  int centroid_success = pcl::compute3DCentroid(outMesh, mid);
  Eigen::Vector3d midVec= {mid[0], mid[1], mid[2]};

  //invert bad polygons --------------------------------
  for (int t=0; t < (outMeshPoly.polygons.size()); t++)
  {
    pcl::Vertices verts;
    verts = outMeshPoly.polygons[t];

    pcl::PointXYZ a = outMesh.points[verts.vertices[0]];
    pcl::PointXYZ b = outMesh.points[verts.vertices[1]];
    pcl::PointXYZ c = outMesh.points[verts.vertices[2]];

    Eigen::Vector3d p0 = {a.x, a.y, a.z};
    Eigen::Vector3d p1 = {b.x, b.y, b.z};
    Eigen::Vector3d p2 = {c.x, c.y, c.z};

    Eigen::Vector3d v1 = p1 - p0; 
    Eigen::Vector3d v2 = p2 - p0;
    Eigen::Vector3d d = p0 - midVec;
    Eigen::Vector3d normal = v1.cross(v2);
    float works = d.dot(normal);

    if (works < 0)
    {
      int temp;
      temp = verts.vertices[1];
      verts.vertices[1] = verts.vertices[2];
      verts.vertices[2] = temp;

    }
    outMeshPoly.polygons[t] = verts;
  } 
  return;
}


bool ConvexHullGenerator::saveMesh(const pcl::PointCloud<pcl::PointXYZ>& outMesh, pcl::PolygonMesh& outMeshPoly, const std::string& outfile)
{
  pcl::toPCLPointCloud2(outMesh, outMeshPoly.cloud);
  pcl::io::savePolygonFile("/tmp/mesh.ply", outMeshPoly, false);
  return true;
}

bool ConvexHullGenerator::generateCH(const std::string& infile, const std::string& outfile)
{

    pcl::PointCloud<pcl::PointXYZ> inMesh;
    pcl::PointCloud<pcl::PointXYZ> outMesh;
    pcl::PolygonMesh outMeshPoly;
    pcl::ConvexHull<pcl::PointXYZ> chull;

    ConvexHullGenerator::makeMesh(infile, inMesh);

    chull.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ> (inMesh))); //generate hull
    chull.reconstruct(outMesh, outMeshPoly.polygons); //save to outMesh

    ConvexHullGenerator::cleanMesh(outMesh, outMeshPoly);
    std::cout<<"this is where we fail" << std::endl;
    bool success = ConvexHullGenerator::saveMesh(outMesh, outMeshPoly, outfile);
    return success;
}

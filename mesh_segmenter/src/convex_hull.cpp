#include <mesh_segmenter/convex_hull.h>

using namespace std; //shouldn't do this
ConvexHullGenerator::ConvexHullGenerator(){}  

pcl::PointCloud<pcl::PointXYZ>::Ptr ConvexHullGenerator::makemesh(string input, pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud)
{
//making mesh----------------------------------------------
  //expects a .ply file
  pcl::PLYReader Reader;
  Reader.read(input, *inCloud); //populate inCloud
  return inCloud;
}


void ConvexHullGenerator::cleanmesh(pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, pcl::PolygonMesh::Ptr outMeshPoly)
{

  //find centroid coords by finding average x, y, z -----------------------

  Eigen::Matrix< float, 4, 1 > mid;
  int centroid_success = pcl::compute3DCentroid(*outMesh, mid); //this needs to be checked
  Eigen::Vector3d midVec= {mid[0], mid[1], mid[2]};

  //invert bad polygons --------------------------------
  for (int t=0; t < (outMeshPoly->polygons.size()); t++)
  {
    pcl::Vertices verts;
    verts = outMeshPoly->polygons[t];

    pcl::PointXYZ a = outCloud->points[verts.vertices[0]];
    pcl::PointXYZ b = outCloud->points[verts.vertices[1]];
    pcl::PointXYZ c = outCloud->points[verts.vertices[2]];


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
    outMeshPoly->polygons[t] = verts;
  } 
  return;
}

bool ConvexHullGenerator::savemesh(pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, pcl::PolygonMesh::Ptr outMeshPoly, string outfile)
{
  pcl::toPCLPointCloud2(*outCloud, outMeshPoly->cloud);

  pcl::io::savePolygonFile(outfile, *outMeshPoly, false); //core dumping in here
  return true;
}

bool ConvexHullGenerator:: generate_ch(string infile,string outfile)
{
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PolygonMesh::Ptr outMeshPoly (new pcl::PolygonMesh);
    //pcl::PolygonMesh testMeshPoly;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;

    outfile = "/home/ctlewis/plzwork.stl";

    //inCloud = ConvexHullGenerator::makemesh(infile, inCloud);


    pcl::PLYReader Reader;
    Reader.read(infile, *inCloud); //populate inCloud

    chull.setInputCloud(inCloud); //generate hull (gdb is showing the chull as empty here, but populated at savePoly)
    chull.reconstruct(*outCloud, outMeshPoly->polygons); //save to outCloud

    /* Method calls ----------------------------------------------------------------
    ConvexHullGenerator::cleanmesh(outCloud, outMeshPoly);
    bool success = ConvexHullGenerator::savemesh(outCloud, outMeshPoly, outfile);
    --------------------------------------------------------------------------------*/
    pcl::toPCLPointCloud2(*outCloud, outMeshPoly->cloud); //packs outMeshPoly with everything we need
    
    //DEBUG BLOCK--------------------------------------------------------------------------------
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> data = outCloud->points;
    cout << data.size() <<endl; 
    std::vector< pcl::uint8_t> d2 = outMeshPoly->cloud.data;
    cout << d2.size() << endl; //I guess this doesn't actually yield any usable information
    //-------------------------------------------------------------------------------------------

    cout << "made it to save file" << endl;

    //core dumping here; Likely its *outMeshPoly (works with an empty mesh). Fails for other outfile formats. id of point is 0, but coords seem correct
    pcl::io::savePolygonFilePLY(outfile, *outMeshPoly, false); 
    
    cout << "no core dump"<<endl; //we can dream
  return true;
}

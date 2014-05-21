#ifndef MODELS
#define MODELS

#include <vector>
#include <tr1/tuple>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>
// Binary serialization is faster than text serialization
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

//xml parser
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
//database
#include <postgresql/libpq-fe.h>
#include <vector>
#include <stdlib.h>
#include <vtkOBJReader.h>
//MESH NORMALS
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkPolyDataNormals.h>
#include <vtkTriangle.h>
#include <vtkDoubleArray.h>
#include <vtkQuadricClustering.h>
#include <vtkQuadricDecimation.h>
#include <vtkFillHolesFilter.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkTriangleFilter.h>
#include <vtkMath.h>
#include <vtkFeatureEdges.h>
#include <vtkCleanPolyData.h>
#include <vtkStripper.h>
#include <vtkPolyDataConnectivityFilter.h>

#include <vtkCleanPolyData.h>
#include <vtkDelaunay2D.h>
#include <vtkDelaunay3D.h>
#include <vtkTriangleFilter.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkButterflySubdivisionFilter.h>
#include <vtkLoopSubdivisionFilter.h>
#include <vtkLinearSubdivisionFilter.h>
#include <vtkMaskPoints.h>
#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkUnstructuredGrid.h>
#include <vtkGeometryFilter.h>
#include <vtkAppendPolyData.h>
#include <vtkCellArray.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkPolygon.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>


#include "object_model.h"


/*boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr cloud)
{
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (255, 255, 255);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGBNormal> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "sample cloud");
    //viewer->addPointCloudNormals<pcl::PointXYZRGBNormal, pcl::Normal> (cloud, normals, 1, 0.01, "normals");
    viewer->addPointCloudNormals<pcl::PointXYZRGBNormal>(cloud,1,0.01,"normals");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, static_cast<float>(205.0/255.0), static_cast<float>(201.0/255.0), static_cast<float>(201.0/255.0), "normals");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "normals");
    viewer->addCoordinateSystem (0.1);
    viewer->initCameraParameters ();
    //viewer->addText("Z", 10, 10, "v1 text", 0);
    pcl::PointXYZ textXPos(0.105,-0.01,0.0);
    pcl::PointXYZ textYPos(0.0,0.105,0.0);
    pcl::PointXYZ textZPos(0.0,0.0,0.105);
    float textScale=0.01;
    viewer->addText3D ("x", textXPos, textScale, 0.5, 0.5, 0.5, "x");
    viewer->addText3D ("y", textYPos, textScale, 0.5, 0.5, 0.5, "y");
    viewer->addText3D ("z", textZPos, textScale, 0.5, 0.5, 0.5, "z");
    //viewer->setCameraPosition ( 0.5,0.1,0.2, 0, 0, 1);

    return (viewer);
}*/

typedef struct 
{
    double x,y,z;
}Point3d;

template <class objectModelT>
class models
{
public:
    typedef pcl::PointCloud<pcl::PointXYZ> pointCloudPointXYZ;
    typedef pointCloudPointXYZ::Ptr pointCloudPointXYZPtr;
    typedef pcl::PointCloud<pcl::Normal> pointCloudNormal;
    typedef pointCloudNormal::Ptr pointCloudNormalPtr;
    typedef pcl::PointCloud<pcl::PointNormal> pointCloudPointNormal;
    typedef pointCloudPointNormal::Ptr pointCloudPointNormalPtr;

    // Variables
    std::vector< boost::shared_ptr<objectModelT> > objectModels;

    // Global parameters (for normals computation)
    static bool radiusSearch;
    static float radius;
    static float neighbours;

    static int angleBins;
    static int distanceBins;

    static float angleStep;
    static float angleStepInverted;
    static float angleStepInvertedCos;

    // Methods
    models()
    {}

    //template <class objectModelT>
    bool newObjectModelDescriptor(unsigned int dbId, std::tr1::tuple<pointCloudPointNormalPtr, float, float, shape_msgs::MeshPtr> & modelData, bool _symmetric)
    {
        typename std::vector<boost::shared_ptr<objectModelT> >::iterator it;

        // Check if this model is new
        for(it=objectModels.begin(); it< objectModels.end(); it++)
        {
            if(dbId==(*it)->dbId)
            {
                std::cout << "Model " << dbId << " previously trained." << endl;
                return false;
            }
        }

        std::cout << "Train model: " << dbId << "..." << endl;
        // Add new model to the objectModels vector
        objectModels.push_back(boost::shared_ptr<objectModelT> ( new objectModelT(dbId, modelData, _symmetric) ));
        std::cout << "Done"<< std::endl;

        return true;
    }

    bool newObjectModelDescriptor(unsigned int dbId, std::string filename, std::string frameId, bool _symmetric, double scale)
    {
        typename std::vector<boost::shared_ptr<objectModelT> >::iterator it;

        // Check if this model is new
        for(it=objectModels.begin(); it< objectModels.end(); it++)
        {
            if(dbId==(*it)->dbId)
            {
                std::cout << "Model " << dbId << " previously trained." << endl;
                return false;
            }
        }

        std::cout << "Train model: " << dbId << "..." << endl;
        // Add new model to the objectModels vector
        std::tr1::tuple<pointCloudPointXYZPtr,pointCloudPointNormalPtr,float> clouds = loadModelPLY(filename, frameId, scale);
        objectModels.push_back(boost::shared_ptr<objectModelT> ( new objectModelT(dbId, clouds, _symmetric) ));
        std::cout << "Done"<< std::endl;

        return true;
    }


    // Load models from household objects database and train
    bool loadModels(bool trainMode, float scale, ros::NodeHandle n_, std::string dbname, std::string host, std::string port, std::string user, std::string password, std::string inputFile, std::string modelsDescriptorsFile, std::string frameId)
    {
        using boost::property_tree::ptree;
        ptree pt;

        // Load data, if available
        //std::cout << "Loading file (THIS IS DISABLED (ENABLE WHEN TESTS AND THESIS ARE OVER!) " << modelsDescriptorsFile.c_str() << std::endl;;
        //loadData(modelsDescriptorsFile.c_str());
        std::cout << "Done" << std::endl;

        typename std::vector<boost::shared_ptr<objectModelT> >::iterator it;
        if(trainMode)
        {

            // Check if there are new models to train
            std::cout << "Models already trained: " << objectModels.size() << std::endl;
            for(it=objectModels.begin(); it < objectModels.end(); ++it)
            {
                std::cout << "\tid: " << (*it)->id << std::endl;
                std::cout << "\thoushold database id: " << (*it)->dbId << std::endl;
            }

            bool newModels=false;

            std::cout << "New model sets acquisition and training:" << std::endl;

            ///////////////////////////////
            // Get models info from .xml //
            ///////////////////////////////

            read_xml(inputFile, pt);
            BOOST_FOREACH( ptree::value_type const& v, pt.get_child("object_library") )
            {
                bool newModel=true;
                if( v.first == "object" )
                {
                    for(it=objectModels.begin(); it< objectModels.end(); ++it)
                    {
                        if((*it)->dbId==v.second.get<unsigned int>("id"))
                            newModel=false;
                    }

                    // Model already trained, continue
                    if(!newModel)
                        continue;
                    // Else, new model
                    newModels=true;

                    //						// Parse graspable point indices
                    //						ptree pt2=v.second;
                    //						pcl::PointIndices _graspablePoints;
                    //						_graspablePoints.header.frame_id=frameId;
                    //						BOOST_FOREACH( ptree::value_type const& g, pt2)
                    //						{
                    //							if( g.first == "graspable_point" )
                    //							{
                    //								_graspablePoints.indices.push_back(g.second.get<int>("<xmlattr>.index"));
                    //							}
                    //						}

                    static std::tr1::tuple<pointCloudPointNormalPtr, float, float, shape_msgs::MeshPtr>  modelData = loadModelData(v.second.get<std::string>("id"), scale, v.second.get<bool>("symmetric"), dbname, host, port, user, password, frameId);
                    // Add new model to the model descriptor library
                    newObjectModelDescriptor(v.second.get<unsigned int>("id"), modelData, v.second.get<bool>("symmetric"));

                    std::cout << "Done" << std::endl;
                }

            }

            // Write new models descriptors to the library file
            if(newModels)
            {
                std::cout << "Writing data to output file..." << std::endl;
                //saveData(modelsDescriptorsFile.c_str());
                std::cout << "Done" << std::endl;
            }
            else
                std::cout << "No new models" << std::endl;
        }
        else
        {
            std::cout << "Models loaded: " << objectModels.size() << std::endl;
            for(it=objectModels.begin(); it < objectModels.end(); ++it)
            {
                std::cout << "\tid: " << (*it)->id << std::endl;
                std::cout << "\tdb id: " << (*it)->dbId << std::endl;
            }
            std::cout << "Done" << std::endl;
        }

        return true;
    }

    bool saveData(const char* filename)
    {
        // create and open a character archive for output
        std::ofstream ofs(filename);
        // save data to archive
        if(ofs.good())
        {
            boost::archive::binary_oarchive oa(ofs);
            // write class instance to archive
            oa << *this;
            return true;
        }
        else
        {
            std::cout << "Error while saving data" << endl;
            return false;
        }
    }

    bool loadData(const char* filename)
    {
        // open a character archive for input
        std::ifstream ifs(filename);
        if(ifs.good())
        {
            // load data from archive
            boost::archive::binary_iarchive ia(ifs);
            // write archive to class instance
            ia >> *this;
            // ia >> std::cout;
            // archive and stream closed when destructors are called
            return true;
        }
        else
        {
            std::cout << "No input file named " << filename << " with trained models available" << endl;
            return false;
        }
    }

    static std::tr1::tuple<pointCloudPointXYZPtr,pointCloudPointNormalPtr,float> loadModelPLY(std::string & filename, std::string & frameId, float scale)
    {
        vtkSmartPointer<vtkPolyData> polygonPolyData = vtkSmartPointer<vtkPolyData>::New();
        // Read File
        std::cout << "Reading file " << filename << "..." << std::endl;
        vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
        std::cout << "Reading " << filename << std::endl;
        reader->SetFileName(filename.c_str());
        reader->Update();
        polygonPolyData->DeepCopy(reader->GetOutput());
        return generateInputSurfletModel(polygonPolyData, scale, frameId);
    }

    pointCloudPointNormalPtr loadModel(std::string & filename, std::string & frameId, float scale)
    {
        vtkSmartPointer<vtkPolyData> polygonPolyData = vtkSmartPointer<vtkPolyData>::New();
        // Read File
        std::cout << "Reading file " << filename << "..." << std::endl;
        vtkSmartPointer<vtkOBJReader> reader =
                vtkSmartPointer<vtkOBJReader>::New();
        reader->SetFileName(filename.c_str());
        reader->Update();


        polygonPolyData->DeepCopy(reader->GetOutput());

        return processData(polygonPolyData);
    }


private:
    // Methods
    friend class boost::serialization::access;
    //serialization methods
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & objectModels;
    }

    static std::tr1::tuple<pointCloudPointNormalPtr, float, float, shape_msgs::MeshPtr> loadModelData(std::string modelId, float scale, bool symmetric, std::string dbname, std::string host, std::string port, std::string user, std::string password, std::string frameId)
    {
        std::vector<Point3d> vertices;
        std::vector<int> polygons;
        PGconn    *conn;
        PGresult  *res;
        std::string query;
        std::string connect;
        connect="dbname=" + dbname + " host=" + host + " port=" + port + " user=" + user + " password=" + password;

        //for binary data...
        const char* result_char;
        size_t length;

        std::cout << " Getting model point cloud for the object "<< modelId.c_str() << " from database "<< dbname.c_str() << " hosted on " << host.c_str() << "..." << std::endl;
        //----------------------------------
        // init

        conn = PQconnectdb(connect.c_str());
        printf("%s\n", connect.c_str());
        std::cout << CONNECTION_BAD << std::endl;
        if (PQstatus(conn) == CONNECTION_BAD)
        {
            puts("We were unable to connect to the database");
            exit(0);
        }

        //----------------------------------
        // get mesh polygons (binary transfer)
        query = "select mesh_triangle_list from mesh where original_model_id=" + modelId;
        res = PQexecParams(conn, query.c_str(), 0, NULL, NULL, NULL, NULL, 1); //last argument is resultFormat... = 0 (text) ... = 1 (binary)

        result_char =  PQgetvalue(res, 0, 0);
        length = PQgetlength(res, 0, 0);

        //printf("> Mesh (polygons): %d bytes.\n", (int)length);

        if(length % sizeof(int) != 0){
            std::cout << "Length does not make sense!"<< std::endl;
            std::cout << "Lenght:" <<length << std::endl;
            exit(0);
        }

        polygons.resize(length/sizeof(int));

        std::cout << "length: "<< length << std::endl;
        memcpy(&((polygons)[0]), result_char, length);

        PQclear(res);

        //----------------------------------
        // get mesh vertices (binary transfer)

        query = "select mesh_vertex_list from mesh where original_model_id=" + modelId;
        res = PQexecParams(conn, query.c_str(), 0, NULL, NULL, NULL, NULL, 1); //last argument is resultFormat... = 0 (text) ... = 1 (binary)

        result_char =  PQgetvalue(res, 0, 0);
        length = PQgetlength(res, 0, 0);

        //printf("> Mesh (vertices): %d bytes.\n", (int)length);
        if(length % (3*sizeof(double)) != 0)
        {
            std::cout << "Length does not make sense!" << std::endl;
            exit(0);
        }

        vertices.resize( length/(3*sizeof(double)) );
        memcpy(&((vertices)[0]), result_char, length);

        PQclear(res);

        PQfinish(conn);
        //----------------------------------
        // clean

        ////////////////////
        // CONVERT TO VTK //
        ////////////////////

        std::cout << "Convert data to VTK" << std::endl;
        // Insert all points
        vtkSmartPointer<vtkCellArray> polygonsVTK = vtkSmartPointer<vtkCellArray>::New();
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        for(size_t v=0; v < vertices.size(); ++v)
        {
            points->InsertNextPoint ( (vertices)[v].x, (vertices)[v].y, (vertices)[v].z) ;
        }

        if(polygons.size() % 3 == 0)
        {
            // TRIANGLE POLYGON
            for(size_t t=0; t < polygons.size(); t+=3)
            {
                vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
                polygon->GetPointIds()->SetNumberOfIds(3); //make a quad
                polygon->GetPointIds()->SetId(0, polygons[t]);
                polygon->GetPointIds()->SetId(1, polygons[t+1]);
                polygon->GetPointIds()->SetId(2, polygons[t+2]);
                polygonsVTK->InsertNextCell ( polygon );
            }

        }
        /*	else if(polygons.size() % 4 == 0)
            {
                // QUADRILATERAL POLYGON
                for(size_t t=0; t < polygons.size(); t+=4)
                {
                    vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
                    polygon->GetPointIds()->SetNumberOfIds(4); //make a quad
                    polygon->GetPointIds()->SetId(0, polygons[t]);
                    polygon->GetPointIds()->SetId(1, polygons[t+1]);
                    polygon->GetPointIds()->SetId(2, polygons[t+2]);
                    polygon->GetPointIds()->SetId(3, polygons[t+3]);
                    polygonsVTK->InsertNextCell ( polygon );
                }

                //vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
                //triangleFilter->SetInputConnection(sphereSource->GetOutputPort());
                //triangleFilter->Update();
            }*/
        else
        {
            printf("Polygon vector should have entries multiple of 3 or 4\n");
            exit(0);
        }

        // Create a polydata object
        vtkSmartPointer<vtkPolyData> polygonPolyData = vtkSmartPointer<vtkPolyData>::New();

        // Add the geometry and topology to the polydata
        polygonPolyData->SetPoints(points);
        polygonPolyData->SetPolys(polygonsVTK);

        std::cout << "Done" << std::endl;

        int numberOfPoints=polygonPolyData->GetNumberOfPoints();
        float xCentroid=0;
        float yCentroid=0;
        double p[3];

        ////////////////////////
        // REMOVE XY CENTROID //
        ////////////////////////

        // Get XY centroid
        for(int v=0; v < numberOfPoints; ++v)
        {
            polygonPolyData->GetPoints()->GetPoint(v, p);

            xCentroid+=p[0];
            yCentroid+=p[1];
        }
        xCentroid/=numberOfPoints;
        yCentroid/=numberOfPoints;
        std::cout << "XY centroid (before): " << xCentroid << " " << yCentroid << std::endl;

        vtkSmartPointer<vtkTransform> translation = vtkSmartPointer<vtkTransform>::New();
        translation->Translate(-xCentroid, -yCentroid, 0.000000);

        vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
#if VTK_MAJOR_VERSION <= 5
        transformFilter->SetInputConnection(polygonPolyData->GetProducerPort());
#else
        transformFilter->SetInputData(polygonPolyData);
#endif
        transformFilter->SetTransform(translation);
        transformFilter->Update();
        polygonPolyData->DeepCopy(transformFilter->GetOutput());


        xCentroid=0;
        yCentroid=0;
        // Get XY centroid
        for(int v=0; v < numberOfPoints; ++v)
        {
            polygonPolyData->GetPoints()->GetPoint(v, p);

            xCentroid+=p[0];
            yCentroid+=p[1];
        }
        xCentroid/=numberOfPoints;
        yCentroid/=numberOfPoints;
        std::cout << "XY centroid (after): " << xCentroid << " " << yCentroid << std::endl;

        ////////////////////////////
        // COMPUTE MODEL DIAMETER //
        ////////////////////////////

        float maxSquaredDistance=0.0;
        for(int v1=0; v1 < numberOfPoints; ++v1)
        {
            double p1[3];
            polygonPolyData->GetPoints()->GetPoint(v1, p1);
            for(int v2=0; v2 < numberOfPoints; ++v2)
            {
                double p2[3];
                polygonPolyData->GetPoints()->GetPoint(v2, p2);
                float squaredDistance=(p2[0]-p1[0])*(p2[0]-p1[0]) + (p2[1]-p1[1])*(p2[1]-p1[1]) + (p2[2]-p1[2])*(p2[2]-p1[2]);
                if(squaredDistance>maxSquaredDistance)
                    maxSquaredDistance=squaredDistance;
            }
        }
        std::cout << "max squared distance: " << maxSquaredDistance << std::endl;
        /*static int aux=0;
            if(aux==0)
                maxModelDistSquared=80718.9;
            else if(aux==1)
                maxModelDistSquared=31192.1;
            else if(aux==2)
                maxModelDistSquared=97844.7;
            else if(aux==3)
                maxModelDistSquared=53962.3;
            aux++;*/

        // SAVE PLY FILE

        /*vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
            transform->Scale(1000,1000,1000);


            #if VTK_MAJOR_VERSION <= 5
                transformFilter->SetInputConnection(polygonPolyData->GetProducerPort());
            #else
                transformFilter->SetInputData(polygonPolyData);
            #endif
            transformFilter->SetTransform(transform);

            transformFilter->Update();
            vtkSmartPointer<vtkPolyData> polygonPolyData1000 = vtkSmartPointer<vtkPolyData>::New();
            polygonPolyData1000->DeepCopy(transformFilter->GetOutput());
            vtkSmartPointer<vtkPLYWriter> plyWriter = vtkSmartPointer<vtkPLYWriter>::New();
            plyWriter->SetFileName("DUARTE_CHAMPAGNE.ply");
            plyWriter->SetInput(polygonPolyData1000);
            plyWriter->SetFileTypeToASCII();
            plyWriter->Write();

            exit(0);*/


        ////////////////////
        // TRIANGLES ONLY //
        ////////////////////
        std::cout << " number of polys before triangulating:"<< polygonPolyData->GetNumberOfPolys()<< std::endl;
        vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
#if VTK_MAJOR_VERSION <= 5
        triangleFilter->SetInputConnection(polygonPolyData->GetProducerPort());
#else
        triangleFilter->SetInputData(polygonPolyData);
#endif
        triangleFilter->Update();
        polygonPolyData->DeepCopy(triangleFilter->GetOutput());
        std::cout << " number of polys after triangulating:"<< polygonPolyData->GetNumberOfPolys()<< std::endl;

        std::cout << "Get surface area: " << std::endl;
        double surfaceArea=0.0;
        for(vtkIdType i = 0; i < polygonPolyData->GetNumberOfCells(); ++i)
        {
            double p0[3];
            double p1[3];
            double p2[3];
            vtkCell* cell = polygonPolyData->GetCell(i);
            vtkTriangle* triangle = dynamic_cast<vtkTriangle*>(cell);
            triangle->GetPoints()->GetPoint(0, p0);
            triangle->GetPoints()->GetPoint(1, p1);
            triangle->GetPoints()->GetPoint(2, p2);
            surfaceArea += vtkTriangle::TriangleArea(p0, p1, p2);
        }
        std::cout << " surface area: " << surfaceArea << std::endl;

        //////////////////////
        // Generate normals //
        //////////////////////
        std::cout << "Compute Normals" << std::endl;
        std::cout << " number of points before estimating normals:"<< polygonPolyData->GetNumberOfPoints() << std::endl;
        vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
#if VTK_MAJOR_VERSION <= 5
        normalGenerator->SetInput(polygonPolyData);
#else
        normalGenerator->SetInputData(polygonPolyData);
#endif
        normalGenerator->ComputePointNormalsOn();
        normalGenerator->ComputeCellNormalsOff();

        /*
                // Optional settings
                normalGenerator->SetFeatureAngle(0.1);
                normalGenerator->SetSplitting(1);
                normalGenerator->SetConsistency(0);
                normalGenerator->SetAutoOrientNormals(0);
                normalGenerator->SetComputePointNormals(1);
                normalGenerator->SetComputeCellNormals(0);
                normalGenerator->SetFlipNormals(0);
                normalGenerator->SetNonManifoldTraversal(1);
                */
        //normalGenerator->SetAutoOrientNormals(1);
        normalGenerator->SetSplitting(0);
        normalGenerator->SetConsistency(1);
        normalGenerator->Update();

        polygonPolyData->DeepCopy(normalGenerator->GetOutput());
        std::cout << " number of points after estimating normals:"<< polygonPolyData->GetNumberOfPoints() << std::endl;

        vtkSmartPointer<vtkFloatArray> pointNormalsRetrieved = vtkFloatArray::SafeDownCast(polygonPolyData->GetPointData()->GetArray("Normals"));
        bool hasPointNormals = GetPointNormals(polygonPolyData);

        if(!hasPointNormals)
        {
            std::cout << "NO NORMALS" << std::endl;
            exit(-1);
        }
        std::cout << "Done" << std::endl;

        shape_msgs::MeshPtr mesh_msg(new shape_msgs::Mesh);
        for(vtkIdType i = 0; i < polygonPolyData->GetNumberOfCells(); ++i)
        {
            double p0[3];
            double p1[3];
            double p2[3];
            vtkCell* cell = polygonPolyData->GetCell(i);
            vtkTriangle* triangle = dynamic_cast<vtkTriangle*>(cell);
            triangle->GetPoints()->GetPoint(0, p0);
            triangle->GetPoints()->GetPoint(1, p1);
            triangle->GetPoints()->GetPoint(2, p2);

            shape_msgs::MeshTriangle triangle_msg;
            triangle_msg.vertex_indices[0]=triangle->GetPointId(0);
            triangle_msg.vertex_indices[1]=triangle->GetPointId(1);
            triangle_msg.vertex_indices[2]=triangle->GetPointId(2);

            mesh_msg->triangles.push_back(triangle_msg);
            geometry_msgs::Point point_msg1;
            point_msg1.x=p0[0];
            point_msg1.y=p0[1];
            point_msg1.z=p0[2];
            mesh_msg->vertices.push_back(point_msg1);
            geometry_msgs::Point point_msg2;
            point_msg2.x=p1[0];
            point_msg2.y=p1[1];
            point_msg2.z=p1[2];
            mesh_msg->vertices.push_back(point_msg2);
            geometry_msgs::Point point_msg3;
            point_msg3.x=p2[0];
            point_msg3.y=p2[1];
            point_msg3.z=p2[2];
            mesh_msg->vertices.push_back(point_msg3);
        }

        //////////////////////////////////////////////////////////////////////////
        // SUB DIVIDE TO GUARANTEE UNIFORM SAMPLING AFTER THE DOWNSAMPLING STEP //
        //////////////////////////////////////////////////////////////////////////

        std::cout << " number of points before subdividing(interpolating):"<< polygonPolyData->GetNumberOfPoints()<< std::endl;

        for(vtkIdType i = 0; i < polygonPolyData->GetNumberOfCells(); )
        {

            double p0[3];
            double p1[3];
            double p2[3];
            vtkCell* cell = polygonPolyData->GetCell(i);
            vtkTriangle* triangle = dynamic_cast<vtkTriangle*>(cell);
            triangle->GetPoints()->GetPoint(0, p0);
            triangle->GetPoints()->GetPoint(1, p1);
            triangle->GetPoints()->GetPoint(2, p2);

            double d01 = vtkMath::Distance2BetweenPoints(p0, p1);
            double d02 = vtkMath::Distance2BetweenPoints(p0, p2);
            double d12 = vtkMath::Distance2BetweenPoints(p1, p2);
            //std::cout << i << "ya:" << d01 << " " << d02 << " " << d12 << std::endl;
            if(d01>0.0001 || d02>0.0001 || d12>0.0001)
            {
                std::cout << i << "yo:" << d01 << " " << d02 << " " << d12 << std::endl;
                ///////////////
                // Subdivide //
                ///////////////

                vtkSmartPointer<vtkLinearSubdivisionFilter> subdivisionFilter = vtkSmartPointer<vtkLinearSubdivisionFilter>::New();
                float numberDivisions=1;
                //	std::cout << "MIN DISTANCE:" << minDistance <<"DIVISIONS:" <<numberDivisions<< std::endl;
                subdivisionFilter->SetNumberOfSubdivisions(numberDivisions);


#if VTK_MAJOR_VERSION <= 5
                subdivisionFilter->SetInputConnection(polygonPolyData->GetProducerPort());
#else
                subdivisionFilter->SetInputData(polygonPolyData);
#endif
                subdivisionFilter->Update();
                std::cout << " number of points before subdividing(interpolating):"<< polygonPolyData->GetNumberOfPoints()<< std::endl;
                polygonPolyData->DeepCopy(subdivisionFilter->GetOutput());

                i=0;
            }
            else ++i;
        }




        //polygonPolyData=interpolateTrianglePoly(polygonPolyData, 0.001); //change this back to 0.001
        std::cout << " number of points after subdividing:"<< polygonPolyData->GetNumberOfPoints()<< std::endl;
        //visualizeMeshes(decimated,polygonsDecimatedWithNormals);
        return std::tr1::tuple<pointCloudPointNormalPtr, float, float, shape_msgs::MeshPtr>(generateInputSurfletModel(polygonPolyData, scale, frameId), surfaceArea, maxSquaredDistance, mesh_msg);
    }


    pointCloudPointNormalPtr processData(vtkSmartPointer<vtkPolyData> polygonPolyData)
    {
        int numberOfPoints=polygonPolyData->GetNumberOfPoints();
        float xCentroid=0;
        float yCentroid=0;
        double p[3];

        ////////////////////////////
        // COMPUTE MODEL DIAMETER //
        ////////////////////////////

        float maxSquaredDistance=0.0;
        for(int v1=0; v1 < numberOfPoints; ++v1)
        {
            double p1[3];
            polygonPolyData->GetPoints()->GetPoint(v1, p1);
            for(int v2=0; v2 < numberOfPoints; ++v2)
            {
                double p2[3];
                polygonPolyData->GetPoints()->GetPoint(v2, p2);
                float squaredDistance=(p2[0]-p1[0])*(p2[0]-p1[0]) + (p2[1]-p1[1])*(p2[1]-p1[1]) + (p2[2]-p1[2])*(p2[2]-p1[2]);
                if(squaredDistance>maxSquaredDistance)
                    maxSquaredDistance=squaredDistance;
            }
        }
        /*static int aux=0;
            if(aux==0)
                maxModelDistSquared=80718.9;
            else if(aux==1)
                maxModelDistSquared=31192.1;
            else if(aux==2)
                maxModelDistSquared=97844.7;
            else if(aux==3)
                maxModelDistSquared=53962.3;
            aux++;*/

        // SAVE PLY FILE

        /*vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
            transform->Scale(1000,1000,1000);


            #if VTK_MAJOR_VERSION <= 5
                transformFilter->SetInputConnection(polygonPolyData->GetProducerPort());
            #else
                transformFilter->SetInputData(polygonPolyData);
            #endif
            transformFilter->SetTransform(transform);

            transformFilter->Update();
            vtkSmartPointer<vtkPolyData> polygonPolyData1000 = vtkSmartPointer<vtkPolyData>::New();
            polygonPolyData1000->DeepCopy(transformFilter->GetOutput());
            vtkSmartPointer<vtkPLYWriter> plyWriter = vtkSmartPointer<vtkPLYWriter>::New();
            plyWriter->SetFileName("DUARTE_CHAMPAGNE.ply");
            plyWriter->SetInput(polygonPolyData1000);
            plyWriter->SetFileTypeToASCII();
            plyWriter->Write();

            exit(0);*/


        ////////////////////
        // TRIANGLES ONLY //
        ////////////////////
        std::cout << " number of polys before triangulating:"<< polygonPolyData->GetNumberOfPolys()<< std::endl;
        vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
#if VTK_MAJOR_VERSION <= 5
        triangleFilter->SetInputConnection(polygonPolyData->GetProducerPort());
#else
        triangleFilter->SetInputData(polygonPolyData);
#endif
        triangleFilter->Update();
        polygonPolyData->DeepCopy(triangleFilter->GetOutput());
        std::cout << " number of polys after triangulating:"<< polygonPolyData->GetNumberOfPolys()<< std::endl;

        std::cout << "Get surface area: " << std::endl;
        double surfaceArea=0.0;
        for(vtkIdType i = 0; i < polygonPolyData->GetNumberOfCells(); ++i)
        {
            double p0[3];
            double p1[3];
            double p2[3];
            vtkCell* cell = polygonPolyData->GetCell(i);
            vtkTriangle* triangle = dynamic_cast<vtkTriangle*>(cell);
            triangle->GetPoints()->GetPoint(0, p0);
            triangle->GetPoints()->GetPoint(1, p1);
            triangle->GetPoints()->GetPoint(2, p2);
            surfaceArea += vtkTriangle::TriangleArea(p0, p1, p2);
        }
        std::cout << " surface area: " << surfaceArea << std::endl;

        //////////////////////
        // Generate normals //
        //////////////////////
        std::cout << "Compute Normals" << std::endl;
        std::cout << " number of points before estimating normals:"<< polygonPolyData->GetNumberOfPoints() << std::endl;
        vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
#if VTK_MAJOR_VERSION <= 5
        normalGenerator->SetInput(polygonPolyData);
#else
        normalGenerator->SetInputData(polygonPolyData);
#endif
        normalGenerator->ComputePointNormalsOn();
        normalGenerator->ComputeCellNormalsOff();

        /*
                // Optional settings
                normalGenerator->SetFeatureAngle(0.1);
                normalGenerator->SetSplitting(1);
                normalGenerator->SetConsistency(0);
                normalGenerator->SetAutoOrientNormals(0);
                normalGenerator->SetComputePointNormals(1);
                normalGenerator->SetComputeCellNormals(0);
                normalGenerator->SetFlipNormals(0);
                normalGenerator->SetNonManifoldTraversal(1);
                */
        //normalGenerator->SetAutoOrientNormals(1);
        normalGenerator->SetSplitting(0);
        normalGenerator->SetConsistency(1);
        normalGenerator->Update();

        polygonPolyData->DeepCopy(normalGenerator->GetOutput());
        std::cout << " number of points after estimating normals:"<< polygonPolyData->GetNumberOfPoints() << std::endl;

        vtkSmartPointer<vtkFloatArray> pointNormalsRetrieved = vtkFloatArray::SafeDownCast(polygonPolyData->GetPointData()->GetArray("Normals"));
        bool hasPointNormals = GetPointNormals(polygonPolyData);

        if(!hasPointNormals)
        {
            std::cout << "NO NORMALS" << std::endl;
            exit(-1);
        }
        std::cout << "Done" << std::endl;


        //////////////////////////////////////////////////////////////////////////
        // SUB DIVIDE TO GUARANTEE UNIFORM SAMPLING AFTER THE DOWNSAMPLING STEP //
        //////////////////////////////////////////////////////////////////////////

        std::cout << " number of points before subdividing(interpolating):"<< polygonPolyData->GetNumberOfPoints()<< std::endl;
        //polygonPolyData=interpolateTrianglePoly(polygonPolyData, 0.001); //change this back to 0.001




        //////////////////////////////////////////////////////////////////////////
        // SUB DIVIDE TO GUARANTEE UNIFORM SAMPLING AFTER THE DOWNSAMPLING STEP //
        //////////////////////////////////////////////////////////////////////////

        for(vtkIdType i = 0; i < polygonPolyData->GetNumberOfCells(); )
        {

            double p0[3];
            double p1[3];
            double p2[3];
            vtkCell* cell = polygonPolyData->GetCell(i);
            vtkTriangle* triangle = dynamic_cast<vtkTriangle*>(cell);
            triangle->GetPoints()->GetPoint(0, p0);
            triangle->GetPoints()->GetPoint(1, p1);
            triangle->GetPoints()->GetPoint(2, p2);

            double d01 = vtkMath::Distance2BetweenPoints(p0, p1);
            double d02 = vtkMath::Distance2BetweenPoints(p0, p2);
            double d12 = vtkMath::Distance2BetweenPoints(p1, p2);
            //std::cout << i << "ya:" << d01 << " " << d02 << " " << d12 << std::endl;
            if(d01>0.0001 || d02>0.0001 || d12>0.0001)
            {
                std::cout << i << "yo:" << d01 << " " << d02 << " " << d12 << std::endl;
                ///////////////
                // Subdivide //
                ///////////////

                vtkSmartPointer<vtkLinearSubdivisionFilter> subdivisionFilter = vtkSmartPointer<vtkLinearSubdivisionFilter>::New();
                float numberDivisions=1;
                //	std::cout << "MIN DISTANCE:" << minDistance <<"DIVISIONS:" <<numberDivisions<< std::endl;
                subdivisionFilter->SetNumberOfSubdivisions(numberDivisions);


#if VTK_MAJOR_VERSION <= 5
                subdivisionFilter->SetInputConnection(polygonPolyData->GetProducerPort());
#else
                subdivisionFilter->SetInputData(polygonPolyData);
#endif
                subdivisionFilter->Update();
                std::cout << " number of points before subdividing(interpolating):"<< polygonPolyData->GetNumberOfPoints()<< std::endl;
                polygonPolyData->DeepCopy(subdivisionFilter->GetOutput());

                i=0;
            }
            else ++i;
        }

        std::cout << " number of points after subdividing:"<< polygonPolyData->GetNumberOfPoints()<< std::endl;
        //visualizeMeshes(decimated,polygonsDecimatedWithNormals);

        double scale=1.0;
        std::string frame_id="object_frame";
        return generateInputSurfletModel(polygonPolyData, scale, frame_id);

    }


    static pointCloudPointNormalPtr generateInputSurfletModel(vtkSmartPointer<vtkPolyData> polygonPolyData, float scale, std::string & frameId)
    {
        pointCloudPointXYZPtr pclPoints (new pointCloudPointXYZ);
        pcl::PointCloud<pcl::Normal>::Ptr pclNormals (new pointCloudNormal);
        pointCloudPointNormalPtr pclSurflets (new pointCloudPointNormal);
        double p[3];
        //double p2[3];
        double pN[3];

        ////////////////////
        // CONVERT TO PCL //
        ////////////////////
        std::cout << "Convert surflet point cloud to PCL" << std::endl;
        pclSurflets->header.frame_id=frameId;
        int numberOfPoints=polygonPolyData->GetNumberOfPoints();

        ///////// Get Point Normals ///////////
        vtkSmartPointer<vtkFloatArray> pointNormalsRetrieved2 = vtkFloatArray::SafeDownCast(polygonPolyData->GetPointData()->GetArray("Normals"));

        // Convert to pcl pcl::PointCloud
        for(int v=0; v < numberOfPoints; ++v)
        {
            //Add point
            polygonPolyData->GetPoints()->GetPoint(v, p);

            //Add normal
            pointNormalsRetrieved2->GetTuple(v, pN);
            //if(pN[2]>0.01||pN[2]<-0.01) // Cyllinder stuff
            //                   continue;

            pclPoints->push_back(pcl::PointXYZ(p[0], p[1], p[2]));
            pcl::Normal aux;

            //std::cout << pclPoints->back() << std::endl;
            double inner;

            //Eigen::Vector3f normal(p[0]-0.0, p[1]-0.0,0.0); // CILINDRO E CONE
            Eigen::Vector3f normal(pN[0], pN[1],pN[2]);
            normal.normalize();

            pclNormals->push_back(pcl::Normal(normal.x(),normal.y(),normal.z()));
        }

        // Concatenate points with normals
        pcl::concatenateFields(*pclPoints, *pclNormals, *pclSurflets);
        std::cout << " Surflet cloud (number of points=" << pclSurflets->size() << ")" << std::endl;

        if(!equalFloat(scale,1.0000))
        {
            pointCloudPointNormalPtr pclCloudOut(new pcl::PointCloud<pcl::PointNormal>);
            Eigen::Transform<float,3,Eigen::Affine> scaleTransform=Eigen::Translation<float,3>(0,0,0)*Eigen::UniformScaling<float> (scale);
            pcl::transformPointCloudWithNormals(*pclSurflets, *pclCloudOut, scaleTransform);
            pclSurflets=pclCloudOut;
        }


        std::cout << "Done" << std::endl;



        /*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = objectModel::viewportsVis(pclSurflets);

    while (!viewer2->wasStopped ())
    {
        viewer2->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }*/

        //clouds.first=pclPointsOriginal;
        //clouds.first=pclPoints;
        //clouds.second=pclSurflets;


        return pclSurflets;
    }

    static bool GetPointNormals(vtkPolyData* polydata)
    {
        std::cout << "In GetPointNormals: " << polydata->GetNumberOfPoints() << std::endl;
        std::cout << "Looking for point normals..." << std::endl;

        // Count points
        vtkIdType numPoints = polydata->GetNumberOfPoints();
        std::cout << "There are " << numPoints << " points." << std::endl;

        // Count triangles
        vtkIdType numPolys = polydata->GetNumberOfPolys();
        std::cout << "There are " << numPolys << " polys." << std::endl;

        ////////////////////////////////////////////////////////////////
        // Double normals in an array
        vtkDoubleArray* normalDataDouble =
                vtkDoubleArray::SafeDownCast(polydata->GetPointData()->GetArray("Normals"));

        if(normalDataDouble)
        {
            int nc = normalDataDouble->GetNumberOfTuples();
            std::cout << "There are " << nc
                      << " components in normalDataDouble" << std::endl;
            return true;
        }

        ////////////////////////////////////////////////////////////////
        // Double normals in an array
        vtkFloatArray* normalDataFloat =
                vtkFloatArray::SafeDownCast(polydata->GetPointData()->GetArray("Normals"));

        if(normalDataFloat)
        {
            int nc = normalDataFloat->GetNumberOfTuples();
            std::cout << "There are " << nc
                      << " components in normalDataFloat" << std::endl;
            return true;
        }

        ////////////////////////////////////////////////////////////////
        // Point normals
        vtkDoubleArray* normalsDouble =
                vtkDoubleArray::SafeDownCast(polydata->GetPointData()->GetNormals());

        if(normalsDouble)
        {
            std::cout << "There are " << normalsDouble->GetNumberOfComponents()
                      << " components in normalsDouble" << std::endl;
            return true;
        }

        ////////////////////////////////////////////////////////////////
        // Point normals
        vtkFloatArray* normalsFloat =
                vtkFloatArray::SafeDownCast(polydata->GetPointData()->GetNormals());

        if(normalsFloat)
        {
            std::cout << "There are " << normalsFloat->GetNumberOfComponents()
                      << " components in normalsFloat" << std::endl;
            return true;
        }

        /////////////////////////////////////////////////////////////////////
        // Generic type point normals
        vtkDataArray* normalsGeneric = polydata->GetPointData()->GetNormals(); //works
        if(normalsGeneric)
        {
            std::cout << "There are " << normalsGeneric->GetNumberOfTuples()
                      << " normals in normalsGeneric" << std::endl;

            double testDouble[3];
            normalsGeneric->GetTuple(0, testDouble);

            std::cout << "Double: " << testDouble[0] << " "
                      << testDouble[1] << " " << testDouble[2] << std::endl;

            // Can't do this:
            /*
            float testFloat[3];
            normalsGeneric->GetTuple(0, testFloat);

            std::cout << "Float: " << testFloat[0] << " "
                  << testFloat[1] << " " << testFloat[2] << std::endl;
            */
            return true;
        }


        // If the function has not yet quit, there were none of these types of normals
        std::cout << "Normals not found!" << std::endl;
        return false;

    }

    static void visualizeMeshes(vtkSmartPointer<vtkPolyData> data1,vtkSmartPointer<vtkPolyData> data2)
    {

        // Create a mapper and actor

        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        vtkSmartPointer<vtkPolyDataMapper> mapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();

#if VTK_MAJOR_VERSION <= 5
        mapper->SetInput(data1);
        mapper2->SetInput(data2);
#else
        mapper->SetInputData(data1);
        mapper2->SetInputData(data2);
#endif

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        vtkSmartPointer<vtkActor> actor2 = vtkSmartPointer<vtkActor>::New();
        actor2->SetMapper(mapper2);

        // There will be one render window
        vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
        renderWindow->SetSize(600, 300);

        // And one interactor
        vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
        interactor->SetRenderWindow(renderWindow);
        // Define viewport ranges
        // (xmin, ymin, xmax, ymax)
        double leftViewport[4] = {0.0, 0.0, 0.5, 1.0};
        double rightViewport[4] = {0.5, 0.0, 1.0, 1.0};

        // Setup both renderers
        vtkSmartPointer<vtkRenderer> leftRenderer = vtkSmartPointer<vtkRenderer>::New();
        renderWindow->AddRenderer(leftRenderer);
        leftRenderer->SetViewport(leftViewport);
        leftRenderer->SetBackground(.6, .5, .4);

        vtkSmartPointer<vtkRenderer> rightRenderer = vtkSmartPointer<vtkRenderer>::New();
        renderWindow->AddRenderer(rightRenderer);
        rightRenderer->SetViewport(rightViewport);
        rightRenderer->SetBackground(.4, .5, .6);

        // Add the sphere to the left and the cube to the right
        leftRenderer->AddActor(actor);
        rightRenderer->AddActor(actor2);

        vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
        leftRenderer->SetActiveCamera(camera);
        rightRenderer->SetActiveCamera(camera);

        leftRenderer->ResetCamera();

        renderWindow->Render();
        interactor->Start();
    }
    //std::vector<btVector3>

    /*! Given a triangle defined by three vertices, returns a set of points obtained
          by sampling the surface of the triangle. Points are obtained by barycentric
          interpolation, with a guarantee that, along the interpolated directions, the
          distance between interpolated locations is not greater than min_res  (could be
          smaller if one of the 0-1 and 0-2 edges of the triangle is significantly shorter
          than the other).

          The vertices themselves are NOT returned in the set of points.
        */
    static vtkSmartPointer<vtkPolyData> interpolateTriangle(vtkSmartPointer<vtkPolyData> polygonPolyData, double min_res)
    {
        vtkSmartPointer<vtkPolyData> polygonPolyDataFinal =  vtkSmartPointer<vtkPolyData>::New();
        vtkSmartPointer<vtkCell> poly;
        vtkSmartPointer<vtkIdList> polyPointList;

        double p[3][3];
        vtkSmartPointer<vtkPoints> pointsFinal = vtkSmartPointer<vtkPoints>::New();
        pointsFinal->DeepCopy(polygonPolyData->GetPoints());

        vtkAppendPolyData* append = vtkAppendPolyData::New();

        //  For each poly...compute new points
        int numberOfPolys=polygonPolyData->GetNumberOfPolys();
        for(vtkIdType polyId=0; polyId < numberOfPolys; ++polyId)
        {
            poly = polygonPolyData->GetCell(polyId);
            polyPointList = poly->GetPointIds();


            // Create a cell array to store the polygon in
            vtkSmartPointer<vtkCellArray> borderPolygonCellArray = vtkSmartPointer<vtkCellArray>::New();
            // Define the polygonal border
            vtkSmartPointer<vtkPolygon> aPolygon = vtkSmartPointer<vtkPolygon>::New();

            vtkIdType pid[1];

            // Points of the entire polygon
            vtkSmartPointer<vtkPoints> pointsPolygon = vtkSmartPointer<vtkPoints>::New();
            vtkSmartPointer<vtkCellArray> vertsPolygon = vtkSmartPointer<vtkCellArray>::New();
            //std::cout << "Poly: " << polyId << std::endl;
            for(vtkIdType i = 0; i < polyPointList->GetNumberOfIds(); ++i)
            {
                //std::cout << "Point Id: " << polyPointList->GetId(i) << std::endl;
                polygonPolyData->GetPoint(polyPointList->GetId(i), &(p[i][0]) );
                //std::cout << "\tPoint: " << polyPointList->GetId(i) << " : " << p[i][0] << " " << p[i][1] << " " <<p[i][2] << std::endl;

                // Point id (border)
                aPolygon->GetPointIds()->InsertNextId(polyPointList->GetId(i));

                pid[0]=pointsPolygon->InsertNextPoint(p[i]);
                //vertsPolygon->InsertNextCell ( 1,pid);
            }

            borderPolygonCellArray->InsertNextCell(aPolygon);

            // Create a polydata to store the boundary. The points must be the
            // same as the points we will triangulate.
            vtkSmartPointer<vtkPolyData> boundary = vtkSmartPointer<vtkPolyData>::New();

            //find out the interpolation resolution for the first coordinate
            //based on the size of the 0-1 and 0-2 edges

            // Find the squared distance between the points.
            double d01 = vtkMath::Distance2BetweenPoints(p[0], p[1]);
            double d02 = vtkMath::Distance2BetweenPoints(p[0], p[2]);
            // Take the square root to get the Euclidean distance between the points.
            d01=sqrt(d01);
            d02=sqrt(d02);

            double res_0 = min_res / std::max(d01, d02);

            //perform the first interpolation
            //we do not want the vertices themselves, so we don't start at 0
            double t0=res_0;
            bool done = false;

            while (!done)
            {
                if (t0 >= 1.0)
                {
                    t0 = 1.0;
                    done = true;
                }
                //compute the resolution for the second interpolation
                double p1[3];
                double p2[3];
                p1[0] = t0*p[0][0] + (1-t0) * p[1][0];
                p1[1] = t0*p[0][1] + (1-t0) * p[1][1];
                p1[2] = t0*p[0][2] + (1-t0) * p[1][2];

                p2[0] = t0*p[0][0] + (1-t0) * p[2][0];
                p2[1] = t0*p[0][1] + (1-t0) * p[2][1];
                p2[2] = t0*p[0][2] + (1-t0) * p[2][2];

                double d12 = vtkMath::Distance2BetweenPoints(p1, p2);
                d12=sqrt(d12);
                //std::cout << "d12:" << d12 << std::endl;
                double res_12 = min_res / d12;
                //std::cout << "res12:" << res_12 << std::endl;
                //std::cout << "t0:" << t0 << std::endl;
                //perform the second interpolation
                double t12 = 0.0;
                bool done12 = false;
                while (!done12)
                {
                    if (t12 >= 1.0)
                    {
                        t12 = 1.0;
                        done12 = true;
                    }
                    //std::cout << " t12:" << t12 << std::endl;
                    //actual point insertion
                    //do not insert the vertices themselves
                    if((t0!=1.0 && t0!=0.0) || (t12!=0.0 && t12!=1.0))
                    {
                        double aux[3];
                        aux[0]=t12*p1[0] + (1.0 - t12)*p2[0];
                        aux[1]=t12*p1[1] + (1.0 - t12)*p2[1];
                        aux[2]=t12*p1[2] + (1.0 - t12)*p2[2];

                        pointsFinal->InsertNextPoint(aux);
                        pointsPolygon->InsertNextPoint(aux);
                    }
                    t12 += res_12;
                }
                t0 += res_0;
            }

            vtkSmartPointer<vtkPolyData> aPolyData = vtkSmartPointer<vtkPolyData>::New();
            aPolyData->SetPoints(pointsPolygon);

            boundary->SetPoints(pointsPolygon);
            boundary->SetPolys(borderPolygonCellArray);

            // Triangulate the grid points
            vtkSmartPointer<vtkDelaunay2D> delaunay = vtkSmartPointer<vtkDelaunay2D>::New();
#if VTK_MAJOR_VERSION <= 5
            delaunay->SetInput(aPolyData);
            // delaunay->SetSource(boundary);
#else
            delaunay->SetInputData(aPolyData);
            // delaunay->SetSourceData(boundary);
#endif
            delaunay->SetProjectionPlaneMode(2);
            delaunay->Update();
            append->AddInput(delaunay->GetOutput());
            append->Update();

            // Visualize
            vtkSmartPointer<vtkPolyDataMapper> meshMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            meshMapper->SetInputConnection(delaunay->GetOutputPort());

            vtkSmartPointer<vtkActor> meshActor = vtkSmartPointer<vtkActor>::New();
            meshActor->SetMapper(meshMapper);

            vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
#if VTK_MAJOR_VERSION <= 5
            glyphFilter->SetInputConnection(aPolyData->GetProducerPort());
#else
            glyphFilter->SetInputData(aPolyData);
#endif
            glyphFilter->Update();

            vtkSmartPointer<vtkPolyDataMapper> pointMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            pointMapper->SetInputConnection(glyphFilter->GetOutputPort());

            vtkSmartPointer<vtkActor> pointActor = vtkSmartPointer<vtkActor>::New();
            pointActor->GetProperty()->SetColor(1,0,0);
            pointActor->GetProperty()->SetPointSize(3);
            pointActor->SetMapper(pointMapper);

            vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
            vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
            renderWindow->AddRenderer(renderer);
            vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
            renderWindowInteractor->SetRenderWindow(renderWindow);

            renderer->AddActor(meshActor);
            renderer->AddActor(pointActor);
            renderer->SetBackground(.3, .6, .3); // Background color green

            renderWindow->Render();
            renderWindowInteractor->Start();
        }

        polygonPolyDataFinal->DeepCopy(append->GetOutput());

        return polygonPolyDataFinal;
    }




    static vtkSmartPointer<vtkPolyData> interpolateTrianglePoly(vtkSmartPointer<vtkPolyData> polygonPolyData, double min_res)
    {
        vtkSmartPointer<vtkPolyData> polygonPolyDataFinal =  vtkSmartPointer<vtkPolyData>::New();
        vtkSmartPointer<vtkCell> poly;
        vtkSmartPointer<vtkIdList> polyPointList;

        double p[3][3];
        float pN[3];
        vtkSmartPointer<vtkPoints> pointsFinal = vtkSmartPointer<vtkPoints>::New();
        pointsFinal->DeepCopy(polygonPolyData->GetPoints());

        ////////////////////////////
        // Generate point normals //
        ////////////////////////////

        vtkSmartPointer<vtkPolyDataNormals> pointNormalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
#if VTK_MAJOR_VERSION <= 5
        pointNormalGenerator->SetInput(polygonPolyData);
#else
        pointNormalGenerator->SetInputData(polygonPolyData);
#endif
        pointNormalGenerator->ComputePointNormalsOn();
        pointNormalGenerator->ComputeCellNormalsOff();

        /*
                // Optional settings
                normalGenerator->SetFeatureAngle(0.1);
                normalGenerator->SetSplitting(1);
                normalGenerator->SetConsistency(0);
                normalGenerator->SetAutoOrientNormals(0);
                normalGenerator->SetComputePointNormals(1);
                normalGenerator->SetComputeCellNormals(0);
                normalGenerator->SetFlipNormals(0);
                normalGenerator->SetNonManifoldTraversal(1);
                */
        //normalGenerator->SetAutoOrientNormals(1);
        pointNormalGenerator->SetSplitting(0);
        pointNormalGenerator->SetConsistency(1);
        pointNormalGenerator->Update();

        vtkSmartPointer<vtkPolyData> pointsWithNormals = vtkSmartPointer<vtkPolyData>::New();
        pointsWithNormals=pointNormalGenerator->GetOutput();


        vtkSmartPointer<vtkFloatArray> pointNormalsRetrieved = vtkFloatArray::SafeDownCast(pointsWithNormals->GetPointData()->GetArray("Normals"));



        //////////////////////////////
        // Generate polygon normals //
        //////////////////////////////

        vtkSmartPointer<vtkPolyDataNormals> polygonNormalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
#if VTK_MAJOR_VERSION <= 5
        polygonNormalGenerator->SetInput(polygonPolyData);
#else
        polygonNormalGenerator->SetInputData(polygonPolyData);
#endif
        polygonNormalGenerator->ComputePointNormalsOff();
        polygonNormalGenerator->ComputeCellNormalsOn();

        /*
                // Optional settings
                normalGenerator->SetFeatureAngle(0.1);
                normalGenerator->SetSplitting(1);
                normalGenerator->SetConsistency(0);
                normalGenerator->SetAutoOrientNormals(0);
                normalGenerator->SetComputePointNormals(1);
                normalGenerator->SetComputeCellNormals(0);
                normalGenerator->SetFlipNormals(0);
                normalGenerator->SetNonManifoldTraversal(1);
                */
        //normalGenerator->SetAutoOrientNormals(1);
        polygonNormalGenerator->SetSplitting(0);
        polygonNormalGenerator->SetConsistency(1);
        polygonNormalGenerator->Update();
        //polygonPolyData = normalGenerator->GetOutput();
        vtkSmartPointer<vtkPolyData> polygonsWithNormals = vtkSmartPointer<vtkPolyData>::New();
        polygonsWithNormals=polygonNormalGenerator->GetOutput();


        vtkSmartPointer<vtkFloatArray> polygonNormalsRetrieved = vtkFloatArray::SafeDownCast(polygonsWithNormals->GetCellData()->GetNormals());


        //  For each poly...compute new points
        int numberOfPolys=polygonPolyData->GetNumberOfPolys();
        for(vtkIdType polyId=0; polyId < numberOfPolys; ++polyId)
        {
            // Get polygon normal (this normal is the same for every point within this polygon)
            polygonNormalsRetrieved->GetTupleValue(polyId, pN);
            poly = polygonPolyData->GetCell(polyId);
            polyPointList = poly->GetPointIds();

            vtkSmartPointer<vtkCellArray> vertsPolygon = vtkSmartPointer<vtkCellArray>::New();

            for(vtkIdType i = 0; i < polyPointList->GetNumberOfIds(); ++i)
            {
                //std::cout << "Point Id: " << polyPointList->GetId(i) << std::endl;
                polygonPolyData->GetPoint(polyPointList->GetId(i), &(p[i][0]) );
                //std::cout << "\tPoint: " << polyPointList->GetId(i) << " : " << p[i][0] << " " << p[i][1] << " " <<p[i][2] << std::endl;
            }

            //find out the interpolation resolution for the first coordinate
            //based on the size of the 0-1 and 0-2 edges

            // Find the squared distance between the points.
            double d01 = vtkMath::Distance2BetweenPoints(p[0], p[1]);
            double d02 = vtkMath::Distance2BetweenPoints(p[0], p[2]);
            // Take the square root to get the Euclidean distance between the points.
            d01=sqrt(d01);
            d02=sqrt(d02);

            double res_0 = min_res / std::max(d01, d02);

            //perform the first interpolation
            //we do not want the vertices themselves, so we don't start at 0
            double t0=res_0;
            bool done = false;

            while (!done)
            {
                if (t0 >= 1.0)
                {
                    t0 = 1.0;
                    done = true;
                }
                //compute the resolution for the second interpolation
                double p1[3];
                double p2[3];
                p1[0] = t0*p[0][0] + (1.0-t0) * p[1][0];
                p1[1] = t0*p[0][1] + (1.0-t0) * p[1][1];
                p1[2] = t0*p[0][2] + (1.0-t0) * p[1][2];

                p2[0] = t0*p[0][0] + (1-t0) * p[2][0];
                p2[1] = t0*p[0][1] + (1.0-t0) * p[2][1];
                p2[2] = t0*p[0][2] + (1.0-t0) * p[2][2];

                double d12 = vtkMath::Distance2BetweenPoints(p1, p2);
                d12=sqrt(d12);
                double res_12 = min_res / d12;
                //perform the second interpolation
                double t12 = 0.0;
                bool done12 = false;
                while (!done12)
                {
                    if (t12 >= 1.0)
                    {
                        t12 = 1.0;
                        done12 = true;
                    }
                    //actual point insertion
                    //do not insert the vertices themselves
                    if((t0!=1.0) || (t12!=0.0 && t12!=1.0))
                    {
                        double aux[3];
                        aux[0]=t12*p1[0] + (1.0 - t12)*p2[0];
                        aux[1]=t12*p1[1] + (1.0 - t12)*p2[1];
                        aux[2]=t12*p1[2] + (1.0 - t12)*p2[2];

                        pointsFinal->InsertNextPoint(aux);
                        pointNormalsRetrieved->InsertNextTupleValue(pN);
                    }
                    t12 += res_12;
                }
                t0 += res_0;
            }
        }

        polygonPolyDataFinal->SetPoints(pointsFinal);
        polygonPolyDataFinal->GetPointData()->SetNormals(pointNormalsRetrieved);

        return polygonPolyDataFinal;
    }


};

#endif //#ifndef MODELS

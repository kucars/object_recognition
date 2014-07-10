#ifndef THREED_GEOMETRIC_MOMENTS
#define THREED_GEOMETRIC_MOMENTS

#include <vector>
#include <pcl/features/normal_3d.h>
#include <pcl/features/feature.h>

class threeDGeometricMoments 
{
	typedef std::vector<double>::iterator zIt;	
	typedef std::vector<std::vector<double> >::iterator yIt;
	typedef std::vector<std::vector<std::vector<double> > >::iterator xIt;
	typedef pcl::PointCloud<pcl::PointXYZ>::iterator pIt;

	public:
		int order;
		std::vector<std::vector< std::vector<double> > > moments;

		threeDGeometricMoments(int _order) : order(_order)
		{
			// pre allocate space for moments
			moments.resize(order);
			for(xIt n=moments.begin(); n<moments.end(); ++n)
			{
				n->resize(order);
				for(yIt o=n->begin(); o<n->end(); ++o)
				{
					o->resize(order);
				}
			}		
		}

		void computeGeometricMoments(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
		{
			//std::cout << "order: " << order << std::endl;
			//std::cout << "size1: " << moments.size() << std::endl;
			//std::cout << "size2: " << moments[0].size() << std::endl;
			//std::cout << "size3: " << moments[0][0].size() << std::endl;
			Eigen::Vector4f centroid;
			pcl::compute3DCentroid(*pointCloud,centroid);

			// COMPUTE CENTRAL MOMENTS
			// for x order
			for(int m=0; m<order; ++m)
				// for y order		
				for(int n=0; n<order; ++n)
					// for z order
					for(int o=0; o<order; ++o)
					{
						moments[m][n][o]=0;
						// for each point
						for(pIt p=pointCloud->begin(); p<pointCloud->end(); ++p)
							moments[m][n][o]+=pow(p->x-centroid[0],m)*pow(p->y-centroid[1],n)*pow(p->z-centroid[2],o);
						// normalize to scale
						double po=pow(moments[0][0][0], (double) (m+n+o+3.0)/3.0 ) ;
						if(!(m==0&&n==0&&o==0))
						{
							moments[m][n][o]=moments[m][n][o]/po;
						}
						std::cout << "moments after(" << m << "," << n << "," << o << "): " << moments[m][n][o] << std::endl;
					}
		}	
};

#endif //#ifndef 3D_GEOMETRIC_MOMENTS

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

#include "MarchingCubes.h"
void exportPointcloud(OFFFILE &m, std::string name){
	std::ofstream exporter;
	exporter.open(name);
	for(auto p: m.points){
		exporter<<p.x<<" "<<p.y<<" "<<p.z<<std::endl;
	}
	exporter.close();
    std::cout <<"FINISH PointCloud export " <<std::endl;
}

void exportMesh(OFFFILE &m, std::string name){
	std::ofstream exporter;
	exporter.open (name);
	exporter <<"OFF"<<std::endl;
	exporter << m.points.size()<<" "<<m.tri_face.size()<<" "<<"0"<<std::endl;
	for(auto p: m.points){
		exporter<<p.x  <<" "<<p.y  <<" "<<p.z <<std::endl;
	}
	for(auto p: m.tri_face){
		exporter<<"3"<<" "<<p.i<<" "<<p.j<<" "<<p.k<<std::endl;
	}
	exporter.close();
    std::cout <<"FINISH Mesh export " <<std::endl;
}
int main(){
    
    std::ifstream readfile;
    readfile.open("../data/bunny.off");
    std::string head;
    readfile >> head;
    int numVertx, numface, numcolor;
    readfile >> numVertx >> numface>> numcolor;
 
    std::vector<Vec3> PointCloud;
    
    float x, y, z, dummy4, dummy5;
    float maxx = 0, maxy = 0, maxz = 0;

	//scale bunny model to density
    for(int i = 0; i < numVertx; i++){
        readfile >> x >> y >> z;
        Vec3 t;
        t.x = int(x / 5 );  
        t.y = int(y / 5 );
        t.z = int(z / 5 );
        PointCloud.push_back(t);
        if(t.x > maxx) maxx = t.x;
        if(t.y > maxy) maxy = t.y;
        if(t.z > maxz) maxz = t.z;
    }
	//set up with max boundary
    int maxxyz = std::max(maxx, std::max(maxy, maxz)) + 1 ;
    int width = maxxyz;  
	int length = maxxyz;
	int height = maxxyz;  

	//init marching cube 
	float isosurface = 0.9;
    MarchingCubes marchingCubes = MarchingCubes(isosurface, width, height, length);


	//insert bunny pointcloud to voxel
	std::vector<std::vector<std::vector<float> > > voxel;
	voxel.resize(height);
	for (int z = 0; z < height; z = z + 1) {
		voxel[z].resize(length);
	}
	for (int z = 0; z < height; z = z + 1) {
		for (int y = 0; y < length; y = y + 1) {
			voxel[z][y].resize(width);
		}
	}
	for (int z = 0; z < height; z = z + 1) {
		for (int y = 0; y < length; y = y + 1) {
			for (int x = 0; x < width; x = x + 1) {
				voxel[z][y][x] = 0;
			}
		}
	}
    for(auto v: PointCloud){
        voxel[v.z][v.y][v.x] = 1;
    }
    


	OFFFILE result;
    result = marchingCubes.execute(voxel);
	exportPointcloud(result, "point.xyz");
	exportMesh(result, "result.off");
 
    return 0;
}
 
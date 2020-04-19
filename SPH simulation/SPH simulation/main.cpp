// SPH simulation.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>

#include "Animation.h"
#include "BccLatticePointGenerator.h"
#include "BoundingBox.h"
#include "Collider.h"
#include "Frame.h"
#include "Matrix3x3.h"
#include "ParticleSystemData.h"
#include "ParticleSystemSolver.h"
#include "PciSphSystemSolver.h"
#include "PhysicsAnimation.h"
#include "PointGenerator.h"
#include "PointHashGridSearcher.h"
#include "PointNeighbourSearcher.h"
#include "Quaternion.h"
#include "SphSpikyKernel.h"
#include "SphStdKernel.h"
#include "SphSystemData.h"
#include "SphSystemSolver.h"
#include "Surface.h"
#include "Transform.h"
#include "Vector3.h"
#include "Box.h"
#include "Plane.h"
#include "VolumeParticleEmitter.h"
#include "RigidBodyCollider.h"
#include "TriangleMesh.h"
#include "Heightfield.h"

void generateInitialVertices(std::vector<Vector3>* vertArray, int width, int depth)
{
	Vector3* vertices = new Vector3[(width) * (depth)];

	//loop through and set vertex positions along the dimensions given
	int i = 0;
	for (int z = 0; z < depth; z++)
	{
		for (int x = 0; x < width; x++)
		{
			Vector3 &newVal = vertArray->at(i);
			newVal.x = x;
			newVal.z = z;
			newVal.y = 0.0f;
			i++;
		}
	}
}

void generateInitialTris(std::vector<Vector3>* triArray, int width, int depth)
{
	Vector3* triangles = new Vector3[(width - 1) * (depth - 1) * 2];
	int tris = 0;
	for (int vert = 0; tris < (width - 1) * (depth - 1) * 2; vert++)
	{
		//6 points in each quadrant on the mesh
		triangles[tris].x = vert;
		triangles[tris].y = vert + width;
		triangles[tris].z = vert + 1;
		triangles[tris + 1].x = vert + 1;
		triangles[tris + 1].y = vert + width;
		triangles[tris + 1].z = vert + width + 1;

		tris += 2;
		if (tris % ((width - 1) * 2) == 0)
		{
			vert++;
		}
	}

	for (int i = 0; i < (width - 1) * (depth - 1) * 2; i++)
	{
		Vector3 &newVal = triArray->at(i);
		newVal = triangles[i];
	}
}

void perlinNoise(int width, int height, int octaves, float Bias, std::vector<Vector3>* vertArray, float* noise_seed)
{
	Vector3* vertices = new Vector3[(width) * (height)];
	float* f_noise_seed = new float[(width) * (height)];
	for (int i = 0; i <= (width) * (height); i++)
	{
		f_noise_seed[i] = noise_seed[i];
	}
	for (int x = 0; x < width; x++)
	{
		for (int z = 0; z < height; z++)
		{
			float noise = 0.0f;
			float scale = 1.0f;
			float scale_acc = 0.0f;

			for (int o = 0; o < octaves; o++)
			{
				int pitch = width;

				for (int j = 0; j < o; j++)
				{
					pitch = (int)(pitch / 2);
				}

				int sample_x_1 = (x / pitch)*pitch;
				int sample_z_1 = (z / pitch)*pitch;

				int sample_x_2 = (sample_x_1 + pitch) % width;
				int sample_z_2 = (sample_z_1 + pitch) % height;

				float blend_x = (float)(x - sample_x_1) / (float)pitch;
				float blend_z = (float)(z - sample_z_1) / (float)pitch;

				float sample_T = (1.0f - blend_x) * f_noise_seed[sample_z_1 * width + sample_x_1] + blend_x * f_noise_seed[sample_z_1 * width + sample_x_2];
				float sample_B = (1.0f - blend_x) * f_noise_seed[sample_z_2 * width + sample_x_1] + blend_x * f_noise_seed[sample_z_2 * width + sample_x_2];

				noise += (blend_z * (sample_B - sample_T) + sample_T) * scale;
				scale_acc += scale;
				scale = scale / Bias;
			}
			vertices[z * (width) + x].y = (noise / scale_acc) * (width*0.5);
		}
	}
	for (int i = 0; i < (width) * (height); ++i)
	{
		//Simply modify each Vector reference
		Vector3 &newVal = vertArray->at(i);
		newVal.y = vertices[i].y;
	}
}

void runSimulation(const PciSphSystemSolverPtr& solver, int numberOfFrames, double fps) 
{	
	auto particles = solver->sphSystemData();

	for (Frame frame(0, 1.0 / fps); frame.index < numberOfFrames; ++frame) 
	{
		solver->Update(frame);
			   
		std::vector<Vector3> positions(particles->numberOfParticles());

		for (size_t i = 0; i < particles->numberOfParticles(); i++)
		{
			if (positions.size() < i)
			{
				positions.push_back(particles->positions().at(i));
			}
			else
			{
				positions[i] = particles->positions()[i];
			}
		}

		char basename[256];
		snprintf(basename, sizeof(basename), "frame_%06d.pos", frame.index);
		std::string filename = "E:/YEAR 3 Uni work/CTP-SPH-Erosion/Assets/Positions/DamBreak" + std::to_string(frame.index) + ".txt";
		std::ofstream file;
		file.open(filename);
		if (file) 
		{
			printf("Writing %s...\n", filename.c_str());

			for (size_t i = 0; i < positions.size(); i++)
			{
				std::string buffer = std::to_string(positions[i].x) + "," + std::to_string(positions[i].y) + "," + std::to_string(positions[i].z) + ",";
				file << buffer;
			}
			file.close();
		}
	}
}

void damBreakSim(double targetSpacing,
	int numberOfFrames, double fps)
{
	float x_size = 100;
	float z_size = 100;
	BoundingBox domain(Vector3(0,0,0), Vector3(1.5, 2, 1.5));
	double lz = domain.depth();

	// Build solver
	auto solver = PciSphSystemSolver::Builder()
		.withTargetDensity(1000)
		.withTargetSpacing(targetSpacing)
		.makeShared();

	solver->setPseudoViscosityCoefficient(0.0);
	solver->setTimeStepLimitScale(10.0);

	// Build emitter
	BoundingBox sourceBound(domain);
	sourceBound.expand(-targetSpacing);

	auto box1 =
		Box::builder()
		.withLowerCorner({ x_size/2-10, (z_size)*0.5, z_size/2 -10  })
		.withUpperCorner({ x_size / 2+10, (z_size)*0.5, z_size / 2 +10  })
		.makeShared();

	//auto box2 =
	//	Box3::builder()
	//	.withLowerCorner({ 2.5 - 0.001, 0, 0.25 * lz - 0.001 })
	//	.withUpperCorner({ 3.5 + 0.001, 0.75 + 0.001, 1.5 * lz + 0.001 })
	//	.makeShared();

	auto emitter = VolumeParticleEmitter::builder()
		.withSurface(box1)
		.withMaxRegion(box1->boundingBox())
		.withSpacing(targetSpacing)
		.withMaxNumberOfParticles(5000)
		.makeShared();

	solver->setEmitter(emitter);

	// Build collider
	//auto box = Box::builder()
	//	.withIsNormalFlipped(true)
	//	.withBoundingBox(domain)
	//	.makeShared();

	//auto collider =
	//	RigidBodyCollider::builder()
	//	.withSurface(box)
	//	.makeShared();

	//solver->setCollider(collider);

	std::vector<Vector3> vertices((x_size)*(z_size));
	std::vector<Vector3> tris((x_size - 1) * (z_size - 1) * 2);

	generateInitialVertices(&vertices, x_size, z_size);
	generateInitialTris(&tris, x_size, z_size);

	float* f_noise_seed = new float[(x_size) * (z_size)];

	std::mt19937_64 rng;
	uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
	std::seed_seq ss{ uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32) };
	rng.seed(ss);
	std::uniform_real_distribution<double> unif(0, 1);

	for (int i = 0; i < (x_size) * (z_size); i++)
	{
		f_noise_seed[i] = unif(rng);
	}

	perlinNoise(x_size, z_size, 5, 1.6f, &vertices, f_noise_seed);

	/*auto triMesh = TriangleMesh::builder()
		.withPoints(vertices)
		.withPointIndices(tris)
		.makeShared();*/

	auto heightfield =
		Heightfield::builder()
		.withPoints(vertices)
		.withResolution(x_size, z_size)
		.makeShared();

	auto collider =
		RigidBodyCollider::builder()
		.withSurface(heightfield)
		.makeShared();

	solver->setCollider(collider);

	char basename[256];
	std::string filename = "E:/YEAR 3 Uni work/CTP-SPH-Erosion/Assets/Positions/Mesh.txt";
	std::ofstream file;
	file.open(filename);
	if (file)
	{
		printf("Writing %s...\n", filename.c_str());

		file << vertices.size() << "," << tris.size() << ",";
		for (size_t i = 0; i < vertices.size(); i++)
		{
			std::string buffer = std::to_string(vertices[i].x) + "," + std::to_string(vertices[i].y) + "," + std::to_string(vertices[i].z) + ",";
			file << buffer;
		}
		for (size_t i = 0; i < tris.size(); i++)
		{
			std::string buffer = std::to_string((int)tris[i].x) + "," + std::to_string((int)tris[i].y) + "," + std::to_string((int)tris[i].z) + ",";
			file << buffer;
		}
		file.close();
	}

	// Run simulation
	runSimulation(solver, numberOfFrames, fps);
}

int main()
{
	damBreakSim(0.1, 250, 60);
}

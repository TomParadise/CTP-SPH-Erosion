// SPH simulation.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

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
	BoundingBox domain(Vector3(0,0,0), Vector3(0.25, 0.5, 0.3));
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
		.withLowerCorner({ 0, 0, 0 })
		.withUpperCorner({ 0.15 + 0.001, 0.25 + 0.001, 0.5 * lz + 0.001 })
		.makeShared();

	//auto box2 =
	//	Box3::builder()
	//	.withLowerCorner({ 2.5 - 0.001, 0, 0.25 * lz - 0.001 })
	//	.withUpperCorner({ 3.5 + 0.001, 0.75 + 0.001, 1.5 * lz + 0.001 })
	//	.makeShared();

	auto emitter = VolumeParticleEmitter::builder()
		.withSurface(box1)
		.withMaxRegion(sourceBound)
		.withSpacing(targetSpacing)
		.withMaxNumberOfParticles(400)
		.makeShared();

	solver->setEmitter(emitter);

	// Build collider
	auto box = Box::builder()
		.withIsNormalFlipped(true)
		.withBoundingBox(domain)
		.makeShared();

	auto collider =
		RigidBodyCollider::builder()
		.withSurface(box)
		.makeShared();

	solver->setCollider(collider);

	// Run simulation
	runSimulation(solver, numberOfFrames, fps);
}

int main()
{
	int in;
	std::cin >> in;
	if (in == 1)
	{
		damBreakSim(0.02,100,60);
	}	
}
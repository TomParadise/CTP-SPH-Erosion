#pragma once
#ifndef INCLUDE_TRIANGLE_MESH_H_
#define INCLUDE_TRIANGLE_MESH_H_

#include <vector>

#include "Quaternion.h"
#include "Surface.h"
#include "Triangle.h"
#include "Bvh.h"

#include <iostream>

	//!
	//! \brief 3-D triangle mesh geometry.
	//!
	//! This class represents 3-D triangle mesh geometry which extends Surface by
	//! overriding surface-related queries. The mesh structure stores point +
	//! normals
	//!
	class TriangleMesh final : public Surface
	{
	public:
		class Builder;

		typedef std::vector<Vector3> Vector3DArray;
		typedef std::vector<Vector3> IndexArray;

		typedef Vector3DArray PointArray;
		typedef Vector3DArray NormalArray;

		//! Default constructor.
		TriangleMesh(const Transform& transform = Transform(),
			bool isNormalFlipped = false);

		//! Constructs mesh with points, normals, uvs, and their indices.
		TriangleMesh(const PointArray& points, const NormalArray& normals, const IndexArray& pointIndices,
			const IndexArray& normalIndices, const Transform& transform_ = Transform(),
			bool isNormalFlipped = false);

		//! Returns true if the mesh has normals.
		bool hasNormals() const;

		//! Updates internal spatial query engine.
		void updateQueryEngine() override;

		//! Returns number of triangles.
		size_t numberOfTriangles() const;

		//! Returns i-th triangle.
		Triangle triangle(size_t i) const;

		////! Writes the mesh in obj format to the output stream.
		//void writeObj(std::ostream* strm) const;

		////! Writes the mesh in obj format to the file.
		//bool writeObj(const std::string& filename) const;

		//! Returns builder fox TriangleMesh3.
		static Builder builder();

	protected:
		Vector3 closestPointLocal(Vector3 otherPoint) const override;

		double closestDistanceLocal(Vector3 otherPoint) override;

		BoundingBox boundingBoxLocal() const override;

		Vector3 closestNormalLocal(const Vector3& otherPoint) const override;

		bool isInsideLocal(Vector3 otherPoint) override;

	private:
		PointArray _points;
		NormalArray _normals;
		IndexArray _pointIndices;
		IndexArray _normalIndices;
		IndexArray _uvIndices;

		mutable Bvh _bvh;
		mutable bool _bvhInvalidated = true;

		mutable std::vector<Vector3> _wnAreaWeightedNormalSums;
		mutable std::vector<Vector3> _wnAreaWeightedAvgPositions;
		mutable bool _wnInvalidated = true;

		void buildBvh() const;

		void buildWindingNumbers() const;

		double windingNumber(const Vector3& queryPoint, size_t triIndex) const;

		double fastWindingNumber(const Vector3& queryPoint, double accuracy) const;

		double fastWindingNumber(const Vector3& queryPoint, size_t rootNodeIndex,
			double accuracy) const;
	};

	//! Shared pointer for the TriangleMesh3 type.
	typedef std::shared_ptr<TriangleMesh> TriangleMeshPtr;


	class TriangleMesh::Builder 
	{
	 public:
		  //! Returns builder with flipped normal flag.
		 Builder& withIsNormalFlipped(bool isNormalFlipped);

		 //! Returns builder with translation.
		 Builder& withTranslation(const Vector3& translation);
	
		 //! Returns builder with orientation.
		 Builder& withOrientation(const Quaternion& orientation);

		 //! Returns builder with transform.
		 Builder& withTransform(const Transform& transform);

		 //! Returns builder with points.
		 Builder& withPoints(const PointArray& points);

		 //! Returns builder with normals.
		 Builder& withNormals(const NormalArray& normals);

		 //! Returns builder with point indices.
		 Builder& withPointIndices(const IndexArray& pointIndices);

		 //! Returns builder with normal indices.
		 Builder& withNormalIndices(const IndexArray& normalIndices);

		 //! Builds TriangleMesh3.
		 TriangleMesh build() const;

		 //! Builds shared pointer of TriangleMesh3 instance.
		 TriangleMeshPtr makeShared() const;

	  private:
		 bool _isNormalFlipped = false;
		 Transform _transform;
		 PointArray _points;
		 NormalArray _normals;
		 IndexArray _pointIndices;
		 IndexArray _normalIndices;
	};


#endif
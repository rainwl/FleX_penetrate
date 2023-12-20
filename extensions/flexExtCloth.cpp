// This code contains NVIDIA Confidential Information and is disclosed to you
// under a form of NVIDIA software license agreement provided separately to you.
//
// Notice
// NVIDIA Corporation and its licensors retain all intellectual property and
// proprietary rights in and to this software and related documentation and
// any modifications thereto. Any use, reproduction, disclosure, or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA Corporation is strictly prohibited.
//
// ALL NVIDIA DESIGN SPECIFICATIONS, CODE ARE PROVIDED "AS IS.". NVIDIA MAKES
// NO WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ALL IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Information and code furnished is believed to be accurate and reliable.
// However, NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2013-2020 NVIDIA Corporation. All rights reserved.

#include "../include/NvFlexExt.h"

#include "../core/cloth.h"

namespace
{
	struct Key
	{
		Key(int i, float d) : index(i), depth(d) {}

		int index;
		float depth;
		
		bool operator < (const Key& rhs) const { return depth < rhs.depth; }
	};
}

int NvFlexExtCreateWeldedMeshIndices(const float* vertices, int numVertices, int* uniqueIndices, int* originalToUniqueMap, float threshold)
{
	memset(originalToUniqueMap, -1, numVertices*sizeof(int));

	const Vec3* positions = (const Vec3*)vertices;

	// use a sweep and prune style search to accelerate neighbor finding
	std::vector<Key> keys;
	for (int i=0; i < numVertices; i++)
		keys.push_back(Key(i, positions[i].z));

	std::sort(keys.begin(), keys.end());

	int uniqueCount = 0;

	// sweep keys to find matching verts
	for (int i=0; i < numVertices; ++i)
	{
		// we are a duplicate, skip
		if (originalToUniqueMap[keys[i].index] != -1)
			continue;

		// scan forward until no vertex can be closer than threshold
		for (int j=i+1; j < numVertices && (keys[j].depth-keys[i].depth) <= threshold; ++j)
		{
			float distance = Length(Vector3(positions[keys[i].index])-Vector3(positions[keys[j].index]));

			if (distance <= threshold)
				originalToUniqueMap[keys[j].index] = uniqueCount;
		}

		originalToUniqueMap[keys[i].index] = uniqueCount;

		uniqueIndices[uniqueCount++] = keys[i].index;
	}

	return uniqueCount;
}

NvFlexExtAsset* NvFlexExtCreateClothFromMesh(const float* particles, int numVertices, const int* indices, int numTriangles, float stretchStiffness, float bendStiffness, float tetherStiffness, float tetherGive, float pressure)
{
	NvFlexExtAsset* asset = new NvFlexExtAsset();
	memset(asset, 0, sizeof(*asset));

	asset->particles = new float[numVertices*4];
	memcpy(asset->particles, particles, numVertices*sizeof(float)*4);	

	asset->triangleIndices = new int[numTriangles*3];
	memcpy(asset->triangleIndices, indices, numTriangles*3*sizeof(int));

	asset->numParticles = numVertices;
	asset->maxParticles = numVertices;

	asset->numTriangles = numTriangles;

	// create cloth mesh
	ClothMesh cloth((Vec4*)particles, numVertices, indices, numTriangles*3, stretchStiffness, bendStiffness, true);

	if (cloth.mValid)
	{
		// create tethers
		if (tetherStiffness > 0.0f)
		{
			std::vector<int> anchors;
			anchors.reserve(numVertices);

			// find anchors
			for (int i=0; i < numVertices; ++i)
			{
				Vec4& particle = ((Vec4*)particles)[i];

				if (particle.w == 0.0f)
					anchors.push_back(i);
			}

			if (anchors.size())
			{
				// create tethers
				for (int i=0; i < numVertices; ++i)
				{
					Vec4& particle = ((Vec4*)particles)[i];
					if (particle.w == 0.0f)
						continue;

					float minSqrDist = FLT_MAX;
					int minIndex = -1;

					// find the closest attachment point
					for (int a=0; a < int(anchors.size()); ++a)
					{
						Vec4& attachment = ((Vec4*)particles)[anchors[a]];

						float distSqr = LengthSq(Vec3(particle)-Vec3(attachment));
						if (distSqr < minSqrDist)
						{
							minSqrDist = distSqr;
							minIndex = anchors[a];
						}
					}

					// add a tether
					if (minIndex != -1)
					{						
						cloth.mConstraintIndices.push_back(i);
						cloth.mConstraintIndices.push_back(minIndex);
						cloth.mConstraintRestLengths.push_back(sqrtf(minSqrDist)*(1.0f + tetherGive));						
						
						// negative stiffness indicates tether (unilateral constraint)
						cloth.mConstraintCoefficients.push_back(-tetherStiffness);
					}
				}
			}
		}

		const int numSprings = int(cloth.mConstraintCoefficients.size());

		asset->springIndices = new int[numSprings*2];
		asset->springCoefficients = new float[numSprings];
		asset->springRestLengths = new float[numSprings];
		asset->numSprings = numSprings;

		for (int i=0; i < numSprings; ++i)
		{
			asset->springIndices[i*2+0] = cloth.mConstraintIndices[i*2+0];
			asset->springIndices[i*2+1] = cloth.mConstraintIndices[i*2+1];			
			asset->springRestLengths[i] = cloth.mConstraintRestLengths[i];
			asset->springCoefficients[i] = cloth.mConstraintCoefficients[i];
		}

		if (pressure > 0.0f)
		{
			asset->inflatable = true;
			asset->inflatableVolume = cloth.mRestVolume;
			asset->inflatableStiffness = cloth.mConstraintScale;
			asset->inflatablePressure = pressure;
		}
	}
	else
	{
		NvFlexExtDestroyAsset(asset);
		return NULL;
	}

	return asset;
}

struct FlexExtTearingClothAsset : public NvFlexExtAsset
{
	ClothMesh* mMesh;
};

//粒子数据、粒子数量、最大粒子数量、顶点索引、三角形数量以及布料的伸展刚性、弯曲刚性和压力
NvFlexExtAsset* NvFlexExtCreateTearingClothFromMesh(const float* particles, int numParticles, int maxParticles, const int* indices, int numTriangles, float stretchStiffness, float bendStiffness, float pressure)
{
	//创建一个新的 FlexExtTearingClothAsset 对象，并将其内存初始化为0
	FlexExtTearingClothAsset* asset = new FlexExtTearingClothAsset();
	memset(asset, 0, sizeof(*asset));

	//	为粒子数组分配内存，每个粒子由四个浮点数表示（通常是位置和质量）。
	//将传入的粒子数据复制到新分配的数组中
	asset->particles = new float[maxParticles*4];
	memcpy(asset->particles, particles, numParticles*sizeof(float)*4);	

	//为三角形的索引数组分配内存，每个三角形由三个索引组成
	//将传入的三角形索引数据复制到新分配的数组中
	asset->triangleIndices = new int[numTriangles*3];
	memcpy(asset->triangleIndices, indices, numTriangles*3*sizeof(int));

	//设置资产的当前粒子数和最大粒子数字段
	asset->numParticles = numParticles;
	asset->maxParticles = maxParticles;

	//设置资产的三角形数量字段
	asset->numTriangles = numTriangles;

	//创建一个新的 ClothMesh 对象，用于表示布料网格。传入粒子数据、粒子数量、三角形索引、伸展刚性和弯曲刚性。最后一个参数 true 可能指创建一个可以撕裂的布料网格
	// create and store cloth mesh
	asset->mMesh = new ClothMesh((Vec4*)particles, numParticles, indices, numTriangles*3, stretchStiffness, bendStiffness, true);

	//创建一个 ClothMesh 类型的引用，指向新创建的布料网格对象
	ClothMesh& cloth = *asset->mMesh;

	if (cloth.mValid)
	{
		//检查 ClothMesh 对象是否有效。如果有效，继续初始化弹簧（或约束）相关的数据
		const int numSprings = int(cloth.mConstraintCoefficients.size());

		//设置资产的弹簧索引、弹簧系数、弹簧静止长度和弹簧数量，这些数据直接引用 ClothMesh 中的对应数据
		// asset references cloth mesh memory directly
		asset->springIndices = &cloth.mConstraintIndices[0];
		asset->springCoefficients = &cloth.mConstraintCoefficients[0];
		asset->springRestLengths = &cloth.mConstraintRestLengths[0];
		asset->numSprings = numSprings;

		//如果指定了正的压力值，则设置资产为可充气的，并初始化相关的压力和体积参数
		if (pressure > 0.0f)
		{
			asset->inflatable = true;
			asset->inflatableVolume = cloth.mRestVolume;
			asset->inflatableStiffness = cloth.mConstraintScale;
			asset->inflatablePressure = pressure;
		}
	}
	else
	{
		NvFlexExtDestroyAsset(asset);
		return NULL;
	}

	return asset;
//这段代码的目的是在物理模拟框架（FleX）中创建一个可撕裂的布料网格资产，该资产可以进一步用于物理模拟中。通过提供粒子位置、
//连接信息和物理属性（如刚性和压力），可以模拟布料的行为，并允许它在给定的压力下进行撕裂和膨胀
}

void NvFlexExtDestroyTearingCloth(NvFlexExtAsset* asset)
{
	FlexExtTearingClothAsset* tearable = (FlexExtTearingClothAsset*)asset;

	delete[] asset->particles;
	delete[] asset->triangleIndices;

	delete tearable->mMesh;
	delete tearable;
}

//这个函数接收一个布料资产(asset)，最大应变值，最大分裂数，粒子拷贝数组，粒子拷贝数量，最大拷贝数，三角形编辑数组，三角形编辑数量，以及最大编辑数
void NvFlexExtTearClothMesh(NvFlexExtAsset* asset, float maxStrain, int maxSplits, NvFlexExtTearingParticleClone* particleCopies, int* numParticleCopies,  int maxCopies, NvFlexExtTearingMeshEdit* triangleEdits, int* numTriangleEdits, int maxEdits) 
{
	//将传入的 NvFlexExtAsset 类型的指针强制转换为 FlexExtTearingClothAsset 类型的指针，以便访问特定于可撕裂布料的属性和方法
	FlexExtTearingClothAsset* tearable = (FlexExtTearingClothAsset*)asset;

	//声明两个 std::vector 容器，分别用于存储三角形更新信息和顶点拷贝信息
	std::vector<ClothMesh::TriangleUpdate> edits;
	std::vector<ClothMesh::VertexCopy> copies;

	//初始化一个计数器用来追踪执行的撕裂操作数量
	int splits = 0;

	//计算并更新最大拷贝数，确保它不会超过布料资产的最大粒子数减去当前粒子数
	maxCopies = Min(maxCopies, tearable->maxParticles-tearable->numParticles);

	//遍历所有弹簧（布料模型中粒子间的连接），如果应变超过最大阈值，则执行撕裂。循环也受到最大拷贝数和最大撕裂次数的限制
	// iterate over all edges and tear if beyond maximum strain
	for (int i=0; i < tearable->numSprings && int(copies.size()) < maxCopies && splits < maxSplits; ++i)
	{
		//获取当前弹簧连接的两个粒子的索引
		int a = tearable->springIndices[i*2+0];
		int b = tearable->springIndices[i*2+1];

		//从粒子数组中获取这两个粒子的位置
		Vec3 p = Vec3(&tearable->particles[a*4]);
		Vec3 q = Vec3(&tearable->particles[b*4]);

		//计算当前弹簧的实际长度与其静止长度的乘以最大应变的比值，如果这个值大于1，意味着应变超出了允许的范围，需要执行撕裂
		// check strain and break if greater than max threshold
		if (Length(p-q) > tearable->springRestLengths[i]*maxStrain)
		{
			//检查两个粒子是否是固定的（通过质量的倒数来表示，即w分量），如果是，则跳过撕裂过程
			// skip fixed particles
			if (Vec4(&tearable->particles[a*4]).w == 0.0f)
				continue;

			if (Vec4(&tearable->particles[b*4]).w == 0.0f)
				continue;

			//随机选择边的一个顶点进行分裂，并定义一个分裂平面
			// choose vertex of edge to split
			const int splitIndex = Randf() > 0.5f ? a : b;
			const Vec3 splitPlane = Normalize(p-q);	// todo: use plane perpendicular to normal and edge..

			//声明两个向量，用来存储与分裂顶点相关的三角形和顶点信息
			std::vector<int> adjacentTriangles;
			std::vector<int> adjacentVertices;

			//调用 SplitVertex 方法来分裂顶点，这个函数会返回新顶点的索引。如果返回 -1，则表示分裂失败
			const int newIndex = tearable->mMesh->SplitVertex((Vec4*)tearable->particles, splitIndex, splitPlane, adjacentTriangles, adjacentVertices, edits, copies, maxCopies-int(copies.size()));

			//如果分裂成功，增加撕裂操作的计数
			if (newIndex != -1)
			{
				++splits;

				//遍历所有相邻的顶点，并将它们从共享的顶点中分离出来，如果它们现在是奇异的（每个顶点现在属于不同的粒子）
				// separate each adjacent vertex if it is now singular
				for (int s=0; s < int(adjacentVertices.size()); ++s)
				{
					const int adjacentVertex = adjacentVertices[s];

					//处理新顶点是否奇异的情况
					tearable->mMesh->SeparateVertex(adjacentVertex, edits, copies, maxCopies-int(copies.size()));
				}

				//更新资产的粒子数，以反映任何新的分裂和撕裂效果
				// also test the new vertex which can become singular
				tearable->mMesh->SeparateVertex(newIndex, edits, copies, maxCopies-int(copies.size()));
			}
		}
	}

	//将顶点拷贝信息从局部向量 copies 复制到输出参数 particleCopies
	// update asset particle count
	tearable->numParticles = tearable->mMesh->mNumVertices;

	// output copies
	for (int c=0; c < int(copies.size()); ++c)
	{
		NvFlexExtTearingParticleClone clone;
		clone.srcIndex = copies[c].srcIndex;
		clone.destIndex = copies[c].destIndex;

		particleCopies[c] = clone;
	}

	//计算要输出的三角形编辑数量，并确保不超过最大编辑数
	// output mesh edits, note that some edits will not be reported if edit buffer is not big enough
	const int numEdits = Min(int(edits.size()), maxEdits);

	//将三角形编辑信息从局部向量 edits 复制到输出参数 triangleEdits，并更新织物资产的三角形索引
	for (int u=0; u < numEdits; ++u)
	{
		NvFlexExtTearingMeshEdit update;
		update.triIndex = edits[u].triangle;
		update.newParticleIndex = edits[u].vertex;

		tearable->triangleIndices[update.triIndex] = update.newParticleIndex;

		triangleEdits[u] = update;
	}

	//设置输出参数 numTriangleEdits 和 numParticleCopies 的值，表示实际编辑的三角形数量和拷贝的顶点数量
	*numTriangleEdits = numEdits;
	*numParticleCopies = int(copies.size());
}

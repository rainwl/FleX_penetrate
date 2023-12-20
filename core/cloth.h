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

#pragma once

#include <set>
#include <vector>
#include <map>
#include <algorithm>
#include <functional>
#include <numeric>

#include "maths.h"

class ClothMesh
{
public:

	struct Edge
	{
		int vertices[2];
		int tris[2];
		
		int stretchConstraint;
		int bendingConstraint;


		Edge(int a, int b)
		{
			assert(a != b);

			vertices[0] = Min(a, b);
			vertices[1] = Max(a, b);

			tris[0] = -1;
			tris[1] = -1;
			
			stretchConstraint = -1;
			bendingConstraint = -1;
		}

		bool IsBoundary() const { return tris[0] == -1 || tris[1] == -1; }

		bool Contains(int index) const
		{
			return (vertices[0] == index) || (vertices[1] == index);
		}

		void Replace(int oldIndex, int newIndex)
		{
			if (vertices[0] == oldIndex)
				vertices[0] = newIndex;
			else if (vertices[1] == oldIndex)
				vertices[1] = newIndex;
			else
				assert(0);
		}

		void RemoveTri(int index)
		{
			if (tris[0] == index)
				tris[0] = -1;
			else if (tris[1] == index)
				tris[1] = -1;
			else
				assert(0);
		}

		bool AddTri(int index)
		{
			if (tris[0] == -1)
			{
				tris[0] = index;
				return true;
			}
			else if (tris[1] == -1)
			{
				// check tri not referencing same edge
				if (index == tris[0])
					return false;		
				else
				{
					tris[1] = index;
					return true;
				}
			}
			else
				return false;
		}

		bool operator < (const Edge& rhs) const
		{
			if (vertices[0] != rhs.vertices[0])
				return vertices[0] < rhs.vertices[0];
			else
				return vertices[1] < rhs.vertices[1];
		}
	};

	struct Triangle
	{
		Triangle(int a, int b, int c)
		{
			assert(a != b && a != c);
			assert(b != c);

			vertices[0] = a;
			vertices[1] = b;
			vertices[2] = c;

			edges[0] = -1;
			edges[1] = -1;
			edges[2] = -1;

			side = -1;

			component = -1;
		}

		bool Contains(int v) const
		{
			if (vertices[0] == v ||
				vertices[1] == v ||
				vertices[2] == v)
				return true;
			else
				return false;
		}

		void ReplaceEdge(int oldIndex, int newIndex)
		{
			for (int i=0; i < 3; ++i)
			{
				if (edges[i] == oldIndex)
				{
					edges[i] = newIndex;
					return;
				}

			}
			assert(0);
		}

		int ReplaceVertex(int oldIndex, int newIndex)
		{
			for (int i=0; i < 3; ++i)
			{
				if (vertices[i] == oldIndex)
				{
					vertices[i] = newIndex;
					return i;
				}
			}

			assert(0);
			return -1;
		}

		int GetOppositeVertex(int v0, int v1) const
		{
			for (int i=0; i < 3; ++i)
			{
				if (vertices[i] != v0 && vertices[i] != v1)
					return vertices[i];
			}

			assert(0);
			return -1;
		}

		int vertices[3];
		int edges[3];

		// used during splitting
		int side;

		// used during singular vertex removal
		mutable int component;
	};

	ClothMesh(const Vec4* vertices, int numVertices, 
			  const int* indices, int numIndices,
			  float stretchStiffness,
			  float bendStiffness, bool tearable=true)
	{
		mValid = false;

		mNumVertices = numVertices;

		if (tearable)
		{
			// tearable cloth uses a simple bending constraint model that allows easy splitting of vertices and remapping of constraints
			typedef std::set<Edge> EdgeSet;
			EdgeSet edges;

			// build unique edge list
			for (int i=0; i < numIndices; i += 3)
			{
				mTris.push_back(Triangle(indices[i+0], indices[i+1], indices[i+2]));

				const int triIndex = i/3;

				// breaking the rules
				Edge& e1 = const_cast<Edge&>(*edges.insert(Edge(indices[i+0], indices[i+1])).first);
				Edge& e2 = const_cast<Edge&>(*edges.insert(Edge(indices[i+1], indices[i+2])).first);
				Edge& e3 = const_cast<Edge&>(*edges.insert(Edge(indices[i+2], indices[i+0])).first);

				if (!e1.AddTri(triIndex))
					return;
				if (!e2.AddTri(triIndex))
					return;
				if (!e3.AddTri(triIndex))
					return;
			}

			// flatten set to array
			mEdges.assign(edges.begin(), edges.end());

			// second pass, set edge indices to faces
			for (int i=0; i < numIndices; i += 3)
			{
				int e1 = int(std::lower_bound(mEdges.begin(), mEdges.end(), Edge(indices[i+0], indices[i+1])) - mEdges.begin());
				int e2 = int(std::lower_bound(mEdges.begin(), mEdges.end(), Edge(indices[i+1], indices[i+2])) - mEdges.begin());
				int e3 = int(std::lower_bound(mEdges.begin(), mEdges.end(), Edge(indices[i+2], indices[i+0])) - mEdges.begin());


				if (e1 != e2 && e1 != e3 && e2 != e3)
				{
					const int triIndex = i/3;

					mTris[triIndex].edges[0] = e1;
					mTris[triIndex].edges[1] = e2;
					mTris[triIndex].edges[2] = e3;
				}
				else
				{
					// degenerate tri
					return;
				}
			}

			// generate distance constraints
			for (size_t i=0; i < mEdges.size(); ++i)
			{
				Edge& edge = mEdges[i];

				// stretch constraint along mesh edges
				edge.stretchConstraint = AddConstraint(vertices, edge.vertices[0], edge.vertices[1], stretchStiffness);

				const int t1 = edge.tris[0];
				const int t2 = edge.tris[1];

				// add bending constraint between connected tris
				if (t1 != -1 && t2 != -1 && bendStiffness > 0.0f)
				{
					int v1 = mTris[t1].GetOppositeVertex(edge.vertices[0], edge.vertices[1]);
					int v2 = mTris[t2].GetOppositeVertex(edge.vertices[0], edge.vertices[1]);
					edge.bendingConstraint = AddConstraint(vertices, v1, v2, bendStiffness);
				}
			}
		}

		// calculate rest volume
		mRestVolume = 0.0f;
		mConstraintScale = 0.0f;

		std::vector<Vec3> gradients(numVertices);

		for (int i=0; i < numIndices; i+=3)
		{
			Vec3 v1 = Vec3(vertices[indices[i+0]]);
			Vec3 v2 = Vec3(vertices[indices[i+1]]);
			Vec3 v3 = Vec3(vertices[indices[i+2]]);

			const Vec3 n = Cross(v2-v1, v3-v1);
			const float signedVolume = Dot(v1, n);

			mRestVolume += signedVolume;

			gradients[indices[i+0]] += n;
			gradients[indices[i+1]] += n;
			gradients[indices[i+2]] += n;
		}
		
		for (int i=0; i < numVertices; ++i)
			mConstraintScale += Dot(gradients[i], gradients[i]);

		mConstraintScale = 1.0f/mConstraintScale;

		mValid = true;

	}

	int AddConstraint(const Vec4* vertices, int a, int b, float stiffness, float give=0.0f)
	{
		int index = int(mConstraintRestLengths.size());

		mConstraintIndices.push_back(a);
		mConstraintIndices.push_back(b);

		const float restLength = Length(Vec3(vertices[a])-Vec3(vertices[b]));
			
		mConstraintRestLengths.push_back(restLength*(1.0f + give));
		mConstraintCoefficients.push_back(stiffness);

		return index;
	}

	int IsSingularVertex(int vertex) const
	{
		std::vector<int> adjacentTriangles;

		// gather adjacent triangles
		for (int i=0; i < int(mTris.size()); ++i)
		{
			if (mTris[i].Contains(vertex))
				adjacentTriangles.push_back(i);
		}

		// number of identified components
		int componentCount = 0;

		// while connected tris not colored
		for (int i=0; i < int(adjacentTriangles.size()); ++i)
		{
			// pop off a triangle
			int seed = adjacentTriangles[i];
			
			// triangle already belongs to a component
			if (mTris[seed].component != -1)
				continue;

			std::vector<int> stack;
			stack.push_back(seed);

			while (!stack.empty())
			{
				int t = stack.back();
				stack.pop_back();

				const Triangle& tri = mTris[t];

				if (tri.component == componentCount)				
				{
					// we're back to the beginning
					// component is fully connected
					break;
				}

				tri.component = componentCount;

				// update mesh
				for (int e=0; e < 3; ++e)
				{
					const Edge& edge = mEdges[tri.edges[e]];

					if (edge.Contains(vertex))
					{
						if (!edge.IsBoundary())
						{
							// push unprocessed neighbors on stack
							for (int s=0; s < 2; ++s)
							{
								assert(mTris[edge.tris[s]].component == -1 || mTris[edge.tris[s]].component == componentCount);

								if (edge.tris[s] != t && mTris[edge.tris[s]].component == -1)
									stack.push_back(edge.tris[s]);
							}
						}
					}
				}	
			}

			componentCount++;
		}

		// reset component indices
		for (int i=0; i < int(adjacentTriangles.size()); ++i)
		{
			assert(mTris[adjacentTriangles[i]].component != -1);

			mTris[adjacentTriangles[i]].component = -1;
		} 

		return componentCount;
	}

	struct TriangleUpdate
	{
		int triangle;
		int vertex;
	};

	struct VertexCopy
	{
		int srcIndex;
		int destIndex;
	};

	//它的目的是将一个网格中多个相连的三角形组件从一个共享的顶点中分离出来，同时确保网格的拓扑结构被适当地更新
	int SeparateVertex(int singularVertex, std::vector<TriangleUpdate>& replacements, std::vector<VertexCopy>& copies, int maxCopies)
	{
		//它接受一个整型参数singularVertex（表示要处理的单一顶点的索引），两个向量的引用参数replacements和copies（用于存储替换和复制操作的结果），以及一个整型参数maxCopies（用于限制可以创建的顶点复制的最大数量）

		//声明一个整型向量adjacentTriangles，用于存储所有与singularVertex相邻的三角形的索引
		std::vector<int> adjacentTriangles;

		//遍历mTris（一个包含网格所有三角形的向量），
		//并找出所有包含singularVertex顶点的三角形。
		//这些三角形的索引被添加到adjacentTriangles向量中
		// gather adjacent triangles
		for (int i=0; i < int(mTris.size()); ++i)
		{
			if (mTris[i].Contains(singularVertex))
				adjacentTriangles.push_back(i);
		}

		//初始化一个整型变量componentCount，用于记录已经识别的连通组件的数量
		// number of identified components
		int componentCount = 0;

		//初始化一个整型变量newIndex，用于存储新顶点的索引。第一个连通组件将保留现有的singularVertex
		// first component keeps the existing vertex
		int newIndex = singularVertex;

		//遍历所有相邻的三角形。如果已达到maxCopies的限制，则退出循环
		// while connected tris not colored
		for (int i=0; i < int(adjacentTriangles.size()); ++i)
		{
			if (maxCopies == 0)
				break;

			//从adjacentTriangles中取出一个三角形的索引，称为seed
			// pop off a triangle
			int seed = adjacentTriangles[i];

			//如果这个三角形已经被分配到了一个组件中，则跳过它
			// triangle already belongs to a component
			if (mTris[seed].component != -1)
				continue;

			//创建一个栈stack，并将seed三角形的索引压入栈中
			std::vector<int> stack;
			stack.push_back(seed);

			//当栈不为空时，循环执行。从栈中取出一个三角形的索引t
			while (!stack.empty())
			{
				int t = stack.back();
				stack.pop_back();

				//获取索引t对应的三角形对象tri的引用
				Triangle& tri = mTris[t];

				//如果三角形tri已经属于当前组件（componentCount），则这表示我们完成了对当前连通组件的遍历
				// test if we're back to the beginning, in which case the component is fully connected
				if (tri.component == componentCount)									
					break;

				//断言确保当前三角形tri尚未分配到任何组件中
				assert(tri.component == -1);

				//将三角形tri分配到当前的组件componentCount
				tri.component = componentCount;

				//在三角形tri中，将singularVertex替换为newIndex。此操作返回被替换的顶点在三角形中的局部索引v
				// update triangle
				int v = tri.ReplaceVertex(singularVertex, newIndex);

				//如果singularVertex和newIndex不相同，即该顶点已经被复制到了一个新的位置，则创建一个TriangleUpdate结构体r来记录这次替换。这个结构体记录了被替换的三角形索引和新的顶点索引，然后将这个结构体添加到replacements向量中
				if (singularVertex != newIndex)
				{
					// output replacement
					TriangleUpdate r;
					r.triangle = t*3 + v;
					r.vertex = newIndex;
					replacements.push_back(r);
				}

				//接下来的循环通过三角形的三条边来更新网格。获取当前三角形第e条边的引用
				// update mesh
				for (int e=0; e < 3; ++e)
				{
					Edge& edge = mEdges[tri.edges[e]];

					//如果这条边包含singularVertex，则将边中的singularVertex替换为newIndex
					if (edge.Contains(singularVertex))
					{
						// update edge to point to new vertex
						edge.Replace(singularVertex, newIndex);

						//更新边的伸展约束，确保约束索引数组中的singularVertex也被更新为newIndex。如果没有找到singularVertex，则触发一个断言错误
						const int stretching = edge.stretchConstraint;
						if (mConstraintIndices[stretching*2+0] == singularVertex)
							mConstraintIndices[stretching*2+0] = newIndex;
						else if (mConstraintIndices[stretching*2+1] == singularVertex)
							mConstraintIndices[stretching*2+1] = newIndex;
						else
							assert(0);

						//对于每条边，除了更新伸展约束之外，如果边不是边界边，则将与其相邻的未处理的三角形压入栈中以供后续处理。如果边不含singularVertex，则检查和可能更新该边的弯曲约束。
						if (!edge.IsBoundary())
						{
							// push unprocessed neighbors on stack
							for (int s=0; s < 2; ++s)
							{
								assert(mTris[edge.tris[s]].component == -1 || mTris[edge.tris[s]].component == componentCount);

								if (edge.tris[s] != t && mTris[edge.tris[s]].component == -1)
									stack.push_back(edge.tris[s]);
							}
						}
					}
					else
					{
						const int bending = edge.bendingConstraint;

						if (bending != -1)
						{
							if (mConstraintIndices[bending*2+0] == singularVertex)
								mConstraintIndices[bending*2+0] = newIndex;
							else if (mConstraintIndices[bending*2+1] == singularVertex)
								mConstraintIndices[bending*2+1] = newIndex;
						}
					}
				}	
			}

			//如果创建了一个新的顶点（singularVertex与newIndex不同），则记录这次复制操作，增加顶点计数器mNumVertices，并减少可以进行的复制操作的剩余次数maxCopies
			// copy vertex
			if (singularVertex != newIndex)
			{
				VertexCopy copy;
				copy.srcIndex = singularVertex;
				copy.destIndex = newIndex;

				copies.push_back(copy);

				mNumVertices++;
				maxCopies--;
			}

			//完成一个组件的遍历后，设置newIndex为下一个新的顶点索引，并增加组件计数器componentCount
			// component traversal finished
			newIndex = mNumVertices;

			componentCount++;
		}

		//该代码段完成了组件分离的最后步骤，它将所有相邻三角形的组件索引重置为 -1，以便它们可以在未来的操作中被重新分类。这里不再需要断言，因为该步骤是清理过程的一部分，不需要验证三角形是否已被分配组件索引
		// reset component indices
		for (int i=0; i < int(adjacentTriangles.size()); ++i)
		{
			//assert(mTris[adjacentTriangles[i]].component != -1);

			mTris[adjacentTriangles[i]].component = -1;
		} 
		//最后，该函数返回componentCount，即在该顶点处分离出的连通组件的数量。这个返回值可能对调用该函数的上层代码有用，可能用于验证操作的成功，或用于进一步的逻辑决策。

		// 总的来说，这个函数通过以下步骤来处理网格数据：
// 
		// 找出所有与特定顶点相邻的三角形。
		// 使用深度优先搜索（通过栈实现）来识别相连的三角形组件，并将新的或现有的顶点索引分配给这些组件中的三角形。
		// 更新相关的三角形、边缘以及它们的约束，以反映顶点的分离和复制。
		// 在处理过程中，记录顶点的复制和三角形顶点的更新。
		// 重置相邻三角形的组件索引以供将来使用。
		// 注意，虽然这段代码在逻辑上是连贯的，但由于它依赖于很多未显示定义的外部结构和状态（如 mTris, mEdges, Edge, Triangle，以及它们的方法如 Contains, ReplaceVertex, IsBoundary 等），因此在没有这些上下文的情况下，完整的功能和性能评估是不完整的。此外，断言（assert）语句用于调试中验证假设，如果这些断言失败，程序将在运行时被终止。在生产环境中，可能需要更稳健的错误处理机制
		
		return componentCount;
	}

	//目的是在模拟物理变形或断裂的过程中，按照一个给定的平面来分割一个网格中的顶点
	int SplitVertex(const Vec4* vertices, int index, Vec3 splitPlane, std::vector<int>& adjacentTris, std::vector<int>& adjacentVertices, std::vector<TriangleUpdate>& replacements, std::vector<VertexCopy>& copies, int maxCopies)
	{
		// vertices: 一个指向顶点数组的指针，每个顶点是一个四维向量 Vec4（可能包含了位置和其他数据，如齐次坐标）。
		// index: 要分割的顶点在 vertices 数组中的索引。
		// splitPlane: 分割平面，一个三维向量 Vec3，定义了分割面的方向。
		// adjacentTris: 一个整数向量，用于存储与分割顶点相邻的三角形索引。
		// adjacentVertices: 一个整数向量，用于存储与分割顶点相邻的顶点索引。
		// replacements: 一个 TriangleUpdate 结构体的向量，记录了需要更新的三角形信息。
		// copies: 一个 VertexCopy 结构体的向量，记录了顶点分割的信息。
		// maxCopies: 允许的最大复制顶点数量。如果为0，表示不允许复制。

		//函数开始时，先检查是否允许复制顶点
		//如果 maxCopies 等于0，函数直接返回 -1，表示没有执行任何分割操作
		if (maxCopies == 0)
			return -1;
		//接下来，函数计算分割顶点与分割平面的点乘结果，这会用于后面确定三角形相对于分割平面的位置
		float w = Dot(vertices[index], splitPlane);

		//然后，函数初始化两个计数器，用于分别记录在分割平面两侧的三角形数量
		int leftCount = 0;
		int rightCount = 0;

		//接着定义一个新的顶点索引 newIndex，这个索引将用于存储分割后新产生的顶点
		//mNumVertices 是当前网格中顶点的数量，新顶点将被添加到数组的末尾
		const int newIndex = mNumVertices;

		//函数的下一部分遍历所有三角形，检查它们是否包含需要分割的顶点
		// classify all tris attached to the split vertex according 
		// to which side of the split plane their centroid lies on O(N)
		for (size_t i = 0; i < mTris.size(); ++i)
		{
			Triangle& tri = mTris[i];

			if (tri.Contains(index))
			{
				const Vec4 centroid = (vertices[tri.vertices[0]] + vertices[tri.vertices[1]] + vertices[tri.vertices[2]]) / 3.0f;

				if (Dot(Vec3(centroid), splitPlane) < w)
				{
					tri.side = 1;

					++leftCount;
				}
				else
				{
					tri.side = 0;

					++rightCount;
				}

				adjacentTris.push_back(int(i));
				for (int v=0; v < 3; ++v)
				{
					if (std::find(adjacentVertices.begin(), adjacentVertices.end(), tri.vertices[v]) == adjacentVertices.end())
					{
						adjacentVertices.push_back(tri.vertices[v]);
					}
				}
			}
		}

		//在这个循环中，函数计算每个包含分割顶点的三角形的质心（中心点），并根据质心与分割平面的相对位置给三角形打上标记（在 tri.side 中保存）。同时，记录每个三角形的索引到 adjacentTris，并将与分割顶点相邻的其他顶点索引保存到 adjacentVertices。

		//接下来的代码段检查如果所有相邻三角形都在分割平面的同一侧，那么不执行分割操作
		// if all tris on one side of split plane then do nothing
		if (leftCount == 0 || rightCount == 0)
			return -1;

		// remap triangle indices
		for (size_t i = 0; i < adjacentTris.size(); ++i)
		{
			const int triIndex = adjacentTris[i];

			Triangle& tri = mTris[triIndex];

			// tris on the negative side of the split plane are assigned the new index
			if (tri.side == 0)
			{
				int v = tri.ReplaceVertex(index, newIndex);

				TriangleUpdate update;
				update.triangle = triIndex*3 + v;
				update.vertex = newIndex;
				replacements.push_back(update);

				// update edges and constraints
				for (int e = 0; e < 3; ++e)
				{
					Edge& edge = mEdges[tri.edges[e]];

					if (edge.Contains(index))
					{
						bool exposed = false;

						if (edge.tris[0] != -1 && edge.tris[1] != -1)
						{
							Triangle& t1 = mTris[edge.tris[0]];
							Triangle& t2 = mTris[edge.tris[1]];

							// Case 1: connected tris lie on opposite sides of the split plane
							// creating a new exposed edge, need to break bending constraint
							// and create new stretch constraint for exposed edge
							if (t1.side != t2.side)
							{
								// create new edge
								Edge newEdge(edge.vertices[0], edge.vertices[1]);
								newEdge.Replace(index, newIndex);
								newEdge.AddTri(triIndex);

								// remove neighbor from old edge
								edge.RemoveTri(triIndex);

								// replace bending constraint with stretch constraint
								assert(edge.bendingConstraint != -1);

								newEdge.stretchConstraint = edge.bendingConstraint;

								mConstraintIndices[newEdge.stretchConstraint * 2 + 0] = newEdge.vertices[0];
								mConstraintIndices[newEdge.stretchConstraint * 2 + 1] = newEdge.vertices[1];
								mConstraintCoefficients[newEdge.stretchConstraint] = mConstraintCoefficients[edge.stretchConstraint];
								mConstraintRestLengths[newEdge.stretchConstraint] = mConstraintRestLengths[edge.stretchConstraint];

								edge.bendingConstraint = -1;

								// don't access Edge& after this 
								tri.ReplaceEdge(tri.edges[e], int(mEdges.size()));
								mEdges.push_back(newEdge);

								exposed = true;
							}
						}

						if (!exposed)
						{
							// Case 2: both tris on same side of split plane or boundary edge, simply remap edge and constraint
							// may have processed this edge already so check that it still contains old vertex
							edge.Replace(index, newIndex);

							const int stretching = edge.stretchConstraint;
							if (mConstraintIndices[stretching * 2 + 0] == index)
								mConstraintIndices[stretching * 2 + 0] = newIndex;
							else if (mConstraintIndices[stretching * 2 + 1] == index)
								mConstraintIndices[stretching * 2 + 1] = newIndex;
							else
								assert(0);
						}
					}
					else
					{
						// Case 3: tri is adjacent to split vertex but this edge is not connected to it
						// therefore there can be a bending constraint crossing this edge connected 
						// to vertex that needs to be remapped
						const int bending = edge.bendingConstraint;

						if (bending != -1)
						{
							if (mConstraintIndices[bending * 2 + 0] == index)
								mConstraintIndices[bending * 2 + 0] = newIndex;
							else if (mConstraintIndices[bending * 2 + 1] == index)
								mConstraintIndices[bending * 2 + 1] = newIndex;
						}
					}
				}

			}
		}

		// output vertex copy
		VertexCopy copy;
		copy.srcIndex = index;
		copy.destIndex = newIndex;

		copies.push_back(copy);

		mNumVertices++;

		return newIndex;
		//如果确实需要分割，函数会继续对相邻的三角形进行处理，将位于分割平面一侧的三角形的顶点从原始索引 index 更新为新索引 newIndex。同时记录这些更新到 replacements 向量中。

        //在处理三角形顶点的同时，还需要更新与这些三角形相关的边缘（Edge）信息和约束（约束可能是弯曲的或伸展的）。这包括替换边缘上的旧顶点索引，以及调整相关的约束索引。如果一个边缘连接着位于分割平面两侧的两个三角形，那么它会变成一个暴露的边缘，并且需要创建新的边缘和伸展约束来替代原来的弯曲约束。
        
        //完成所有这些更新后，函数会将分割操作的结果保存到 copies 向量中，包括原始顶点和新复制顶点的索引。
        
        //最后，函数递增网格中顶点的数量 mNumVertices，并返回新顶点的索引 newIndex。
        
        //整个函数的目的是在不改变网格拓扑结构的情况下，将一个顶点分割成两个顶点，并更新网格中三角形、边缘和约束的信息，以便这些信息反映出顶点的分割。这对于实现顶点的精确控制和模拟物理变化非常重要。
	}

	std::vector<int> mConstraintIndices;
	std::vector<float> mConstraintCoefficients;
	std::vector<float> mConstraintRestLengths;

	std::vector<Edge> mEdges;
	std::vector<Triangle> mTris;
	
	int mNumVertices;

	float mRestVolume;
	float mConstraintScale;

	bool mValid;
};

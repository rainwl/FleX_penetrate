
class Tearing : public Scene
{
public:

	Tearing(const char* name) : Scene(name) {}

	~Tearing()
	{
		NvFlexExtDestroyTearingCloth(mCloth);
	}

	void Initialize()
	{
		Mesh* mesh = ImportMesh(GetFilePathByPlatform("../../data/irregular_plane.obj").c_str());
		mesh->Transform(RotationMatrix(kPi, Vec3(0.0f, 1.0f, 0.0f))*RotationMatrix(kPi*0.5f, Vec3(1.0f, 0.0f, 0.0f))*ScaleMatrix(2.0f));

		// 定义两个Vec3类型的变量，用于存储网格的边界
		Vec3 lower, upper;
		mesh->GetBounds(lower, upper); // 获取网格的边界

		float radius = 0.065f;
		// 设置粒子的相位，用于粒子之间的碰撞检测
		int phase = NvFlexMakePhase(0, eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter);

		// 遍历网格中的每个顶点
		for (size_t i = 0; i < mesh->GetNumVertices(); ++i)
		{
			// 获取顶点位置
			Vec3 p = Vec3(mesh->m_positions[i]);
			
			// 设置质量的倒数，默认为1.0f
			float invMass = 1.0f;

			// 如果顶点的y坐标等于边界的上界，则将质量的倒数设置为0（静止的点）
			if (p.y == upper.y)
				invMass = 0.0f;

			// 将顶点位置沿y轴向上移动1.5f单位
			p += Vec3(0.0f, 1.5f, 0.0f);

			// 将顶点位置和质量的倒数加入到g_buffers的positions数组中
			g_buffers->positions.push_back(Vec4(p.x, p.y, p.z, invMass));
			// 初始化速度为0
			g_buffers->velocities.push_back(0.0f);
			// 将粒子的相位加入到g_buffers的phases数组中
			g_buffers->phases.push_back(phase);
		}
		// 设置额外粒子的数量
		g_numExtraParticles = 1000;
		
		// 从网格创建可撕裂的布料，并设置相关参数
		mCloth = NvFlexExtCreateTearingClothFromMesh((float*)&g_buffers->positions[0], int(g_buffers->positions.size()), int(g_buffers->positions.size()) + g_numExtraParticles, (int*)&mesh->m_indices[0], mesh->GetNumFaces(), 0.8f, 0.8f, 0.0f);

		// 将网格的索引数据复制到g_buffers的triangles数组中, 将网格索引数据复制到粒子缓冲区的三角形列表中
		g_buffers->triangles.assign((int*)&mesh->m_indices[0], mesh->m_indices.size());

		// 初始化三角形法线数组，将所有法线初始化为朝向Z轴正方向
		g_buffers->triangleNormals.resize(mesh->GetNumFaces(), Vec3(0.0f, 0.0f, 1.0f));

		// 复制弹簧索引到g_buffers的springIndices数组中
		g_buffers->springIndices.assign(mCloth->springIndices, mCloth->numSprings * 2);

		// 复制弹簧刚度到g_buffers的springStiffness数组中
		g_buffers->springStiffness.assign(mCloth->springCoefficients, mCloth->numSprings);

		// 复制弹簧原始长度到g_buffers的springLengths数组中
		g_buffers->springLengths.assign(mCloth->springRestLengths, mCloth->numSprings);

		/// 设置模拟参数，包括粒子半径、动态摩这段代码似乎是用于初始化一个物理模拟环境，特别是用于模拟布料或柔性物体的场景。它使用了NvFlex库，这是NVIDIA提供的一种用于实时模拟柔体动力学的粒子系统库。下面是对这段代码的中文逐行分析：
		g_params.radius = radius;
		g_params.dynamicFriction = 0.025f;
		g_params.dissipation = 0.0f;
		g_params.numIterations = 16;
		g_params.particleCollisionMargin = g_params.radius*0.05f;
		g_params.relaxationFactor = 1.0f;
		g_params.drag = 0.03f;

		g_params.relaxationMode = eNvFlexRelaxationGlobal;
		g_params.relaxationFactor = 0.35f;

		g_numSubsteps = 2;

		g_pause = false;

		// draw options		
		g_drawPoints = false;
	}

	void Update()
	{
		g_params.wind[0] = 0.1f;
		g_params.wind[1] = 0.1f;
		g_params.wind[2] = -0.2f;
		g_windStrength = 6.0f;

		// 定义布料撕裂的最大应变值以及最大可以处理的拷贝和修改数量
		const float maxStrain = 3.0f;
		const int maxCopies = 2048;
		const int maxEdits = 2048;

		// 创建数组用于存储可能的粒子拷贝和布料的三角形修改
		NvFlexExtTearingParticleClone particleCopies[maxCopies];
		int numParticleCopies;

		NvFlexExtTearingMeshEdit triangleEdits[maxEdits];
		int numTriangleEdits;

		// 将模拟中粒子位置的当前状态复制到布料对象的粒子数组中
		// update asset's copy of the particles
		memcpy(mCloth->particles, &g_buffers->positions[0], sizeof(Vec4)*g_buffers->positions.size());

		// 对布料网格进行撕裂处理，基于最大应变值，处理粒子的拷贝和三角形的修改
		NvFlexExtTearClothMesh(mCloth, maxStrain, 4, particleCopies, &numParticleCopies, maxCopies, triangleEdits, &numTriangleEdits, maxEdits);

		// 遍历所有粒子拷贝，将新粒子的数据复制到粒子缓冲区中
		// copy particles
		for (int i = 0; i < numParticleCopies; ++i)
		{
			const int srcIndex = particleCopies[i].srcIndex;// 源粒子索引
			const int destIndex = particleCopies[i].destIndex;// 新粒子索引

			// 复制粒子位置，将新粒子的质量设置为1（可能表示无限大质量，用于用户交互的粒子）
			g_buffers->positions[destIndex] = Vec4(Vec3(g_buffers->positions[srcIndex]), 1.0f);	// override mass because picked particle has inf. mass

			// 复制粒子的静止位置
			g_buffers->restPositions[destIndex] = g_buffers->restPositions[srcIndex];

			// 复制粒子的速度
			g_buffers->velocities[destIndex] = g_buffers->velocities[srcIndex];

			// 复制粒子的“phase”（可能是用于处理粒子间碰撞和相互作用的属性）
			g_buffers->phases[destIndex] = g_buffers->phases[srcIndex];

			// 将新粒子索引添加到活动粒子索引列表中
			g_buffers->activeIndices.push_back(destIndex);
		}

		// 应用三角形修改到索引缓冲区，这可能影响网格拓扑结构
		// apply triangle modifications to index buffer
		for (int i = 0; i < numTriangleEdits; ++i)
		{
			const int index = triangleEdits[i].triIndex;// 需要修改的三角形索引
			const int newValue = triangleEdits[i].newParticleIndex;// 新粒子的索引

			// 更新三角形的某个顶点索引
			g_buffers->triangles[index] = newValue;
		}

		// 更新布料对象中粒子的数量
		mCloth->numParticles += numParticleCopies;

		// 更新布料对象中的弹簧约束
		// update constraints
		g_buffers->springIndices.assign(mCloth->springIndices, mCloth->numSprings * 2);
		g_buffers->springStiffness.assign(mCloth->springCoefficients, mCloth->numSprings);
		g_buffers->springLengths.assign(mCloth->springRestLengths, mCloth->numSprings);
	}

	virtual void Sync()
	{
		// update solver data not already updated in the main loop
		// 这一行调用了 NvFlex 库的 NvFlexSetSprings 函数，用于将弹簧（或者说是弹性约束）的数据从 CPU 内存同步到 NvFlex 物理求解器
		// g_buffers->springIndices.buffer: 弹簧索引的 GPU 缓冲区，它指定了每个弹簧连接的两个粒子的索引
		// g_buffers->springLengths.buffer: 弹簧的静止长度的 GPU 缓冲区，这些值用于计算弹簧力
		NvFlexSetSprings(g_solver, g_buffers->springIndices.buffer, g_buffers->springLengths.buffer, g_buffers->springStiffness.buffer, g_buffers->springLengths.size());
		// 这一行调用了 NvFlexSetDynamicTriangles 函数，用于同步动态三角形的数据到求解器
		NvFlexSetDynamicTriangles(g_solver, g_buffers->triangles.buffer, g_buffers->triangleNormals.buffer, g_buffers->triangles.size() / 3);
		NvFlexSetRestParticles(g_solver, g_buffers->restPositions.buffer, NULL);
	}

	NvFlexExtAsset* mCloth;
};
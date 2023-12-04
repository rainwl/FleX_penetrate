
#include "softbody.h"

class PotPourri : public Scene
{
public:

	PotPourri(const char* name) : Scene(name)
	{
		SoftBody::Instance instance("../../data/bunny.ply");
		//instance = SoftBody::Instance("../../data/bunny.ply");
		soft_body.push_back(instance);
		plasticDeformation = false;
	}
	//SoftBody:: Instance instance;
	std::vector<SoftBody::Instance> soft_body;
	virtual void Initialize()
	{

	
		
		int sx = 2;
		int sy = 2;
		int sz = 2;

		Vec3 lower(0.0f, 4.2f + g_params.radius*0.25f, 0.0f);

		int dimx = 5;
		int dimy = 10;
		int dimz = 5;

		float radius = g_params.radius;
		int group = 0;

		if (1)
		{
			// create a basic grid
			for (int y=0; y < dimy; ++y)
				for (int z=0; z < dimz; ++z)
					for (int x=0; x < dimx; ++x)
						CreateParticleShape(
						GetFilePathByPlatform("../../data/box.ply").c_str(), 
						(g_params.radius*0.905f)*Vec3(float(x*sx), float(y*sy), float(z*sz)) + (g_params.radius*0.1f)*Vec3(float(x),float(y),float(z)) + lower,
						g_params.radius*0.9f*Vec3(float(sx), float(sy), float(sz)), 0.0f, g_params.radius*0.9f, Vec3(0.0f), 1.0f, true, 1.0f, NvFlexMakePhase(group++, 0), true, 0.001f);

			AddPlinth();
		}

		if (1)
		{
			int dimx = 60;
			int dimy = 40;

			float stretchStiffness = 1.0f;
			float bendStiffness = 0.5f;
			float shearStiffness = 0.7f;

			int clothStart = g_buffers->positions.size();
		
			CreateSpringGrid(Vec3(0.0f, 0.0f, -1.0f), dimx, dimy, 1, radius*0.5f, NvFlexMakePhase(group++, 0), stretchStiffness, bendStiffness, shearStiffness, Vec3(0.0f), 1.0f);
		
			int corner0 = clothStart + 0;
			int corner1 = clothStart + dimx-1;
			int corner2 = clothStart + dimx*(dimy-1);
			int corner3 = clothStart + dimx*dimy-1;

			g_buffers->positions[corner0].w = 0.0f;
			g_buffers->positions[corner1].w = 0.0f;
			g_buffers->positions[corner2].w = 0.0f;
			g_buffers->positions[corner3].w = 0.0f;

			// add tethers
			for (int i=clothStart; i < int(g_buffers->positions.size()); ++i)
			{
				float x = g_buffers->positions[i].x;
				g_buffers->positions[i].y = 4.0f - sinf(DegToRad(15.0f))*x;
				g_buffers->positions[i].x = cosf(DegToRad(25.0f))*x;

				if (i != corner0 && i != corner1 && i != corner2 && i != corner3)
				{
					float stiffness = -0.5f;
					float give = 0.05f;

					CreateSpring(corner0, i, stiffness, give);			
					CreateSpring(corner1, i, stiffness, give);
					CreateSpring(corner2, i, stiffness, give);			
					CreateSpring(corner3, i, stiffness, give);
				}
			}

			g_buffers->positions[corner1] = g_buffers->positions[corner0] + (g_buffers->positions[corner1]-g_buffers->positions[corner0])*0.9f;
			g_buffers->positions[corner2] = g_buffers->positions[corner0] + (g_buffers->positions[corner2]-g_buffers->positions[corner0])*0.9f;
			g_buffers->positions[corner3] = g_buffers->positions[corner0] + (g_buffers->positions[corner3]-g_buffers->positions[corner0])*0.9f;
		}


		for (int i=0; i < 50; ++i)
			CreateParticleShape(GetFilePathByPlatform("../../data/banana.obj").c_str(), Vec3(0.4f, 8.5f + i*0.25f, 0.25f) + RandomUnitVector()*radius*0.25f, Vec3(1), 0.0f, radius, Vec3(0.0f), 1.0f, true, 0.5f, NvFlexMakePhase(group++, 0), true, radius*0.1f, 0.0f, 0.0f, 1.25f*Vec4(0.875f, 0.782f, 0.051f, 1.0f));		
		

		//g_numExtraParticles = 32*1024;		
		g_numSubsteps = 2;
		g_params.numIterations = 4;

		g_params.radius *= 1.0f;
		g_params.staticFriction = 0.7f;
		g_params.dynamicFriction = 0.75f;
		g_params.dissipation = 0.01f;
		g_params.particleCollisionMargin = g_params.radius*0.05f;
		g_params.sleepThreshold = g_params.radius*0.25f;	
		g_params.damping = 0.25f;
		g_params.maxAcceleration = 400.0f;

		g_windStrength = 0.0f;

		// draw options
		g_drawPoints = false;

		g_emitters[0].mEnabled = true;
		g_emitters[0].mSpeed = (g_params.radius*2.0f/g_dt);

		for(auto & instance :soft_body){
			CreateSoftBody(instance,1);}
	}

	virtual void Update()
	{
			
	}
	bool plasticDeformation;
	struct RenderingInstance
	{
		Mesh* mMesh;
		std::vector<int> mSkinningIndices;
		std::vector<float> mSkinningWeights;
		vector<Vec3> mRigidRestPoses;
		Vec3 mColor;
		int mOffset;
	};
	void CreateSoftBody(SoftBody::Instance instance, int group = 0)
	{
		RenderingInstance renderingInstance;

		Mesh* mesh = ImportMesh(GetFilePathByPlatform(instance.mFile).c_str());
		mesh->Normalize();
		mesh->Transform(TranslationMatrix(Point3(instance.mTranslation))*ScaleMatrix(instance.mScale*0.1f));

		renderingInstance.mMesh = mesh;
		renderingInstance.mColor = instance.mColor;
		renderingInstance.mOffset = g_buffers->rigidTranslations.size();

		double createStart = GetSeconds();

		// create soft body definition
		NvFlexExtAsset* asset = NvFlexExtCreateSoftFromMesh(
			(float*)&renderingInstance.mMesh->m_positions[0],
			renderingInstance.mMesh->m_positions.size(),
			(int*)&renderingInstance.mMesh->m_indices[0],
			renderingInstance.mMesh->m_indices.size(),
			0.1f,
			instance.mVolumeSampling,
			instance.mSurfaceSampling,
			instance.mClusterSpacing*0.1f,
			instance.mClusterRadius*0.1f,
			instance.mClusterStiffness,
			instance.mLinkRadius*0.1f,
			instance.mLinkStiffness,
			instance.mGlobalStiffness,
			instance.mClusterPlasticThreshold,
			instance.mClusterPlasticCreep);

		double createEnd = GetSeconds();

		// create skinning
		const int maxWeights = 4;

		renderingInstance.mSkinningIndices.resize(renderingInstance.mMesh->m_positions.size()*maxWeights);
		renderingInstance.mSkinningWeights.resize(renderingInstance.mMesh->m_positions.size()*maxWeights);

		for (int i = 0; i < asset->numShapes; ++i)
			renderingInstance.mRigidRestPoses.push_back(Vec3(&asset->shapeCenters[i * 3]));

		double skinStart = GetSeconds();

		NvFlexExtCreateSoftMeshSkinning(
			(float*)&renderingInstance.mMesh->m_positions[0],
			renderingInstance.mMesh->m_positions.size(),
			asset->shapeCenters,
			asset->numShapes,
			instance.mSkinningFalloff,
			instance.mSkinningMaxDistance,
			&renderingInstance.mSkinningWeights[0],
			&renderingInstance.mSkinningIndices[0]);

		double skinEnd = GetSeconds();

		printf("Created soft in %f ms Skinned in %f\n", (createEnd - createStart)*1000.0f, (skinEnd - skinStart)*1000.0f);

		const int particleOffset = g_buffers->positions.size();
		const int indexOffset = g_buffers->rigidOffsets.back();

		// add particle data to solver
		for (int i = 0; i < asset->numParticles; ++i)
		{
			g_buffers->positions.push_back(&asset->particles[i * 4]);
			g_buffers->velocities.push_back(0.0f);

			const int phase = NvFlexMakePhase(group, eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter);
			g_buffers->phases.push_back(phase);
		}

		// add shape data to solver
		for (int i = 0; i < asset->numShapeIndices; ++i)
			g_buffers->rigidIndices.push_back(asset->shapeIndices[i] + particleOffset);

		for (int i = 0; i < asset->numShapes; ++i)
		{
			g_buffers->rigidOffsets.push_back(asset->shapeOffsets[i] + indexOffset);
			g_buffers->rigidTranslations.push_back(Vec3(&asset->shapeCenters[i * 3]));
			g_buffers->rigidRotations.push_back(Quat());
			g_buffers->rigidCoefficients.push_back(asset->shapeCoefficients[i]);
		}


		// add plastic deformation data to solver, if at least one asset has non-zero plastic deformation coefficients, leave the according pointers at NULL otherwise
		if (plasticDeformation)
		{
			if (asset->shapePlasticThresholds && asset->shapePlasticCreeps)
			{
				for (int i = 0; i < asset->numShapes; ++i)
				{
					g_buffers->rigidPlasticThresholds.push_back(asset->shapePlasticThresholds[i]);
					g_buffers->rigidPlasticCreeps.push_back(asset->shapePlasticCreeps[i]);
				}
			}
			else
			{
				for (int i = 0; i < asset->numShapes; ++i)
				{
					g_buffers->rigidPlasticThresholds.push_back(0.0f);
					g_buffers->rigidPlasticCreeps.push_back(0.0f);
				}
			}
		}
		else 
		{
			if (asset->shapePlasticThresholds && asset->shapePlasticCreeps)
			{
				int oldBufferSize = g_buffers->rigidCoefficients.size() - asset->numShapes;

				g_buffers->rigidPlasticThresholds.resize(oldBufferSize);
				g_buffers->rigidPlasticCreeps.resize(oldBufferSize);

				for (int i = 0; i < oldBufferSize; i++)
				{
					g_buffers->rigidPlasticThresholds[i] = 0.0f;
					g_buffers->rigidPlasticCreeps[i] = 0.0f;
				}

				for (int i = 0; i < asset->numShapes; ++i)
				{
					g_buffers->rigidPlasticThresholds.push_back(asset->shapePlasticThresholds[i]);
					g_buffers->rigidPlasticCreeps.push_back(asset->shapePlasticCreeps[i]);
				}

				plasticDeformation = true;
			}
		}

		// add link data to the solver 
		for (int i = 0; i < asset->numSprings; ++i)
		{
			g_buffers->springIndices.push_back(asset->springIndices[i * 2 + 0]);
			g_buffers->springIndices.push_back(asset->springIndices[i * 2 + 1]);

			g_buffers->springStiffness.push_back(asset->springCoefficients[i]);
			g_buffers->springLengths.push_back(asset->springRestLengths[i]);
		}

		NvFlexExtDestroyAsset(asset);

		mRenderingInstances.push_back(renderingInstance);
	}
	std::vector<RenderingInstance> mRenderingInstances;
};

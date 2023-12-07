#include "softbody.h"

class PotPourri : public Scene
{
public:
    PotPourri(const char *name) : Scene(name), mRadius(0.1f)
    {
        SoftBody::Instance fat("../../data/fat.obj");
        fat.mScale = Vec3(32.0f);
        fat.mClusterSpacing = 1.75f;
        fat.mClusterRadius = 2.0f; //3.0f
        fat.mClusterStiffness = 1.f; //0.15f
        fat.mSurfaceSampling = 1.0f;
        soft_body.push_back(fat);

        plasticDeformation = false;
    }

    virtual void Initialize()
    {
#pragma region g_params
        float radius = mRadius;
        // no fluids or sdf based collision
        g_solverDesc.featureMode = eNvFlexFeatureModeSimpleSolids;
        g_params.radius = radius;
        g_params.dynamicFriction = 1.0f; //0.35f
        g_params.particleFriction = 1.0f; //0.25f
        g_params.numIterations = 4;
        g_params.collisionDistance = radius * 0.75f;
        g_params.relaxationFactor = 1.0f;
        g_windStrength = 0.0f;
        g_numSubsteps = 2;
        g_buffers->rigidOffsets.push_back(0);


        g_numSubsteps = 2;
        g_params.staticFriction = 1.0f; //0.7
        g_params.dissipation = 0.01f;
        g_params.particleCollisionMargin = g_params.radius * 0.05f;
        g_params.sleepThreshold = g_params.radius * 0.25f;
        g_params.damping = 0.25f;
        g_params.maxAcceleration = 400.0f;
        g_params.shockPropagation = 3.f;
        g_emitters[0].mEnabled = true;
        g_emitters[0].mSpeed = (g_params.radius * 2.0f / g_dt);
        mRenderingInstances.resize(0);
        // expand radius for better self collision
        g_params.radius *= 1.5f;
        g_lightDistance *= 1.5f;
#pragma endregion

#pragma region soft body
        // build soft bodies 
        for (const auto &i : soft_body)
            CreateSoftBody(i, 1); // NOLINT(bugprone-narrowing-conversions)
#pragma endregion

        for (int i = 0; i < g_buffers->positions.size(); ++i)
            if (g_buffers->positions[i].z < 0.1f || g_buffers->positions[i].z > 3.1f)
                g_buffers->positions[i].w = 0.0f;
    }

    void Update() override
    {
        ClearShapes();
    
        mTime += g_dt;
    
        // let cloth settle on object
        float startTime = 1.0f;
    
        float time = Max(0.0f, mTime - startTime);
        float lastTime = Max(0.0f, time - g_dt);
    
        const float rotationSpeed = 1.0f;
        const float translationSpeed = 1.0f;
    
        // Vec3 pos = Vec3(translationSpeed*(1.0f-cosf(time)), 1.0f, 1.0f);
        // Vec3 prevPos = Vec3(translationSpeed*(1.0f-cosf(lastTime)), 1.0f, 1.0f);
    
        Vec3 pos = Vec3(translationSpeed * (1.0f), 1.0f - cosf(time), 1.5f);
        Vec3 prevPos = Vec3(translationSpeed * (1.0f), 1.0f - cosf(time), 1.5f);
        //
        // Quat rot = QuatFromAxisAngle(Vec3(0.0f, 1.0f, 0.0f), 1.0f-cosf(rotationSpeed*time));
        // Quat prevRot = QuatFromAxisAngle(Vec3(0.0f, 1.0f, 0.0f), 1.0f-cosf(rotationSpeed*lastTime));
    
        Quat rot = QuatFromAxisAngle(Vec3(0.0f, 1.0f, 0.0f), 0.0f);
        Quat prevRot = QuatFromAxisAngle(Vec3(0.0f, 1.0f, 0.0f), 0.0f);
        AddCapsule(0.12f, 2.0f, pos, rot);
        g_buffers->shapePrevPositions[0] = Vec4(prevPos, 0.0f);
        g_buffers->shapePrevRotations[0] = prevRot;
    
    
        Vec3 pos2 = Vec3(translationSpeed * (0.0f - cosf(time)), 0.7f, 2.3f );
        Vec3 prevPos2 = Vec3(translationSpeed * (0.0f - cosf(time)), 0.7f, 2.3f);
    
    
        AddCapsule(0.1f, 0.5f, pos2, rot);
        g_buffers->shapePrevPositions[1] = Vec4(prevPos2,0.0f);
        g_buffers->shapePrevRotations[1] = prevRot;
        
        UpdateShapes();
    }

    NvFlexTriangleMeshId meshid;

    float mRadius;

    std::vector<SoftBody::Instance> soft_body;

private:
    struct RenderingInstance
    {
        Mesh *mMesh;
        std::vector<int> mSkinningIndices;
        std::vector<float> mSkinningWeights;
        vector<Vec3> mRigidRestPoses;
        Vec3 mColor;
        int mOffset;
    };

    std::vector<RenderingInstance> mRenderingInstances;

    void CreateSoftBody(SoftBody::Instance instance, int group = 0)
    {
        RenderingInstance renderingInstance;

        Mesh *mesh = ImportMesh(GetFilePathByPlatform(instance.mFile).c_str());
        mesh->Normalize();
        mesh->Transform(TranslationMatrix(Point3(instance.mTranslation)) * ScaleMatrix(instance.mScale * 0.1f));

        renderingInstance.mMesh = mesh;
        renderingInstance.mColor = instance.mColor;
        renderingInstance.mOffset = g_buffers->rigidTranslations.size();

        double createStart = GetSeconds();

        // create soft body definition
        NvFlexExtAsset *asset = NvFlexExtCreateSoftFromMesh(
            (float *)&renderingInstance.mMesh->m_positions[0],
            renderingInstance.mMesh->m_positions.size(),
            (int *)&renderingInstance.mMesh->m_indices[0],
            renderingInstance.mMesh->m_indices.size(),
            0.1f,
            instance.mVolumeSampling,
            instance.mSurfaceSampling,
            instance.mClusterSpacing * 0.1f,
            instance.mClusterRadius * 0.1f,
            instance.mClusterStiffness,
            instance.mLinkRadius * 0.1f,
            instance.mLinkStiffness,
            instance.mGlobalStiffness,
            instance.mClusterPlasticThreshold,
            instance.mClusterPlasticCreep);

        double createEnd = GetSeconds();

        // create skinning
        const int maxWeights = 4;

        renderingInstance.mSkinningIndices.resize(renderingInstance.mMesh->m_positions.size() * maxWeights);
        renderingInstance.mSkinningWeights.resize(renderingInstance.mMesh->m_positions.size() * maxWeights);

        for (int i = 0; i < asset->numShapes; ++i)
            renderingInstance.mRigidRestPoses.push_back(Vec3(&asset->shapeCenters[i * 3]));

        double skinStart = GetSeconds();

        NvFlexExtCreateSoftMeshSkinning(
            (float *)&renderingInstance.mMesh->m_positions[0],
            renderingInstance.mMesh->m_positions.size(),
            asset->shapeCenters,
            asset->numShapes,
            instance.mSkinningFalloff,
            instance.mSkinningMaxDistance,
            &renderingInstance.mSkinningWeights[0],
            &renderingInstance.mSkinningIndices[0]);

        double skinEnd = GetSeconds();

        printf("Created soft in %f ms Skinned in %f\n", (createEnd - createStart) * 1000.0f,
               (skinEnd - skinStart) * 1000.0f);

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

    bool plasticDeformation;
    float mTime;
};

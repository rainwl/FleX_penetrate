#include "softbody.h"

class PotPourri : public Scene
{
public:
    PotPourri(const char *name) : Scene(name), mRadius(0.1f)
    {
        // SoftBody::Instance octopus("../../data/softs/octopus.obj");
        SoftBody::Instance octopus("../../data/fat.obj");
        octopus.mScale = Vec3(32.0f);
        octopus.mClusterSpacing = 1.75f;
        octopus.mClusterRadius = 2.0f; //3.0f
        octopus.mClusterStiffness = 0.5f; //0.15f
        octopus.mSurfaceSampling = 1.0f;
        soft_body.push_back(octopus);

        plasticDeformation = false;
    }

    virtual void Initialize()
    {
#pragma region g_params
        float radius = mRadius;
        // no fluids or sdf based collision
        g_solverDesc.featureMode = eNvFlexFeatureModeSimpleSolids;
        g_params.radius = radius;
        g_params.dynamicFriction = 0.35f;
        g_params.particleFriction = 0.25f;
        g_params.numIterations = 4;
        g_params.collisionDistance = radius * 0.75f;
        g_params.relaxationFactor = 1.0f;
        g_windStrength = 0.0f;
        g_numSubsteps = 2;
        g_buffers->rigidOffsets.push_back(0);


        g_numSubsteps = 2;
        g_params.staticFriction = 0.7f;
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

#pragma region mesh
        int group = 2;
        Mesh *bowl = ImportMesh(GetFilePathByPlatform("../../data/bowl.obj").c_str());
        bowl->Normalize(2.0f);
        bowl->CalculateNormals();
        bowl->Transform(TranslationMatrix(Point3(-1.0f, 0.0f, -1.0f)));

        //NvFlexTriangleMeshId mesh = CreateTriangleMesh(bowl);
        meshid = CreateTriangleMesh(bowl);
        AddTriangleMesh(meshid, Vec3(), Quat(), 1.0f);

        delete bowl;
#pragma endregion

#pragma region rigid
        // Vec3 lower(0.0f, 1.5f + g_params.radius * 0.25f, 0.0f);
        //
        //
        // int dimx = 3;
        // int dimy = 3;
        // int dimz = 3;
        // int sx = 2;
        // int sy = 4;
        // int sz = 2;
        //
        // Mesh *mesh_r = ImportMesh(GetFilePathByPlatform("../../data/box.ply").c_str());
        //
        // // create a basic grid
        // for (int y = 0; y < dimy; ++y)
        //     for (int z = 0; z < dimz; ++z)
        //         for (int x = 0; x < dimx; ++x)
        //             CreateParticleShape(
        //                 mesh_r,
        //                 (g_params.radius * 0.905f) * Vec3(float(x * sx), float(y * sy), float(z * sz)) + (g_params.
        //                     radius * 0.1f) * Vec3(float(x), float(y), float(z)) + lower,
        //                 g_params.radius * 0.9f * Vec3(float(sx), float(sy), float(sz)), 0.0f, g_params.radius * 0.9f,
        //                 Vec3(0.0f), 1.0f, true, 1.0f, NvFlexMakePhase(g_buffers->rigidOffsets.size() + 1, 0), false,
        //                 0.002f); // 0.002f);
        //
        //
        // delete mesh_r;
#pragma endregion

#pragma region sdf

        // float maxShapeRadius = 0.25f;
        // float minShapeRadius = 0.1f;
        //
        // NvFlexDistanceFieldId sdf = CreateSDF(GetFilePathByPlatform("../../data/bunny.ply").c_str(), 128);
        //
        // for (int i = 0; i < dimx; ++i)
        // {
        //     for (int j = 0; j < dimy; ++j)
        //     {
        //         for (int k = 0; k < dimz; ++k)
        //         {
        //             int type = Rand() % 6;
        //
        //             Vec3 shapeTranslation = Vec3(float(i), float(j) + 0.5f, float(k)) * maxShapeRadius * 2.0f;
        //             Quat shapeRotation = QuatFromAxisAngle(UniformSampleSphere(), Randf() * k2Pi);
        //
        //             switch (type)
        //             {
        //             case 0:
        //                 {
        //                     AddSphere(Randf(minShapeRadius, maxShapeRadius), shapeTranslation, shapeRotation);
        //                     break;
        //                 }
        //             case 1:
        //                 {
        //                     AddCapsule(Randf(minShapeRadius, maxShapeRadius) * 0.5f, Randf() * maxShapeRadius,
        //                                shapeTranslation, shapeRotation);
        //                     break;
        //                 }
        //             case 2:
        //                 {
        //                     Vec3 extents = 0.75f * Vec3(Randf(minShapeRadius, maxShapeRadius),
        //                                                 Randf(minShapeRadius, maxShapeRadius),
        //                                                 Randf(minShapeRadius, maxShapeRadius));
        //
        //                     AddBox(extents, shapeTranslation, shapeRotation);
        //                     break;
        //                 }
        //             case 3:
        //                 {
        //                     AddRandomConvex(6 + Rand() % 6, shapeTranslation, minShapeRadius, maxShapeRadius,
        //                                     UniformSampleSphere(), Randf() * k2Pi);
        //                     break;
        //                 }
        //             case 4:
        //                 {
        //                     AddTriangleMesh(meshid, shapeTranslation, shapeRotation, Randf(0.5f, 1.0f) * maxShapeRadius);
        //                     break;
        //                 }
        //             case 5:
        //                 {
        //                     AddSDF(sdf, shapeTranslation, shapeRotation, maxShapeRadius * 2.0f);
        //                     break;
        //                 }
        //             };
        //         }
        //     }
        // }
        //
        //
        // if (1)
        // {
        //     int dimx = 60;
        //     int dimy = 40;
        //
        //     float stretchStiffness = 1.0f;
        //     float bendStiffness = 0.5f;
        //     float shearStiffness = 0.7f;
        //
        //     int clothStart = g_buffers->positions.size();
        //
        //     CreateSpringGrid(Vec3(0.0f, 0.0f, -1.0f), dimx, dimy, 1, radius * 0.5f, NvFlexMakePhase(group++, 0),
        //                      stretchStiffness, bendStiffness, shearStiffness, Vec3(0.0f), 1.0f);
        //
        //     int corner0 = clothStart + 0;
        //     int corner1 = clothStart + dimx - 1;
        //     int corner2 = clothStart + dimx * (dimy - 1);
        //     int corner3 = clothStart + dimx * dimy - 1;
        //
        //     g_buffers->positions[corner0].w = 0.0f;
        //     g_buffers->positions[corner1].w = 0.0f;
        //     g_buffers->positions[corner2].w = 0.0f;
        //     g_buffers->positions[corner3].w = 0.0f;
        //
        //     // add tethers
        //     for (int i = clothStart; i < int(g_buffers->positions.size()); ++i)
        //     {
        //         float x = g_buffers->positions[i].x;
        //         g_buffers->positions[i].y = 4.0f - sinf(DegToRad(15.0f)) * x;
        //         g_buffers->positions[i].x = cosf(DegToRad(25.0f)) * x;
        //
        //         if (i != corner0 && i != corner1 && i != corner2 && i != corner3)
        //         {
        //             float stiffness = -0.5f;
        //             float give = 0.05f;
        //
        //             CreateSpring(corner0, i, stiffness, give);
        //             CreateSpring(corner1, i, stiffness, give);
        //             CreateSpring(corner2, i, stiffness, give);
        //             CreateSpring(corner3, i, stiffness, give);
        //         }
        //     }
        //
        //     g_buffers->positions[corner1] = g_buffers->positions[corner0] + (g_buffers->positions[corner1] - g_buffers->
        //         positions[corner0]) * 0.9f;
        //     g_buffers->positions[corner2] = g_buffers->positions[corner0] + (g_buffers->positions[corner2] - g_buffers->
        //         positions[corner0]) * 0.9f;
        //     g_buffers->positions[corner3] = g_buffers->positions[corner0] + (g_buffers->positions[corner3] - g_buffers->
        //         positions[corner0]) * 0.9f;
        // }
        // // fix any particles below the ground plane in place
        // for (int i = 0; i < int(g_buffers->positions.size()); ++i)
        //     if (g_buffers->positions[i].y < 0.0f)
        //         g_buffers->positions[i].w = 0.0f;
#pragma endregion

        for (int i = 0; i < int(g_buffers->positions.size()); ++i)
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


        Vec3 pos2 = Vec3(translationSpeed * (0.0f - cosf(time)), 0.8f, 2.2f );
        Vec3 prevPos2 = Vec3(translationSpeed * (0.0f - cosf(time)), 0.8f, 2.2f);


        AddCapsule(0.2f, 0.5f, pos2, rot);
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

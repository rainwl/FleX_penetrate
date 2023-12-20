#include "softbody.h"

class PotPourri : public Scene
{
public:
    SoftBody *soft_body;

    PotPourri(const char *name) : Scene(name)
    {
        if (true)
        {
            SoftBody::Instance bowl("../../data/bowl_high.ply");
            bowl.mScale = Vec3(10.0f);
            bowl.mClusterSpacing = 2.0f;
            bowl.mClusterRadius = 2.0f;
            bowl.mClusterStiffness = 0.55f;
            soft_body = new SoftBody("Soft Bowl");
            soft_body->AddInstance(bowl);
        }
    }

    void Initialize() override
    {
        if (true)
        {
            soft_body->Initialize();
        }
    }
};

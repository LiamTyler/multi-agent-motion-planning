#pragma once

#include "include/Progression.h"

class PRM;

class PRMRenderer : public RenderComponent {
    public:
        PRMRenderer(PRM* prm, Mesh* mesh, Material* mat, const std::string& shaderID);
        ~PRMRenderer();

        void Start();
        void Update(float dt);
        void Stop();
        void Render(Shader& shader, const Camera& camera);

    protected:
        Mesh* mesh_;
        PRM* prm_;
};

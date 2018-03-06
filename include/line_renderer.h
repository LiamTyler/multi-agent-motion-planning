#pragma once

#include "include/Progression.h"

class PRM;

class LineRenderer : public RenderComponent {
    public:
        LineRenderer();
        LineRenderer(glm::vec4 color, glm::vec3* lines_, int numLines_,
                int lineWidth, const std::string& shaderID);
        ~LineRenderer();

        void Start();
        void UploadData(glm::vec3* lines, int numlines);
        void Update(float dt);
        void Stop();
        void Render(Shader& shader, const Camera& camera);

    protected:
        glm::vec4 color_;
        glm::vec3* lines_;
        int numLines_;
        int lineWidth_;
        GLuint vao;
        GLuint* vbos;
};

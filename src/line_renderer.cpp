#include "include/line_renderer.h"

LineRenderer::LineRenderer() : LineRenderer(
        glm::vec4(0, 1, 0, 1),
        nullptr,
        0,
        2,
        "lineShader") {}

LineRenderer::LineRenderer(glm::vec4 color, glm::vec3* lines, int numLines,
        int lineWidth, const std::string& shaderID) : RenderComponent(nullptr, shaderID)
{
    color_ = color;
    lines_ = lines;
    numLines_ = numLines;
    lineWidth_ = lineWidth;
}

LineRenderer::~LineRenderer() {}

void LineRenderer::Start() {
    Shader& shader = *renderer->GetShader(shaderID_);
    vao = renderer->CreateNonResourceVao(shaderID_, this);
    vbos = renderer->CreateVbos(1);
    glBindVertexArray(vao);


    // vertices
    glBindBuffer(GL_ARRAY_BUFFER, vbos[0]);
    glBufferData(GL_ARRAY_BUFFER, 2 * numLines_ * sizeof(glm::vec3),
            lines_, GL_STATIC_DRAW);
    glEnableVertexAttribArray(shader["vertex"]);
    glVertexAttribPointer(shader["vertex"], 3, GL_FLOAT, GL_FALSE, 0, 0);
}

void LineRenderer::UploadData(glm::vec3* lines, int numlines) {
    lines_ = lines;
    numLines_ = numlines;
    glBindBuffer(GL_ARRAY_BUFFER, vbos[0]);
    glBufferData(GL_ARRAY_BUFFER, 2 * numLines_ * sizeof(glm::vec3),
            lines_, GL_STATIC_DRAW);
}
    
void LineRenderer::Update(float dt) {
}

void LineRenderer::Stop() {
}

void LineRenderer::Render(Shader& shader, const Camera& camera) {
    glUniform4fv(shader["color"], 1, glm::value_ptr(color_));
    glUniformMatrix4fv(shader["MVP"], 1, GL_FALSE, glm::value_ptr(camera.GetP() * camera.GetV()));
    glLineWidth(lineWidth_);

    glDrawArrays(GL_LINES, 0, numLines_);
}

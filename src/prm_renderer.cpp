#include "include/prm_renderer.h"
#include "include/prm.h"

PRMRenderer::PRMRenderer(PRM* prm, Mesh* mesh, Material* mat,
        const std::string& shaderID) : RenderComponent(mat, shaderID)
{
    prm_ = prm;
    mesh_ = mesh;
}

PRMRenderer::~PRMRenderer() {}

void PRMRenderer::Start() {
    Shader& shader = *renderer->GetShader(shaderID_);
    RID id = mesh_->rid;
    if (!renderer->VaoExists(id)) {
        GLuint vao = renderer->CreateAndRegisterVao(shaderID_, id, this);
        GLuint* vbos = renderer->CreateVbos(3);
        glBindVertexArray(vao);

        // vertices
        glBindBuffer(GL_ARRAY_BUFFER, vbos[0]);
        glBufferData(GL_ARRAY_BUFFER, mesh_->GetNumVertices() * sizeof(glm::vec3),
                mesh_->GetVertices(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(shader["vertex"]);
        glVertexAttribPointer(shader["vertex"], 3, GL_FLOAT, GL_FALSE, 0, 0);

        // normals
        glBindBuffer(GL_ARRAY_BUFFER, vbos[1]);
        glBufferData(GL_ARRAY_BUFFER, mesh_->GetNumVertices() * sizeof(glm::vec3),
                mesh_->GetNormals(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(shader["normal"]);
        glVertexAttribPointer(shader["normal"], 3, GL_FLOAT, GL_FALSE, 0, 0);

        // indices
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos[2]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh_->GetNumTriangles() * sizeof(glm::ivec3),
                mesh_->GetIndices(), GL_STATIC_DRAW);
    } else {
        renderer->Register(id, this);
    }
}

void PRMRenderer::Update(float dt) {
}

void PRMRenderer::Stop() {
}

void PRMRenderer::Render(Shader& shader, glm::mat4& V) {
    // send material
    glUniform4fv(shader["ka"], 1, value_ptr(material_->ka));
    glUniform4fv(shader["kd"], 1, value_ptr(material_->kd));
    glUniform4fv(shader["ks"], 1, value_ptr(material_->ks));
    glUniform1f(shader["specular"], material_->specular);
    if (material_->diffuseTex) {
        glUniform1i(shader["textured"], true);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, material_->diffuseTex->GetHandle());
        glUniform1i(shader["diffuseTex"], 0);
    }

    for (int i = 0; i < prm_->nodes_.size(); i++) {
        glm::mat4 model(1);
        glm::vec3 pos = prm_->nodes_[i]->position;
        pos.y = .1;
        model = glm::translate(model, pos);
        float radius = .2;
        model = glm::scale(model, glm::vec3(radius, 1, radius));
        glm::mat4 MV = V * model;
        glm::mat4 normalMatrix = glm::transpose(glm::inverse(MV));
        glUniformMatrix4fv(shader["modelViewMatrix"], 1, GL_FALSE, value_ptr(MV));
        glUniformMatrix4fv(shader["normalMatrix"], 1, GL_FALSE, value_ptr(normalMatrix));

        glDrawElements(GL_TRIANGLES, mesh_->GetNumTriangles() * 3, GL_UNSIGNED_INT, 0);
    }
}

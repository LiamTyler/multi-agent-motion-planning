#include "include/Progression.h"

using namespace std;

int main() {
    InitEngine();
    Window window("Starter Project", 800, 600);

    Material* cubeMat = resourceManager->AllocateResource<Material>(Material(
            glm::vec4(1.0, .4, .4, 1),
            glm::vec4(1.0, .4, .4, 1),
            glm::vec4(.6, .6,  .6, 1),
            50));
    Mesh* mesh = resourceManager->LoadMesh("models/sphere.obj");

    GameObject Bjorn = GameObject(Transform(glm::vec3(0, 0, -5)));
    Bjorn.AddComponent<MeshRenderer>(new MeshRenderer(mesh, cubeMat, "meshShader"));

    Camera camera = Camera();
    camera.AddComponent<CameraController>(new CameraController(4, .005));

    bool quit = false;
    window.SetRelativeMouse(true);
    while (!quit) {
        window.StartFrame();
        quit = input->HandleInput();
        if (input->KeyPressed(K_ESC))
            quit = true;

        float dt = window.GetDT();
        camera.Update(dt);
        Bjorn.Update(dt);
        renderer->RenderScene(camera);

        window.EndFrame();
    }

    QuitEngine();
    return 0;
}

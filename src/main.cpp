#include "include/Progression.h"

using namespace std;

int main() {
    InitEngine();
    Window window("Starter Project", 800, 600);

    Material* redMat = resourceManager->AllocateResource<Material>(Material(
            glm::vec4(1.0, .4, .4, 1),
            glm::vec4(1.0, .4, .4, 1),
            glm::vec4(.6, .6,  .6, 1),
            50));
    Material* grayMat = resourceManager->AllocateResource<Material>(Material(
            glm::vec4(.4, .4, .4, 1),
            glm::vec4(.4, .4, .4, 1),
            glm::vec4(.6, .6,  .6, 1),
            50));
    Material* blueMat = resourceManager->AllocateResource<Material>(Material(
            glm::vec4(.3, .3, 1.0, 1),
            glm::vec4(.3, .3, 1.0, 1),
            glm::vec4(.6, .6,  .6, 1),
            50));
    Mesh* planeMesh = resourceManager->LoadMesh("models/plane.obj");
    Mesh* cylinderMesh = resourceManager->LoadMesh("models/cylinder.obj");

    GameObject floor;
    floor.transform.scale = glm::vec3(10, 1, 10);
    floor.AddComponent<MeshRenderer>(new MeshRenderer(planeMesh, grayMat, "meshShader"));

    GameObject obstacle;
    obstacle.AddComponent<MeshRenderer>(new MeshRenderer(cylinderMesh, redMat, "meshShader"));
    obstacle.transform.position.y = 1;
    obstacle.transform.scale = glm::vec3(2, 1, 2);

    GameObject agent;
    agent.AddComponent<MeshRenderer>(new MeshRenderer(cylinderMesh, blueMat, "meshShader"));
    agent.transform.position = glm::vec3(-9, 0, 9);
    agent.transform.scale = glm::vec3(.25, 1, .25);

    Camera camera = Camera();
    camera.AddComponent<CameraController>(new CameraController(8, .005));
    camera.transform.rotation.x = glm::radians(-90.0f);
    camera.transform.position = glm::vec3(0, 20, 0);

    bool quit = false;
    window.SetRelativeMouse(true);
    while (!quit) {
        window.StartFrame();
        quit = input->HandleInput();
        if (input->KeyPressed(K_ESC))
            quit = true;

        float dt = window.GetDT();
        camera.Update(dt);
        floor.Update(dt);
        obstacle.Update(dt);
        agent.Update(dt);
        renderer->RenderScene(camera);

        window.EndFrame();
    }

    QuitEngine();
    return 0;
}

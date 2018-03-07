#include "include/Progression.h"
#include "include/cspace.h"
#include "include/prm.h"
#include "include/prm_renderer.h"
#include "include/line_renderer.h"
#include "include/path.h"

using namespace std;

CSpace GenerateCSpace(vector<GameObject*>& obstacles) {
    Transform boundaries = Transform(
            glm::vec3(0, 0, 0),
            glm::vec3(0, 0, 0),
            glm::vec3(20, 1, 20));
    return CSpace(boundaries, &obstacles, .5);
}

PRM* GeneratePRM(CSpace& c, int samples, float neighbor_radius) {
    PRM* prm = new PRM(&c, neighbor_radius);
    prm->GeneratePRM(samples);
    return prm;
} 


int main() {
    InitEngine();
    srand(time(NULL));
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
    agent.transform.scale = glm::vec3(.5, 1, .5);

    Camera camera = Camera();
    camera.AddComponent<CameraController>(new CameraController(8, .005));
    camera.transform.rotation.x = glm::radians(-90.0f);
    camera.transform.position = glm::vec3(0, 20, 0);

    renderer->AddShader("lineShader", "shaders/line_shader.vert",
                                      "shaders/line_shader.frag", "");
    std::vector<GameObject*> obstacles;
    obstacles.push_back(&obstacle);
    CSpace cspace = GenerateCSpace(obstacles);
    PRM* prm = GeneratePRM(cspace, 50, 10);
    prm->nodeRenderer = new PRMRenderer(prm, planeMesh, blueMat, "meshShader");
    prm->nodeRenderer->Start();
    glm::vec3* lines = prm->GetLines();
    int numLines = prm->GetNumLines();
    prm->lineRenderer = new LineRenderer;
    prm->lineRenderer->Start();
    prm->lineRenderer->UploadData(lines, numLines);

    Path path(&cspace, prm);
    if (path.GeneratePath(agent.transform.position, glm::vec3(9, 0, -9)))
        cout << "path found" << endl;
    else
        cout << "NO path found" << endl;

    std::vector<PRMNode*> agentPath = path.path_;
    glm::vec3 diff = agentPath[0]->position - agent.transform.position;
    int currentNode = 0;

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
        if (currentNode != agentPath.size()) {
            glm::vec3 currPos = agent.transform.position;
            glm::vec3 goalPos = agentPath[currentNode]->position;
            currPos += diff * dt * .2;
            agent.transform.position = currPos;
            if (glm::length(goalPos - currPos) < 0.05) {
                currentNode += 1;
                if (currentNode != agentPath.size())
                    diff = agentPath[currentNode]->position - currPos;
            }
        }

        renderer->RenderScene(camera);

        window.EndFrame();
    }

    QuitEngine();
    return 0;
}

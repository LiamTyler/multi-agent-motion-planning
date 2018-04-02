#include "include/Progression.h"
#include "include/cspace.h"
#include "include/prm.h"
#include "include/prm_renderer.h"
#include "include/line_renderer.h"
#include "include/nav_agent.h"

using namespace std;

std::vector<NavAgent*> navAgentList;
std::vector<GameObject*> obstacles;

CSpace GenerateCSpace(vector<GameObject*>& obstacles) {
    Transform boundaries = Transform(
            glm::vec3(0, 0, 0),
            glm::vec3(0, 0, 0),
            // glm::vec3(20, 1, 20));
            glm::vec3(32, 1, 18));
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
    Material* boidMat = resourceManager->AllocateResource<Material>(Material(
            glm::vec4(.3, 1.0, .3, 1),
            glm::vec4(.3, 1.0, 0.3, 1),
            glm::vec4(.6, .6,  .6, 1),
            50));
    Mesh* planeMesh = resourceManager->LoadMesh("models/plane.obj");
    Mesh* triangleMesh = resourceManager->LoadMesh("models/triangle.obj");
    Mesh* cylinderMesh = resourceManager->LoadMesh("models/cylinder.obj");

    GameObject floor;
    floor.transform.scale = 1.1 * glm::vec3(16, 1, 9);
    floor.AddComponent<MeshRenderer>(new MeshRenderer(planeMesh, grayMat, "meshShader"));

    std::vector<glm::vec3> opos;
    opos.push_back(glm::vec3(0, 1, 0));
    opos.push_back(glm::vec3(-.5, 1, -1));
    opos.push_back(glm::vec3(-1.5, 1, -2));
    opos.push_back(glm::vec3(-2.5, 1, -2.5));
    opos.push_back(glm::vec3(-3.5, 1, -2.75));
    opos.push_back(glm::vec3(.5, 1, 1));
    opos.push_back(glm::vec3(.75, 1, 2));
    float r = 1;
    for (int i = 0; i < opos.size(); i++) {
        GameObject* obstacle = new GameObject;
        obstacle->AddComponent<MeshRenderer>(new MeshRenderer(cylinderMesh, redMat, "meshShader"));
        obstacle->transform.position = opos[i];
        obstacle->transform.scale = glm::vec3(r, 1, r);
        obstacles.push_back(obstacle);
    }

    std::vector<GameObject*> agents;
    std::vector<glm::vec3> starts;
    starts.push_back(glm::vec3(-9, 0, 9));
    starts.push_back(glm::vec3(9, 0, 9));
    starts.push_back(glm::vec3(9, 0, -9));
    starts.push_back(glm::vec3(-9, 0, -9));
    // for (int i = 0; i < starts.size(); i++) {
    for (int i = 0; i < 150; i++) {
        GameObject* agent = new GameObject;
        agent->AddComponent<MeshRenderer>(new MeshRenderer(triangleMesh, boidMat, "meshShader"));
        agent->transform.position = glm::vec3(-15, 0, 8);
        // agent->transform.position = starts[i];
        agent->transform.position.y = 1;
        agent->transform.scale = glm::vec3(.5, 1, .5);
        agents.push_back(agent);
    }

    Camera camera = Camera();
    camera.AddComponent<CameraController>(new CameraController(8, .005));
    camera.transform.rotation.x = glm::radians(-90.0f);
    camera.transform.position = glm::vec3(0, 20, 0);

    renderer->AddShader("lineShader", "shaders/line_shader.vert",
                                      "shaders/line_shader.frag", "");
    CSpace cspace = GenerateCSpace(obstacles);
    PRM* prm = GeneratePRM(cspace, 50, 10);
    prm->nodeRenderer = new PRMRenderer(prm, planeMesh, blueMat, "meshShader");
    prm->nodeRenderer->Start();
    // prm->lineRenderer = new LineRenderer;
    // prm->lineRenderer->Start();

    // glm::vec3* lines = prm->GetLines();
    // int numLines = prm->GetNumLines();
    // prm->lineRenderer->UploadData(lines, numLines);

    for (int i = 0; i < agents.size(); i++) {
        GameObject* agent = agents[i];
        agent->AddComponent<NavAgent>(new NavAgent(prm, 4, 3));
        // agent->GetComponent<NavAgent>()->SetGoal(-starts[i]);
        // agent->GetComponent<NavAgent>()->FindPath();
        agent->GetComponent<NavAgent>()->SetGoal(glm::vec3(15, 0, -8));
        agent->GetComponent<NavAgent>()->FindPath();
        agent->GetComponent<NavAgent>()->active = true;
        navAgentList.push_back(agent->GetComponent<NavAgent>());
    }

    bool quit = false;
    bool paused = false;
    window.SetRelativeMouse(true);
    float boidDT = 1.0 / 60;
    float boidTime = 0;
    while (!quit) {
        window.StartFrame();
        quit = input->HandleInput();
        if (input->KeyPressed(K_ESC))
            quit = true;
        if (input->KeyPressed(K_P))
            paused = !paused;

        float dt = window.GetDT();
        boidTime += dt;
        camera.Update(dt);
        if (!paused) {
            floor.Update(dt);
            // fixed boid update at 60 fps
            if (boidTime > boidDT) {
                for (auto& agent : agents) {
                    agent->Update(boidDT);
                }
                boidTime = 0;
            }
        }

        renderer->RenderScene(camera);

        window.EndFrame();
    }

    QuitEngine();
    return 0;
}

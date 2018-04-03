#include "include/Progression.h"
#include "include/cspace.h"
#include "include/prm.h"
#include "include/prm_renderer.h"
#include "include/line_renderer.h"
#include "include/nav_agent.h"

using namespace std;

std::vector<NavAgent*> navAgentList;
std::vector<GameObject*> obstacles;

glm::vec3 intersectPlane(const glm::vec3& rayPoint, const glm::vec3& rayDir,
        const glm::vec3& planeP, const glm::vec3& planeN) {
    float t = glm::dot(planeP - rayPoint, planeN) / glm::dot(rayDir, planeN);
    return rayPoint + t * rayDir;
}

glm::vec3 GetWorldPos(Camera* camera) {
    float x = input->mouse.x;
    float y = input->mouse.y;
    float nsx = 2 * x / 800 - 1;
    float nsy = 1 - 2 * y / 600;
    glm::vec4 ray_clip = glm::vec4(nsx, nsy, -1, 1);
    glm::vec4 ray_eye = glm::inverse(camera->GetP()) * ray_clip;
    ray_eye = glm::vec4(ray_eye.x, ray_eye.y, -1, 0);
    glm::vec3 ray_world = glm::normalize(glm::vec3(camera->GetV() * ray_eye));
    ray_world.y *= -1;
    ray_world.z *= -1;
    glm::vec3 ret = intersectPlane(camera->transform.position, ray_world,
            glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
    return ret;
}

CSpace GenerateCSpace(vector<GameObject*>& obstacles) {
    Transform boundaries = Transform(
            glm::vec3(0, 0, 0),
            glm::vec3(0, 0, 0),
            glm::vec3(32, 1, 18));
    return CSpace(boundaries, &obstacles, .5);
}

PRM* GeneratePRM(CSpace& c, int samples, float neighbor_radius) {
    PRM* prm = new PRM(&c, neighbor_radius);
    prm->GeneratePRM(samples);
    return prm;
} 

void RedoAgents() {
    // if any agent is in an obstacle right now, move him out, and plan new paths
    for (int i = 0; i < navAgentList.size(); i++) {
        GameObject* agent = navAgentList[i]->gameObject;
        glm::vec3 aPos = agent->transform.position;
        aPos.y = 0;
        for (int ii = 0; ii < obstacles.size(); ii++) {
            glm::vec3 oPos = obstacles[ii]->transform.position;
            oPos.y = 0;
            // float r = 1.5;
            float r = obstacles[ii]->transform.scale.x + agent->transform.scale.x;
            glm::vec3 dir = aPos - oPos;
            float l = glm::length(dir);
            if (l <= r) {
                aPos = oPos + (r + 0.01) * glm::normalize(dir);
                agent->transform.position.x = aPos.x;
                agent->transform.position.z = aPos.z;
            }
        }
        agent->GetComponent<NavAgent>()->FindPath();
    }
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
    Mesh* triangleMesh2 = resourceManager->LoadMesh("models/triangleMesh.obj");
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
    float obstacle_r = 1;
    for (int i = 0; i < opos.size(); i++) {
        GameObject* obstacle = new GameObject;
        obstacle->AddComponent<MeshRenderer>(new MeshRenderer(cylinderMesh, redMat, "meshShader"));
        obstacle->transform.position = opos[i];
        obstacle->transform.scale = glm::vec3(obstacle_r, 1, obstacle_r);
        obstacles.push_back(obstacle);
    }

    std::vector<GameObject*> agents;
    std::vector<glm::vec3> starts;
    starts.push_back(glm::vec3(-9, 0, 9));
    starts.push_back(glm::vec3(9, 0, 9));
    starts.push_back(glm::vec3(9, 0, -9));
    starts.push_back(glm::vec3(-9, 0, -9));
    // for (int i = 0; i < starts.size(); i++) {
    for (int i = 0; i < 200; i++) {
        GameObject* agent = new GameObject;
        agent->AddComponent<MeshRenderer>(new MeshRenderer(triangleMesh2, boidMat, "meshShader"));
        agent->transform.position = glm::vec3(-15, 0, 8);
        // agent->transform.position = starts[i];
        agent->transform.position.y = 1;
        agent->transform.scale = glm::vec3(.5, 1, .5);
        agents.push_back(agent);
    }

    Transform t(glm::vec3(0, 40, 0), glm::vec3(glm::radians(-90.0f), 0, 0), glm::vec3(1));
    Camera camera = Camera(t);

    renderer->AddShader("lineShader", "shaders/line_shader.vert",
                                      "shaders/line_shader.frag", "");
    CSpace cspace = GenerateCSpace(obstacles);
    PRM* prm = GeneratePRM(cspace, 20, 10);
    prm->nodeRenderer = new PRMRenderer(prm, planeMesh, blueMat, "meshShader");
    prm->nodeRenderer->Start();
    prm->lineRenderer = new LineRenderer;
    prm->lineRenderer->Start();

    glm::vec3* lines = prm->GetLines();
    int numLines = prm->GetNumLines();
    prm->lineRenderer->UploadData(lines, numLines);

    for (int i = 0; i < agents.size(); i++) {
        GameObject* agent = agents[i];
        agent->AddComponent<NavAgent>(new NavAgent(prm, 4, 3));
        // agent->GetComponent<NavAgent>()->SetGoal(-starts[i]);
        // agent->GetComponent<NavAgent>()->FindPath();
        agent->GetComponent<NavAgent>()->SetGoal(glm::vec3(15, 0, -8));
        agent->GetComponent<NavAgent>()->FindPath();
        navAgentList.push_back(agent->GetComponent<NavAgent>());
    }

    bool quit = false;
    bool paused = false;
    // window.SetRelativeMouse(true);
    float boidDT = 1.0 / 200;
    float boidTime = 0;
    while (!quit) {
        window.StartFrame();
        quit = input->HandleInput();
        if (input->KeyPressed(K_ESC))
            quit = true;
        if (input->KeyPressed(K_P))
            paused = !paused;
        if (input->KeyPressed(M_LEFT)) {
            glm::vec3 newGoal = GetWorldPos(&camera);
            for (int i = 0; i < agents.size(); i++) {
                GameObject* agent = agents[i];
                agent->GetComponent<NavAgent>()->SetGoal(newGoal);
            }
            RedoAgents();
        }
        if (input->KeyPressed(M_RIGHT)) {
            glm::vec3 pos = GetWorldPos(&camera);
            pos.y = 1;
            GameObject* obstacle = new GameObject;
            obstacle->AddComponent<MeshRenderer>(new MeshRenderer(cylinderMesh, redMat, "meshShader"));
            obstacle->transform.position = pos;
            obstacle->transform.scale = glm::vec3(obstacle_r, 1, obstacle_r);
            obstacles.push_back(obstacle);
            prm->ClearPRM();
            prm->GeneratePRM(70);
            lines = prm->GetLines();
            numLines = prm->GetNumLines();
            prm->lineRenderer->UploadData(lines, numLines);
            RedoAgents();
        }

        float dt = window.GetDT();
        boidTime += dt;
        camera.Update(dt);
        if (!paused) {
            if (boidTime > boidDT) {
                for (auto& agent : agents) {
                    agent->Update(0.007);
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
